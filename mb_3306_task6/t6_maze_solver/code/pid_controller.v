module pid_controller(
    input clk,
    input rst_n,
    input sample_enable,          // Pulse to trigger PID calculation
    input clk_adj,                // Unused but kept for compatibility
    input signed [31:0] target_val,      // Desired value (e.g. speed in ticks/sample)
    input signed [31:0] current_val,     // Actual value
    output reg [7:0] pwm_out      // Output drive (0-255)
);

    // Default PID Constants (Can be overridden when instantiating)
    parameter Kp = 4;
    parameter Ki = 0; 
    parameter Kd = 1;

    (* keep = "true" *) reg signed [31:0] error;
    reg signed [31:0] prev_error;
    (* keep = "true" *) reg signed [31:0] integral;
    (* keep = "true" *) reg signed [31:0] derivative;
    (* keep = "true" *) reg signed [31:0] pid_out;
    
    // Max integral for anti-windup
    localparam MAX_INTEGRAL = 1000;
    localparam MIN_INTEGRAL = -1000;

    initial begin
        pwm_out = 0;
        prev_error = 0;
        integral = 0;
        pid_out = 0;
    end

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            prev_error <= 0;
            integral <= 0;
            pid_out <= 0;
            pwm_out <= 0;
        end
        else if (sample_enable) begin
            // 1. Calculate Error
            error = target_val - current_val;

            // 2. Integral Term with Anti-Windup
            integral = integral + error;
            if (integral > MAX_INTEGRAL) integral = MAX_INTEGRAL;
            else if (integral < MIN_INTEGRAL) integral = MIN_INTEGRAL;

            // 3. Derivative Term
            derivative = error - prev_error;
            prev_error <= error;

            // 4. Compute PID Output
            pid_out = (Kp * error) + (Ki * integral) + (Kd * derivative);

            // 5. Output Clamping (Synchronous)
            if (pid_out < 0) begin
                 pwm_out <= 0;
            end
            else if (pid_out > 255) begin
                 pwm_out <= 255;
            end
            else begin
                 pwm_out <= pid_out[7:0];
            end
        end
    end

endmodule