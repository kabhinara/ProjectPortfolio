module ultrasonic #(
    parameter THRESHOLD_MM = 90,
    parameter THRESHOLD_HIGH = 120,
    parameter TRIGGER_OFF = 0
)(
    input clk_50M, reset, echo_rx,
    output reg trig,
    output op,
    output wire [15:0] distance_out
);

    wire echo_rx_safe;
    synchronizer sync_echo (.clk(clk_50M), .in(echo_rx), .out(echo_rx_safe));

initial begin
    trig = 0;
end

reg [19:0] counter = 0;        // Reduced from 32 to 20 bits (enough for 12ms)
reg [21:0] echo_counter = 0;   // Increased to 22 bits to prevent overflow (max ~14m range)
reg object_present = 0;
reg echo_prev = 0;
reg measuring = 0;
reg [15:0] stored_distance = 0;
reg started = 0;
reg [2:0] pulse_index = 0;     // Reduced from 32 to 3 bits (8 shifts max)
reg fresh_update = 0;          // New flag to track valid readings

// Distance calculation
wire [15:0] calculated_distance = (echo_counter * 34) / 10000;
localparam SHIFT_STEP = 554;

always @(posedge clk_50M or negedge reset) begin
    if (!reset) begin
        counter <= 0;
        echo_counter <= 0;
        object_present <= 0;
        echo_prev <= 0;
        measuring <= 0;
        trig <= 0;
        stored_distance <= 0;
        started <= 0;
        pulse_index <= 0;
        fresh_update <= 0;
    end else begin
        echo_prev <= echo_rx_safe;
        counter <= counter + 1;

        if (!started && counter >= 50) begin
            started <= 1;
            counter <= 0;
        end

        if (started) begin
            // Trigger pulse and Timeout Logic
            if ((counter >= (TRIGGER_OFF + pulse_index * SHIFT_STEP)) && (counter < (TRIGGER_OFF + pulse_index * SHIFT_STEP + 500))) begin
                trig <= 1;
                // At the start of the trigger pulse, check if we got an update last cycle
                if (counter == (TRIGGER_OFF + pulse_index * SHIFT_STEP)) begin
                    if (!fresh_update) 
                        object_present <= 0; // No echo received in previous cycle -> Clear OP
                    fresh_update <= 0; // Reset for this cycle
                end
            end else
                trig <= 0;

            // Rising edge → start measurement
            if (!echo_prev && echo_rx_safe && !measuring) begin
                measuring <= 1;
                echo_counter <= 0;
                // object_present <= 0; // Removed: Hold previous value during measurement
            end

            // Count echo width
            if (measuring && echo_rx_safe)
                echo_counter <= echo_counter + 1;

            // Falling edge → calculate distance
            if (echo_prev && !echo_rx_safe && measuring) begin
                measuring <= 0;
                stored_distance <= calculated_distance;

                // 70 mm threshold for obstacle detection with hysteresis
                if (calculated_distance < THRESHOLD_MM)
                    object_present <= 1;
                else if (calculated_distance > THRESHOLD_HIGH)
                    object_present <= 0;
                
                fresh_update <= 1; // Mark that we received a valid echo this cycle
            end

            // Increment pulse index every cycle
            if (counter >= 599999) begin
                counter <= 0;
                if (pulse_index < 7)  // Prevent overflow
                    pulse_index <= pulse_index + 1;
            end
        end
    end
end

assign op = object_present;
assign distance_out = stored_distance;

endmodule