module servo_control (
    input clk_3125,
    input rst_n, // Added
    input [3:0] angle_index,
    output reg servo_pwm
);

    reg [15:0] counter = 0;
    reg [15:0] pulse_count;

    always @(*) begin
        case (angle_index)
            4'd0: pulse_count = 16'd3824; // -33.75 deg (Anticlockwise)
            4'd1: pulse_count = 16'd5120; // 0 deg (Default Position)
            4'd2: pulse_count = 16'd5552; // +11.25 deg (Clockwise step)
            4'd3: pulse_count = 16'd5984; // +22.5 deg
            4'd4: pulse_count = 16'd6416; // +33.75 deg
            4'd5: pulse_count = 16'd6848; // +45 deg
            4'd8: pulse_count = 16'd8125; // 90 deg (Legacy)
            default: pulse_count = 16'd5120;
        endcase
    end

    always @(posedge clk_3125) begin
        if (!rst_n) begin
            counter <= 0;
            servo_pwm <= 0;
        end else begin
            if (counter >= 16'd62499) begin
                counter <= 0;
            end else begin
                counter <= counter + 1;
            end

            if (counter < pulse_count) begin
                servo_pwm <= 1'b1;
            end else begin
                servo_pwm <= 1'b0;
            end
        end
    end

endmodule