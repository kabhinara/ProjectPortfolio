module wall_centering_controller (
    input clk,
    input [15:0] dist_left,
    input [15:0] dist_right,
    input valid_left,  
    input valid_right, 
    input enable,      
    output reg [7:0] pwm_left,
    output reg [7:0] pwm_right
);

    localparam TARGET_DIST_MM  = 80; 
    localparam BASE_SPEED      = 255; 
    localparam KP              = 10;   
    localparam KD              = 3;   
    
    reg signed [31:0] error;
    reg signed [31:0] last_error;
    reg signed [31:0] d_term;
    reg signed [31:0] adjustment;
    reg signed [31:0] pwm_l_calc, pwm_r_calc;

    function [15:0] abs_diff;
        input [15:0] a, b;
        begin abs_diff = (a > b) ? (a - b) : (b - a); end
    endfunction

    localparam MAX_WALL_DIST = 250; 

    always @(posedge clk) begin
        if (enable) begin
            if (valid_left && (dist_left < MAX_WALL_DIST) && valid_right && (dist_right < MAX_WALL_DIST)) begin
                if (abs_diff(dist_left, TARGET_DIST_MM) < abs_diff(dist_right, TARGET_DIST_MM)) begin
                    error <= $signed(dist_left) - TARGET_DIST_MM;
                end else begin
                    error <= TARGET_DIST_MM - $signed(dist_right);
                end
            end 
            else if (valid_left && (dist_left < MAX_WALL_DIST)) begin
                error <= $signed(dist_left) - TARGET_DIST_MM;
            end 
            else if (valid_right && (dist_right < MAX_WALL_DIST)) begin
                error <= TARGET_DIST_MM - $signed(dist_right);
            end 
            else begin error <= 0; end

            d_term <= error - last_error;
            adjustment <= (error * KP) + (d_term * KD);
            last_error <= error;

            pwm_l_calc <= BASE_SPEED - adjustment;
            pwm_r_calc <= BASE_SPEED + adjustment;

            if (pwm_l_calc > 255) pwm_left <= 255;
            else if (pwm_l_calc < 0) pwm_left <= 0;
            else pwm_left <= pwm_l_calc[7:0];

            if (pwm_r_calc > 255) pwm_right <= 255;
            else if (pwm_r_calc < 0) pwm_right <= 0;
            else pwm_right <= pwm_r_calc[7:0];
        end
        else begin
            error <= 0; last_error <= 0; d_term <= 0; adjustment <= 0;
            pwm_left <= BASE_SPEED[7:0]; pwm_right <= BASE_SPEED[7:0];
        end
    end
endmodule
