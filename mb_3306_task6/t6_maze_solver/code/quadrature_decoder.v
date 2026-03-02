module quadrature_decoder (
    input wire clk,
    input wire refresh,
    input wire enc_A,
    input wire enc_B,
    (* preserve *) output reg [31:0] pulse_count
);

    reg [2:0] quadA_delayed, quadB_delayed;
    
    // Combinational logic for enable and direction
    // Based on the difference between the 2nd and 3rd stages of the shift register
    wire count_enable = quadA_delayed[1] ^ quadA_delayed[2] ^ quadB_delayed[1] ^ quadB_delayed[2];
    wire count_direction = quadA_delayed[1] ^ quadB_delayed[2];

    always @(posedge clk) begin
        if (refresh) begin
            pulse_count <= 0;
            quadA_delayed <= 0;
            quadB_delayed <= 0;
        end else begin
            // Shift in new values
            quadA_delayed <= {quadA_delayed[1:0], enc_A};
            quadB_delayed <= {quadB_delayed[1:0], enc_B};
            
            if(count_enable) begin
                if(count_direction) 
                    pulse_count <= pulse_count + 1;
                else 
                    pulse_count <= pulse_count - 1;
            end
        end
    end

endmodule