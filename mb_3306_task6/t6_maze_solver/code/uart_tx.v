module uart_tx(
    input clk_3125,
    input rst_n, 
    input parity_type, tx_start,
    input [7:0] data,
    output reg tx, tx_done
);

/* UART TX - NO PARITY (10-bit frame: START + 8 DATA + STOP) */

    reg [4:0] clock_count;
    reg [2:0] bit_index;
    reg [1:0] state;
    reg active;

    always @(posedge clk_3125 or negedge rst_n) begin
        if (!rst_n) begin
            state <= 2'b00;
            clock_count <= 0;
            bit_index <= 0;
            active <= 0;
            tx <= 1'b1;
            tx_done <= 1'b0;
        end else if (active) begin
            if (clock_count == 26) begin
                clock_count <= 0;
                case (state)
                    2'b00: begin 
                        state <= 2'b01;
                        bit_index <= 0;
                        tx <= data[0];
                    end
                    2'b01: begin 
                        if (bit_index == 7) begin
                            state <= 2'b10;
                            tx <= 1'b1; // Stop bit
                        end else begin
                            bit_index <= bit_index + 1;
                            tx <= data[bit_index + 1];
                        end
                    end
                    2'b10: begin 
                        tx_done <= 1;
                        active <= 0;
                        state <= 0;
                    end
                    default: state <= 0;
                endcase
            end else begin
                clock_count <= clock_count + 1;
            end
        end else begin 
            tx <= 1;
            tx_done <= 0;
            clock_count <= 0;
            state <= 0;
            if (tx_start) begin
                active <= 1;
                tx <= 0; 
                clock_count <= 0;
                state <= 2'b00;
            end
        end
    end
endmodule
