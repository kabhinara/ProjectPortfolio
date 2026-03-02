module uart_rx(
    input clk_3125,
    input rst_n,
    input rx,
    output reg [7:0] rx_msg,
    output reg rx_parity, 
    output reg rx_complete
    );

/* UART receiver - NO PARITY (10-bit frame: START + 8 DATA + STOP) */

reg [7:0] byte_buffer;
reg [4:0] clock_count;
reg [2:0] byte_count;
reg [1:0] state;

localparam S_IDLE        = 2'b00;
localparam S_START_CHECK = 2'b01;
localparam S_DATA        = 2'b10;
localparam S_STOP        = 2'b11;

always @(posedge clk_3125 or negedge rst_n) begin
	if (!rst_n) begin
		state <= S_IDLE;
		clock_count <= 0;
		byte_count <= 0;
		byte_buffer <= 0;
		rx_msg <= 0;
		rx_parity <= 0;
		rx_complete <= 0;
	end
	else begin
		case (state)
			S_IDLE: begin
				rx_complete <= 0;
				if (!rx) begin
					clock_count <= 0;
					state <= S_START_CHECK;
				end
			end

			S_START_CHECK: begin
				if (clock_count == 5'd13) begin
					if (!rx) begin
						clock_count <= 0;
						byte_count <= 0;
						byte_buffer <= 0;
						state <= S_DATA;
					end else begin
						state <= S_IDLE;
					end
				end else begin
					clock_count <= clock_count + 1'b1;
				end
			end

			S_DATA: begin
				if (clock_count == 5'd26) begin
					byte_buffer <= {rx, byte_buffer[7:1]};
					clock_count <= 0;
					if (byte_count == 3'd7) begin
						byte_count <= 0;
						state <= S_STOP;
					end else begin
						byte_count <= byte_count + 1'b1;
					end
				end else begin
					clock_count <= clock_count + 1'b1;
				end
			end
			
			S_STOP: begin
				if (clock_count == 5'd26) begin
					rx_msg <= byte_buffer;
					rx_complete <= 1;
					state <= S_IDLE;
					clock_count <= 0;
				end else begin
					rx_complete <= 0;
					clock_count <= clock_count + 1'b1;
				end
			end
			
			default: state <= S_IDLE;
		endcase
	end
end		
endmodule
