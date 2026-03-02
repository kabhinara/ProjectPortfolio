
module frequency_scaling (
    input clk_16x,
	 input rst_n,
    output reg clk_1x = 0
);

// initial begin
//     clk_1x = 0;
// end
//////////////////DO NOT MAKE ANY CHANGES ABOVE THIS LINE //////////////////

reg [2:0] counter = 0;

always @ (posedge clk_16x or negedge rst_n) begin
	if (!rst_n) begin
		clk_1x <= 0;
		counter <= 0;
	end
	else begin
		if (!counter) begin
			clk_1x <= ~clk_1x;
		end
		counter <= counter + 1'b1;
	end
end
	
//////////////////DO NOT MAKE ANY CHANGES BELOW THIS LINE //////////////////

endmodule