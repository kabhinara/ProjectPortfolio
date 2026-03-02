
module pwm_generator(
    input clk_524288x,
    input [7:0] duty_cycle,
    output reg clk_1x = 0, 
    output reg pwm_signal = 1
);

// initial begin
//     clk_1x = 0; pwm_signal = 1;
// end
//////////////////DO NOT MAKE ANY CHANGES ABOVE THIS LINE //////////////////

reg [7:0] counter1 = 0;
reg [17:0] counter2 = 0;
always @ (posedge clk_524288x) begin
	if(!counter2)
		begin
			clk_1x = ~clk_1x;
		end
		
	if(counter1 <= duty_cycle)
			pwm_signal = 1;
	else
			pwm_signal = 0;
	counter1 = counter1 + 1'b1;
	counter2 = counter2 + 1'b1;
end
//////////////////DO NOT MAKE ANY CHANGES BELOW THIS LINE //////////////////

endmodule
