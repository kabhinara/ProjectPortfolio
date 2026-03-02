
module pwm_generator(
    input clk_3125KHz,
    input [3:0] duty_cycle,
    output reg clk_195KHz, pwm_signal
);

initial begin
    clk_195KHz = 0; pwm_signal = 1;
end
//////////////////DO NOT MAKE ANY CHANGES ABOVE THIS LINE //////////////////

reg [3:0] counter1 = 0;
reg [2:0] counter2 = 0;
always @ (posedge clk_3125KHz) begin
	if(!counter2)
		begin
			clk_195KHz = ~clk_195KHz;
		end
		
	if(counter1 < duty_cycle)
			pwm_signal = 1;
	else
			pwm_signal = 0;
	counter1 = counter1 + 1'b1;
	counter2 = counter2 + 1'b1;
end
//////////////////DO NOT MAKE ANY CHANGES BELOW THIS LINE //////////////////

endmodule