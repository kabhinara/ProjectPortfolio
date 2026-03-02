module pid (
	input clk_50M,
	input len1, len2, ren1, ren2, echo_mid, echo_left, echo_right,
	output lin1, lin2, rin1, rin2, enar, enal, trig_mid, trig_left, trig_right,
    output [3:0] leds
);

wire [3:0] speedstep;
wire [2:0] move_sig;
wire move_done_sig;

motor_driver l298n (
    .clk_50M(clk_50M),
    .len1(len1), .len2(len2), .ren1(ren1), .ren2(ren2),
    .echo_mid(echo_mid), .echo_left(echo_left), .echo_right(echo_right),
    .lin1(lin1), .lin2(lin2), .rin1(rin1), .rin2(rin2),
    .enal(enal), .enar(enar),
    .trig_mid(trig_mid), .trig_left(trig_left), .trig_right(trig_right),
    .current_move(move_sig),
    .current_move_done(move_done_sig)
);

assign leds[0] = move_done_sig;
assign leds[3:1] = move_sig;

endmodule
