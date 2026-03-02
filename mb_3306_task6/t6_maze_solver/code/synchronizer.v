module synchronizer (
    input clk,
    input in,
    output reg out
);
    reg sync_0;
    always @(posedge clk) begin
        sync_0 <= in;
        out <= sync_0;
    end
endmodule
