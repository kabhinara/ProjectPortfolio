// Task 2C - MazeSolver Bot

module pledge (
    input clk,
    input rst_n,
	 input move_done,
    input left, mid, right, // 0 - no wall, 1 - wall
    output reg [2:0] move
);

/*

| cmd | move  | meaning   |
|-----|-------|-----------|
| 000 | 0     | STOP      |
| 001 | 1     | FORWARD   |
| 010 | 2     | LEFT      |
| 011 | 3     | RIGHT     | 
| 100 | 4     | U_TURN    |

START POS   : 4,8
EXIT POS    : 4,0
DEADENDS    : 9

*/
//////////////////DO NOT MAKE ANY CHANGES ABOVE THIS LINE //////////////////

reg backtrack;
reg initialized;

initial begin 
	initialized = 0;
	backtrack = 0;
	move = 3'b000;
end

always @(posedge clk or negedge rst_n) begin
	if (!rst_n) begin
		  move <= 3'b000;
		  backtrack <= 0;
		  initialized <= 0;
	end else if (move_done) begin
		  // Update backtrack on U-turn
		  if (move == 3'b100) begin
				backtrack <= ~backtrack;
		  end
		  
		  if (!initialized) begin
				// At startup: we're at [4,8] facing South according to testbench
				// We need to face North to enter the maze properly
				// Since we can't move South (out of bounds), we must turn
				move <= 3'b100; // U-TURN to face North
				initialized <= 1;
		  end else begin
				// Normal maze solving
				if (!backtrack) begin
					 // Left-hand rule
					 if (!left)       move <= 3'b010;
					 else if (!mid)   move <= 3'b001;
					 else if (!right) move <= 3'b011;
					 else             move <= 3'b100;
				end else begin
					 // Right-hand rule  
					 if (!right)      move <= 3'b011;
					 else if (!mid)   move <= 3'b001;
					 else if (!left)  move <= 3'b010;
					 else             move <= 3'b100;
				end
		  end
	 end
end

//////////////////DO NOT MAKE ANY CHANGES BELOW THIS LINE //////////////////

endmodule
