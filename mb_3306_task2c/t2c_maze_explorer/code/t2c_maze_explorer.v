module t2c_maze_explorer (
    input clk,
    input rst_n,
    input left, mid, right, // 0 - no wall, 1 - wall
    output reg [2:0] move
);

/*

 |  cmd  |  move   |  meaning    | 
 | ----- | ------- | ----------- | 
 |  000  |  0      |  STOP       | 
 |  001  |  1      |  FORWARD    | 
 |  010  |  2      |  LEFT       | 
 |  011  |  3      |  RIGHT      |  
 |  100  |  4      |  U_TURN     | 

START POS   : 4,8
EXIT POS    : 4,0
DEADENDS    : 9

*/
//////////////////DO NOT MAKE ANY CHANGES ABOVE THIS LINE //////////////////

/*
Add your logic here
*/

/*
# Team ID:          eYRC#3306
# Theme:            MazeSolver Bot
# Author List:      K.Abhinav Narasimhan, J Sajan, Surendar Viswa S, S V Nimal Ronish
# Filename:         t2c_maze_explorer.v
# File Description: Implementation of Maze Explorer algorithm using a custom-made Modified DFS/Tremaux's Algorithm
# Global variables: None
*/

// The 4-way junction case is WIP, awaiting changes to adhere to the optimized logic, otherwise code is complete

reg state;
reg [1:0] dir;
reg [7:0] pos;
reg mark[98:9];
integer i;
wire [7:0] rpos,lpos,mpos,bpos;
wire [3:0] rdist,ldist,mdist;
wire [6:0] index,lindex,bindex,mindex,rindex;
reg [1:0] flag;

initial begin
	flag[0] = 1;
	state <= 1'bx;
	i = 0;
	pos <= 8'b01001001;
	dir <= 2'b10;
end

assign bpos = (dir == 0) ? {pos[7:4], pos[3:0] - 4'b1} : (dir==1) ? {pos[7:4] - 4'b1, pos[3:0]} : (dir==2) ? {pos[7:4], pos[3:0] + 4'b1} : {pos[7:4] + 4'b1, pos[3:0]}; 
assign mpos = (dir == 0) ? {pos[7:4], pos[3:0] + 4'b1} : (dir==1) ? {pos[7:4] + 4'b1, pos[3:0]} : (dir==2) ? {pos[7:4], pos[3:0] - 4'b1} : {pos[7:4] - 4'b1, pos[3:0]}; 
assign rpos = (dir == 0) ? {pos[7:4] - 4'b1, pos[3:0]} : (dir==1) ? {pos[7:4], pos[3:0] + 4'b1} : (dir==2) ? {pos[7:4] + 4'b1, pos[3:0]} : {pos[7:4], pos[3:0] - 4'b1};
assign lpos = (dir == 0) ? {pos[7:4] + 4'b1, pos[3:0]} : (dir==1) ? {pos[7:4], pos[3:0] - 4'b1} : (dir==2) ? {pos[7:4] - 4'b1, pos[3:0]} : {pos[7:4], pos[3:0] + 4'b1}; 

assign ldist = (lpos[7:4] > 4'b0100 ? lpos[7:4] - 4'b0100 : 4'b0100 - lpos[7:4]) + lpos[3:0];
assign rdist = (rpos[7:4] > 4'b0100 ? rpos[7:4] - 4'b0100 : 4'b0100 - rpos[7:4]) + rpos[3:0];
assign mdist = (mpos[7:4] > 4'b0100 ? mpos[7:4] - 4'b0100 : 4'b0100 - mpos[7:4]) + mpos[3:0];

assign index = pos[7:4] + pos[3:0]*9;
assign bindex = (dir==0) ? index-7'b0001001 : (dir==1) ? index-7'b0000001 : (dir==2) ? index+7'b0001001 : index+7'b0000001;
assign mindex = 2*index - bindex;
assign lindex = (dir==0) ? index+7'b0000001 : (dir==1) ? index-7'b0001001 : (dir==2) ? index-7'b0000001 : index+7'b0001001;
assign rindex = 2*index - lindex;

always @(negedge clk or negedge rst_n) begin
	if (!rst_n) begin
		flag[0] <= 1;
		pos <= 8'b01001001;
      dir <= 2;
      move <= 3'b000;
		i = 0;
		for (i = 9; i < 90; i = i + 1) begin
			mark[i] <= 1'bx;
		end
		for (i = 90; i < 99; i = i + 1) begin
			mark[i] <= 1'bz;
		end
		state <= 1'bx;
	end
	else begin
		i = 0;
		case (state)
		
			0: begin
				
				if ((left | (mark[lindex] === 1'bz)) & (right | (mark[rindex] === 1'bz)) & (mid | (mark[mindex] === 1'bz))) begin	//	1 - walls or fully marked regardless of how many walls - subtree done
					move <= 3'b100;
					flag[0] <= 1;
				end
				
				else if (pos == 8'b01000001 & !flag[0]) begin 
					flag[1] <= 1;
					move <= 3'b100;	// 2 - Subtree we came from is not done at the end OR there are subtrees branching from end that are not done - flag[1]track
				end
				
				else if (~(mid ^ left ^ right)) move <= (!left) ? 3'b010 : ((!right) ? 3'b011 : 3'b001); //	3 - Only one direction left (1 => cant be fully marked)
				
				else if (mid | left | right) begin	//	Two directions left (1 => cant be fully marked)
					if (left) begin
						if (flag[0]) begin // 4 - One in the three road junction has been explored
							if (mark[mindex] === 1'bz) begin // 5 - Forward is also marked - so right is only unexplored route, and thus till now fully explored as mark must propagate from the start
								flag[0] <= 1;
								move <= 3'b011;
							end
							else if (mark[rindex] === 1'bz) begin	// similar to above if statement
								flag[0] <= 1;
								move <= 3'b001;
							end
							else if (mark[mindex] === 1'b1) begin
								flag[0] <= 0;
								move <= 3'b011;
							end
							else if (mark[rindex] === 1'b1) begin
								flag[0] <= 0;
								move <= 3'b001;
							end
							// What if flag[1] is explored, but both forward and right are unexplored? Would it make any difference if flag[1] is unexplored? YES, because then the else condition of 7 wont trigger
							else begin	// 6 - Both forward and right are not done - choose the branch likely to not reach the end to avoid prematurely reaching the end - the one that is immediately farther from end
								flag[0] <= 0; // prefer 0 and 1 over 3 in next line
								//	if both are equally explored, tiebreak ; else less explored
								if (mark[rindex] === 1'bx && mark[mindex] === 1'bx) begin
									move <= (mdist > rdist) ? 3'b001 : 3'b011;
								end
								else move <= mark[rindex] === 1'b0 ? 3'b011 : 3'b001;
							end
						end	
						else begin	// 7 - None of the directions are done - but back tracking immediately is a poor idea since it could reach the end and come flag[1] and again it would flag[1]track and loop on and on - choose the branch likely to not reach the end to avoid prematurely reaching the end - the one that is immediately farther from end - l>r>m if equal distance
							if (mark[rindex] === 1'bx && mark[mindex] === 1'bx) begin
								move <= (mdist > rdist) ? 3'b001 : 3'b011;
							end
							else begin
								if (flag[1]) move <= mark[rindex] === 1'bz ? 3'b001 : 3'b011;
								else move <= 3'b100;
							end
						end
					end
					else if (right) begin	// Same logic as before, for when right is walled
						if (flag[0]) begin
							if (mark[mindex] === 1'bz) begin
								flag[0] <= 1;
								move <= 3'b010;
							end
							else if (mark[lindex] === 1'bz) begin
								flag[0] <= 1;
								move <= 3'b001;
							end
							else if (mark[mindex] === 1'b1) begin
								flag[0] <= 0;
								move <= 3'b010;
							end
							else if (mark[lindex] === 1'b1) begin
								flag[0] <= 0;
								move <= 3'b001;
							end
							else begin
								flag[0] <= 0;
								if (mark[lindex] === 1'bx && mark[mindex] === 1'bx) begin
									move <= (mdist > ldist) ? 3'b001 : 3'b010;
								end
								else move <= mark[lindex] === 1'bx ? 3'b010 : 3'b001;
							end
						end	
						else begin
							if (mark[lindex] === 1'bx && mark[mindex] === 1'bx) begin
								move <= (mdist > ldist) ? 3'b001 : 3'b010;
							end
							else begin
								if (flag[1]) move <= mark[lindex] === 1'bz ? 3'b001 : 3'b010;
								else move <= 3'b100;
							end
						end
					end
					else begin	// Same logic as before, for when mid is walled
						if (flag[0]) begin
							if (mark[rindex] === 1'bz) begin
								flag[0] <= 1;
								move <= 3'b010;
							end
							else if (mark[lindex] === 1'bz) begin
								flag[0] <= 1;
								move <= 3'b011;
							end
							else if (mark[rindex] === 1'b1) begin
								flag[0] <= 0;
								move <= 3'b010;
							end
							else if (mark[lindex] === 1'b1) begin
								flag[0] <= 0;
								move <= 3'b011;
							end
							else begin
								flag[0] <= 0;
								if (mark[rindex] === 1'bx && mark[lindex] === 1'bx) begin
									move <= (rdist > ldist) ? 3'b011 : 3'b010;
								end
								else move <= mark[lindex] === 1'bx ? 3'b010 : 3'b011;
							end
						end	
						else begin
							if (mark[rindex] === 1'bx && mark[lindex] === 1'bx) begin
								move <= (rdist > ldist) ? 3'b011 : 3'b010;
							end
							else begin
								if (flag[1]) move <= mark[lindex] === 1'bz ? 3'b011 : 3'b010;
								else move <= 3'b100;
							end
						end
					end
				end
				
//				Not refined yet. WIP.
//				else begin	// All 3 sides are open
//					if ((mark[lindex] == mark[rindex]) & (mark[rindex] == mark[mindex])) begin // 8 - All 3 sides are equally explored -  1 => all 3 cannot be fully explored - so either all 0 or all 1
//						flag[0] <= 0;
//						move <= (rdist > ldist) ? ((mdist > rdist) ? 3'b001 : 3'b011) : 3'b010;
////						checks[6] <= !checks[6];
//					end
//					else if ((mark[rindex] != 2) & (mark[lindex] != 2) & (mark[mindex] != 2)) begin // So none are fully explored => two unexplored or one unexplored
//						flag[0] <= 0;
////						checks[7] <= !checks[7];
//						if (!mark[lindex]) move <= (!mark[rindex] & (rdist > ldist)) ? 3'b011 : ((!mark[mindex] & (mdist > ldist)) ? 3'b001 : 3'b010);	//	9 - One is explored, and if the other unexplored and distance is greater for it then take it, if not just take the first one anyways - though both other directions should be checked for zero
//						else if (!mark[rindex]) move <= (!mark[mindex] & (mdist > rdist)) ? 3'b001 : 3'b011; // By now we know that one side is marked 1, so we just check for other two if they are marked simultaneously or not		if (!mark[lindex]) move <= 3'b010;
//						else move <= 3'b001; // So only way we reach this statement is if both other directions are marked 1 - in which case this is unexplored, as 8 => 111 won't reach the parent block
//					end
//					// By now we have eliminated 000, 001, 011 and 111; but what about 002, 012, 112, 022 and 122? 
//					else if (mark[lindex] + mark[rindex] + mark[mindex] == 2) begin	//	10 - One of the directions must be fully explored, with other 2 completely unexplored (002)
//						flag[0] <= 0;
////						checks[8] <= !checks[8];
//						move <= !mark[lindex] ? ((!mark[rindex] & (rdist > ldist)) ? 3'b011 : ((!mark[mindex] & (mdist > ldist)) ?  3'b001 : 3'b010)) : ((mdist > rdist) ? 3'b001 : 3'b011);
//					end
//					else if (mark[lindex] + mark[rindex] + mark[mindex] == 3) begin	// 11 - Each direction explored differently (012)
//						flag[0] <= 0;
//						move <= !mark[lindex] ? 3'b010 : (!mark[rindex] ? 	3'b011 : 3'b001);
////						checks[9] <= !checks[9];
//					end
//					else if ((mark[lindex] != 2) + (mark[rindex] != 2) + (mark[mindex] != 2) == 2) begin	// 12 - Two directions are partially explored (112)
//						flag[0] <= 0;
////						checks[10] <= !checks[10];
//						move <= (mark[lindex] != 2) ? ((mark[rindex] != 2) ? ((rdist > ldist) ? 3'b011 : 3'b010) : ((mdist > ldist) ? 3'b001 : 3'b010)) : ((mdist > rdist) ? 3'b001 : 3'b011);
//					end
//					else begin
//						move <= (mark[lindex] != 2) ? 3'b010 : (mark[rindex] != 2 ? 3'b011 : 3'b001); // 13 - Two directions are fully explored - must go to other side; flag[0] remains as is (if flag[1] is 1 then keep 1, else keep 0)
////						checks[11] <= !checks[11];
//					end
//				end
				state <= 1;
			end
						
			1: begin
				if (move == 1) pos <= mpos;
				else if (move == 2) begin
					pos <= lpos;
					dir <= dir + 2'b01;
				end
				else if (move == 3) begin
					pos <= rpos;
					dir <= dir - 2'b01;
				end
				else if (move == 4) begin
					pos <= bpos;
					dir[1] <= !dir[1];
				end
				
				if (flag[0]) mark[index] <= 1'bz;
				else if (flag[1]) mark[index] <= 1'b1;
				else mark[index] <= 1'b0;
				state <= 0;
			end
			default: state <= 1;
		endcase
	end
end
			
endmodule
