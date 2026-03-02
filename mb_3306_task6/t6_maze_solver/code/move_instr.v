module move_instr (
    input clk,
    input rst_n,
    input move_done,
    input proceed,          // NEW: Handshake input
    input [3:0] flag_count,
    input [3:0] flag_idx,
    input ir_right, ir_left,
    input left, mid, right, // 0 - no wall, 1 - wall
    output reg [7:0] pos_out,
    output reg [2:0] move
);

    // State Encoding for 'mark'
    localparam UNEXPLORED = 2'b00;
    localparam VISITED_0  = 2'b01; 
    localparam VISITED_1  = 2'b10; 
    localparam WALL       = 2'b11; 

    reg state;
    reg [1:0] dir;
    reg [7:0] pos;
    
    // Flag logic
    reg short_route;

    // 2-bit array to store 4 states
    reg [1:0] mark [0:99]; 
    
    integer i;
    wire [7:0] rpos, lpos, mpos, bpos;
    wire [3:0] rdist, ldist, mdist;
    wire [6:0] index, lindex, bindex, mindex, rindex;
    reg [1:0] flag;
    reg first_move;
    reg move_done_prev;

    always @(*) begin
        pos_out = pos;
    end

    initial begin
        flag[0] = 1;
        state <= 1'bx;
        i = 0;
        pos <= 8'b01001001;
        dir <= 2'b10;
        first_move = 1;
        move_done_prev = 0;
        short_route = 0;
        for (i = 9; i < 99; i = i + 1) begin
            mark[i] = UNEXPLORED;
        end
    end

    // Position and Index Logic
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
            first_move <= 1;
            move_done_prev <= 0;
            short_route <= 0;
            for (i = 0; i < 9; i = i + 1) begin
		mark[i] <= WALL;
	    end
            for (i = 9; i < 99; i = i + 1) begin
                mark[i] <= UNEXPLORED;
            end
            state <= 0;
        end
        else begin
            move_done_prev <= move_done;
            // Sticky short_route logic
            if (flag_count != 0 && flag_idx >= flag_count) begin
		short_route <= 1;
		mark[4] <= 0;
	    end
            case (state)
                0: begin
                    if (first_move) begin
                        move <= 3'b001; // Force Forward
                        first_move <= 0;
                    end
                    // Logic using 2-bit states
                    else if ((left | (mark[lindex] == WALL)) & (right | (mark[rindex] == WALL)) & (mid | (mark[mindex] == WALL))) begin 
                        move <= 3'b100;
                        flag[0] <= 1;
                    end
                    
                    else if (pos == 8'b01000001 & !flag[0]) begin 
                        flag[1] <= 1;
                        move <= 3'b100;
                    end
                    
                    else if (~(mid ^ left ^ right)) move <= (!left) ? 3'b010 : ((!right) ? 3'b011 : 3'b001);
                    
                    else if (mid | left | right) begin
                        if (left) begin
                            if (flag[0]) begin
                                if (mark[mindex] == WALL) begin
                                    flag[0] <= 1;
                                    move <= 3'b011;
                                end
                                else if (mark[rindex] == WALL) begin
                                    flag[0] <= 1;
                                    move <= 3'b001;
                                end
                                else if (mark[mindex] == VISITED_1) begin
                                    flag[0] <= 0;
                                    move <= 3'b011;
                                end
                                else if (mark[rindex] == VISITED_1) begin
                                    flag[0] <= 0;
                                    move <= 3'b001;
                                end
                                else begin
                                    flag[0] <= 0;
                                    if (mark[rindex] == UNEXPLORED && mark[mindex] == UNEXPLORED) begin
                                        move <= (short_route ? (mdist <= rdist) : (mdist >= rdist)) ? 3'b001 : 3'b011;
                                    end
                                    else move <= (mark[rindex] == UNEXPLORED) ? 3'b011 : 3'b001;
                                end
                            end 
                            else begin
                                if (mark[rindex] == UNEXPLORED && mark[mindex] == UNEXPLORED) begin
                                    move <= (short_route ? (mdist <= rdist) : (mdist >= rdist)) ? 3'b001 : 3'b011;
                                end
                                else begin
                                    if (flag[1]) move <= (mark[rindex] == WALL) ? 3'b001 : 3'b011;
                                    else move <= 3'b100;
                                end
                            end
                        end
                        else if (right) begin
                            if (flag[0]) begin
                                if (mark[mindex] == WALL) begin
                                    flag[0] <= 1;
                                    move <= 3'b010;
                                end
                                else if (mark[lindex] == WALL) begin
                                    flag[0] <= 1;
                                    move <= 3'b001;
                                end
                                else if (mark[mindex] == VISITED_1) begin
                                    flag[0] <= 0;
                                    move <= 3'b010;
                                end
                                else if (mark[lindex] == VISITED_1) begin
                                    flag[0] <= 0;
                                    move <= 3'b001;
                                end
                                else begin
                                    flag[0] <= 0;
                                    if (mark[lindex] == UNEXPLORED && mark[mindex] == UNEXPLORED) begin
                                        move <= (short_route ? (mdist <= ldist) : (mdist > ldist)) ? 3'b001 : 3'b010;
                                    end
                                    else move <= (mark[lindex] == UNEXPLORED) ? 3'b010 : 3'b001;
                                end
                            end 
                            else begin
                                if (mark[lindex] == UNEXPLORED && mark[mindex] == UNEXPLORED) begin
                                    move <= (short_route ? (mdist <= ldist) : (mdist > ldist)) ? 3'b001 : 3'b010;
                                end
                                else begin
                                    if (flag[1]) move <= (mark[lindex] == WALL) ? 3'b001 : 3'b010;
                                    else move <= 3'b100;
                                end
                            end
                        end
                        else begin // Mid is walled
                            if (flag[0]) begin
                                if (mark[rindex] == WALL) begin
                                    flag[0] <= 1;
                                    move <= 3'b010;
                                end
                                else if (mark[lindex] == WALL) begin
                                    flag[0] <= 1;
                                    move <= 3'b011;
                                end
                                else if (mark[rindex] == VISITED_1) begin
                                    flag[0] <= 0;
                                    move <= 3'b010;
                                end
                                else if (mark[lindex] == VISITED_1) begin
                                    flag[0] <= 0;
                                    move <= 3'b011;
                                end
                                else begin
                                    flag[0] <= 0;
                                    if (mark[rindex] == UNEXPLORED && mark[lindex] == UNEXPLORED) begin
                                        move <= (short_route ? (rdist < ldist) : (rdist > ldist)) ? 3'b011 : 3'b010;
                                    end
                                    else move <= (mark[lindex] == UNEXPLORED) ? 3'b010 : 3'b011;
                                end
                            end 
                            else begin
                                if (mark[rindex] == UNEXPLORED && mark[lindex] == UNEXPLORED) begin
                                    move <= (short_route ? (rdist < ldist) : (rdist > ldist)) ? 3'b011 : 3'b010;
                                end
                                else begin
                                    if (flag[1]) move <= (mark[lindex] == WALL) ? 3'b011 : 3'b010;
                                    else move <= 3'b100;
                                end
                            end
                        end
                    end
                    state <= 1;
                end
                            
                1: begin 
                    if (move_done && !move_done_prev) begin 
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
                        
                        if (flag[0]) mark[index] <= WALL;
                        else if (flag[1]) mark[index] <= VISITED_1;
                        else mark[index] <= VISITED_0;
                        
                        state <= 0;
                        move <= 3'b000;
                    end
                end
                default: state <= 1;
            endcase

            // Force flag to 1 once mission objective (flag_count) is reached
            if (flag_count != 0 && flag_idx >= flag_count) flag <= 1;
        end
    end
endmodule
