module motor_driver (
    input clk_50M,
    input rst_n,
    input len1, len2, ren1, ren2, echo_mid, echo_left, echo_right,
    input ir_right, ir_left,
    input proceed,
    input [3:0] target_flags,
    output reg [7:0] pos_out,
    output reg lin1, lin2, rin1, rin2,
    output wire enal, enar, trig_mid, trig_left, trig_right,
    output wire [2:0] current_move,
    output wire stop_done, 
    output wire [3:0] flag_idx_out,
    output wire is_spinning,
    output wire is_measuring,
    output wire approach_trigger, 
    output wire measure_trigger,
    output wire end_trigger,
    output wire wall_left, wall_mid, wall_right,
    output wire [15:0] dist_left_out, dist_right_out, dist_mid_out
);

    // ==========================================
    // PARAMS & CONSTANTS
    // ==========================================
    localparam integer TARGET_SPEED_MAX = 60;   
    localparam integer TUNING_KP = 8;
    localparam integer TUNING_KI = 0;
    localparam integer TUNING_KD = 3;
    localparam integer TARGET_SPIN_SPEED = 40;
    localparam [7:0] SPIN_PWM = 8'd150; 
    
    // TIMING & COUNTS (TICKS/CYCLES)
    localparam [31:0] TURN_TICKS      = 3000;       
    localparam [31:0] UTURN_TICKS     = 4000;       
    localparam [31:0] MEASURE_TIME    = 150000000;  // 3.0s @50MHz
    localparam [31:0] UNDIP_TIME      = 20000000;   // 0.4s @50MHz
    localparam [31:0] BRAKE_TIME      = 15000000;   // 0.3s pause
    localparam [31:0] ALIGN_TIME      = 2500000;    
    localparam [31:0] FWD_GATE_TIME   = 50000000;   
    localparam [31:0] FWD_MAX_TICKS   = 9200;       
    
    // STATES
    localparam STATE_IDLE   = 4'd0;
    localparam STATE_PREP   = 4'd1;
    localparam STATE_SPIN   = 4'd2;
    localparam STATE_BRAKE  = 4'd3;
    localparam STATE_DRIVE  = 4'd4;
    localparam STATE_ALIGN  = 4'd5;
    localparam STATE_FINISH = 4'd6;
    localparam STATE_APPROACH = 4'd7;
    localparam STATE_MEASURE  = 4'd8;
    localparam STATE_UNDIP    = 4'd9;
    localparam STATE_REVERSE  = 4'd10;
    localparam STATE_CHECK_FWD = 4'd11;
    localparam STATE_CHECK_REV = 4'd12;
    localparam STATE_FLAG_ALIGN = 4'd13;
    localparam STATE_HALT       = 4'd14;

    // ==========================================
    // I/O & SENSORS
    // ==========================================
    assign wall_left = left; assign wall_mid = mid; assign wall_right = right;
    assign dist_left_out = distance_left; assign dist_right_out = distance_right; assign dist_mid_out = distance_mid;

    wire ir_right_sync, ir_left_sync;
    synchronizer sync_ir_r ( .clk(clk_50M), .in(ir_right), .out(ir_right_sync) );
    synchronizer sync_ir_l ( .clk(clk_50M), .in(ir_left),  .out(ir_left_sync) );

    wire clk_3125K, clk_195K, clk_12K, clk_95;
    wire left, mid, right;
    wire [15:0] distance_mid, distance_left, distance_right;
    
    frequency_scaling f3125K (.clk_16x(clk_50M), .rst_n(rst_n), .clk_1x(clk_3125K));
    frequency_scaling f195K (.clk_16x(clk_3125K), .rst_n(rst_n), .clk_1x(clk_195K));
    frequency_scaling f12K (.clk_16x(clk_195K), .rst_n(rst_n), .clk_1x(clk_12K));

    wire [7:0] pwm_left, pwm_right;
    wire [7:0] drive_pwm_l, drive_pwm_r; 
    wire [7:0] pid_out_left, pid_out_right; 

    pwm_generator pg_enal (clk_12K, pwm_left, clk_95, enal);
    pwm_generator pg_enar (clk_12K, pwm_right, , enar);

    ultrasonic #(90, 120, 0) us_mid (clk_50M, 1'b1, echo_mid, trig_mid, mid, distance_mid); 
    ultrasonic #(120, 150, 200000) us_left (clk_50M, 1'b1, echo_left, trig_left, left, distance_left);
    ultrasonic #(120, 150, 400000) us_right (clk_50M, 1'b1, echo_right, trig_right, right, distance_right);

    // ENCODERS
    reg signed [31:0] count_left = 0, count_right = 0;
    reg signed [31:0] last_count_left = 0, last_count_right = 0;
    reg signed [31:0] measured_speed_left = 0, measured_speed_right = 0;
    reg [2:0] leftA_delayed, leftB_delayed, rightA_delayed, rightB_delayed;
    reg [18:0] sample_counter = 0;
    reg sample_enable = 0;

    wire left_enable = leftA_delayed[1] ^ leftA_delayed[2] ^ leftB_delayed[1] ^ leftB_delayed[2];
    wire left_direction = leftA_delayed[1] ^ leftB_delayed[2];
    wire right_enable = rightA_delayed[1] ^ rightA_delayed[2] ^ rightB_delayed[1] ^ rightB_delayed[2];
    wire right_direction = rightA_delayed[1] ^ rightB_delayed[2];

    always @(posedge clk_50M) begin
        if (!rst_n) begin
            count_left <= 0; count_right <= 0;
            leftA_delayed <= 0; leftB_delayed <= 0;
            rightA_delayed <= 0; rightB_delayed <= 0;
        end else begin
            leftA_delayed <= {leftA_delayed[1:0], len2};
            leftB_delayed <= {leftB_delayed[1:0], len1};
            rightA_delayed <= {rightA_delayed[1:0], ren2};
            rightB_delayed <= {rightB_delayed[1:0], ren1};
            if(left_enable) count_left <= (left_direction) ? count_left + 1 : count_left - 1;
            if(right_enable) count_right <= (right_direction) ? count_right + 1 : count_right - 1;
            if (sample_counter >= 500000) begin
                sample_counter <= 0; sample_enable <= 1;
                measured_speed_left <= count_left - last_count_left;
                measured_speed_right <= count_right - last_count_right;
                last_count_left <= count_left; last_count_right <= count_right;
            end else begin
                sample_counter <= sample_counter + 1; sample_enable <= 0;
            end
        end
    end

    // CONTROLLERS
    reg signed [31:0] target_speed_left, target_speed_right;
    reg [3:0] sys_state; reg [3:0] next_state; 

    function [31:0] abs;
        input signed [31:0] val;
        begin abs = (val < 0) ? -val : val; end
    endfunction
    
    function [31:0] abs_diff32;
        input [31:0] a, b;
        reg signed [31:0] diff;
        begin diff = a - b; abs_diff32 = (diff < 0) ? -diff : diff; end
    endfunction

    // Wall Centering (RAW PWM)
    wire wcc_enable = (sys_state == STATE_DRIVE || sys_state == STATE_APPROACH); 
    wall_centering_controller wcc (
        .clk(clk_50M), .dist_left(distance_left), .dist_right(distance_right),
        .valid_left(left), .valid_right(right), .enable(wcc_enable),
        .pwm_left(drive_pwm_l), .pwm_right(drive_pwm_r)
    );

    pid_controller #(.Kp(TUNING_KP), .Ki(TUNING_KI), .Kd(TUNING_KD)) pid_l (
        .clk(clk_50M), .rst_n(rst_n), .sample_enable(sample_enable), .clk_adj(clk_95),
        .target_val(target_speed_left), .current_val(abs(measured_speed_left)), .pwm_out(pid_out_left)
    );
    pid_controller #(.Kp(TUNING_KP), .Ki(TUNING_KI), .Kd(TUNING_KD)) pid_r (
        .clk(clk_50M), .rst_n(rst_n), .sample_enable(sample_enable), .clk_adj(clk_95),
        .target_val(target_speed_right), .current_val(abs(measured_speed_right)), .pwm_out(pid_out_right)
    );

    // FSM LOGIC
    reg [2:0] prev_move;
    reg [31:0] state_timer;
    reg [31:0] capture_l, capture_r;
    reg [2:0] proceed_sync; reg proceed_latch; reg proceed_active;
    reg [31:0] target_ticks_u32;
    reg [31:0] prev_wall_diff;   
    reg [3:0] dir_bits; 
    
    wire [2:0] move_wire;
    wire [7:0] pos_internal;
    reg stop_done_reg; 
    always @(*) begin pos_out = pos_internal; end

    reg [3:0] manual_flag_counter;
    move_instr sequencer (
        .clk(clk_50M), .rst_n(rst_n), .move_done(stop_done_reg), .proceed(proceed_latch), 
        .flag_count(target_flags), .flag_idx(manual_flag_counter), 
        .ir_right(ir_right_sync), .ir_left(ir_left_sync),
        .left(wall_left), .mid(wall_mid), .right(wall_right), .pos_out(pos_internal), .move(move_wire)
    );
    
    // Flag Index Mapping
    reg [3:0] mapped_flag_idx;
    always @(*) begin
        case (pos_internal)
            8'h06: mapped_flag_idx = 4'd1;
            8'h22: mapped_flag_idx = 4'd2;
            8'h13: mapped_flag_idx = 4'd3;
            8'h01: mapped_flag_idx = 4'd4;
            8'h18: mapped_flag_idx = 4'd5;
            8'h36: mapped_flag_idx = 4'd6;
            8'h89: mapped_flag_idx = 4'd7;
            8'h74: mapped_flag_idx = 4'd8;
            8'h51: mapped_flag_idx = 4'd9;
            default: mapped_flag_idx = 4'd0;
        endcase
    end

    assign current_move = move_wire; assign stop_done = stop_done_reg;
    assign flag_idx_out = mapped_flag_idx;
    assign is_spinning = (sys_state == STATE_SPIN);
    assign is_measuring = (sys_state == STATE_MEASURE);

    reg [7:0] pwm_left_mux, pwm_right_mux;
    always @(*) begin
        target_speed_left = 0; target_speed_right = 0;
        if (sys_state == STATE_SPIN) begin
            pwm_left_mux = pid_out_left; pwm_right_mux = pid_out_right;
            target_speed_left = TARGET_SPIN_SPEED; target_speed_right = TARGET_SPIN_SPEED;
        end else if (sys_state == STATE_DRIVE || sys_state == STATE_APPROACH) begin
            pwm_left_mux = drive_pwm_l; pwm_right_mux = drive_pwm_r;
        end else if (sys_state == STATE_REVERSE) begin
            pwm_left_mux = 8'd180; pwm_right_mux = 8'd180;
        end else begin
            pwm_left_mux = 255; pwm_right_mux = 255;
        end
    end
    assign pwm_left = pwm_left_mux; assign pwm_right = pwm_right_mux;

    // PULSE STRETCHERS
    reg [15:0] app_trig_cnt, meas_trig_cnt, end_trig_cnt;
    reg app_trig_reg, meas_trig_reg, end_trig_reg;
    assign approach_trigger = app_trig_reg;
    assign measure_trigger  = meas_trig_reg;
    assign end_trigger      = end_trig_reg;

    reg first_move_gate;
    reg end_sent_latch;
    reg measurement_done_latch;

    always @(posedge clk_50M) begin
        if (!rst_n) begin
            sys_state <= STATE_IDLE; prev_move <= 3'b111; state_timer <= 0;
            lin1<=1; lin2<=1; rin1<=1; rin2<=1;
            proceed_latch <= 1; stop_done_reg <= 0;
            app_trig_cnt <= 0; meas_trig_cnt <= 0; end_trig_cnt <= 0;
            app_trig_reg <= 0; meas_trig_reg <= 0; end_trig_reg <= 0;
            first_move_gate <= 1; end_sent_latch <= 0; measurement_done_latch <= 0;
            manual_flag_counter <= 0;
        end else begin
            if (app_trig_cnt > 0) begin app_trig_cnt <= app_trig_cnt - 1; app_trig_reg <= 1; end else app_trig_reg <= 0;
            if (meas_trig_cnt > 0) begin meas_trig_cnt <= meas_trig_cnt - 1; meas_trig_reg <= 1; end else meas_trig_reg <= 0;
            if (end_trig_cnt > 0) begin end_trig_cnt <= end_trig_cnt - 1; end_trig_reg <= 1; end else end_trig_reg <= 0;

            if (pos_internal == 8'h40 && stop_done_reg && !end_sent_latch) begin
                end_sent_latch <= 1; end_trig_cnt <= 2000; sys_state <= STATE_HALT;
            end

            proceed_sync <= {proceed_sync[1:0], proceed};
            if (proceed_sync[1] && !proceed_sync[2]) proceed_latch <= 1;
            proceed_active <= proceed_sync[1];

            if (sys_state == STATE_HALT) begin
                lin1<=1; lin2<=1; rin1<=1; rin2<=1;
            end else if (move_wire != prev_move) begin
                if (move_wire == 3'b000 || prev_move == 3'b111) begin
                    sys_state <= STATE_PREP; prev_move <= move_wire;
                end else if (proceed_latch && !proceed_active) begin
                    sys_state <= STATE_PREP; prev_move <= move_wire; stop_done_reg <= 0;
                end
            end else begin
                case (sys_state)
                    STATE_IDLE: begin lin1<=1; lin2<=1; rin1<=1; rin2<=1; end
                    STATE_PREP: begin
                        capture_l <= count_left; capture_r <= count_right; state_timer <= 0;
                        measurement_done_latch <= 0; // Reset for every new move
                        case (move_wire)
                            3'b001: begin dir_bits <= 4'b1010; sys_state <= STATE_DRIVE; end
                            3'b010: begin dir_bits <= 4'b0110; target_ticks_u32 <= TURN_TICKS; next_state <= STATE_SPIN; sys_state <= STATE_BRAKE; end
                            3'b011: begin dir_bits <= 4'b1001; target_ticks_u32 <= TURN_TICKS; next_state <= STATE_SPIN; sys_state <= STATE_BRAKE; end
                            3'b100: begin 
                                if (distance_left > distance_right) dir_bits <= 4'b0110; else dir_bits <= 4'b1001;
                                target_ticks_u32 <= UTURN_TICKS; sys_state <= STATE_CHECK_FWD; 
                            end
                            default: sys_state <= STATE_IDLE;
                        endcase
                    end
                    STATE_APPROACH: begin
                        lin1<=1; lin2<=0; rin1<=1; rin2<=0; 
                        if (state_timer == 0) app_trig_cnt <= 2000;
                        if (ir_right_sync == 1'b0) begin sys_state <= STATE_MEASURE; state_timer <= 0; end
                        else state_timer <= state_timer + 1;
                    end
                    STATE_MEASURE: begin
                        lin1<=1; lin2<=1; rin1<=1; rin2<=1;
                        if (state_timer < MEASURE_TIME) begin
                            state_timer <= state_timer + 1;
                            if (state_timer == 145000000) begin
                                meas_trig_cnt <= 2000;
                                measurement_done_latch <= 1; 
                            end
                        end else begin sys_state <= STATE_UNDIP; state_timer <= 0; end
                    end
                    STATE_UNDIP: begin
                        lin1<=1; lin2<=1; rin1<=1; rin2<=1;
                        if (state_timer < UNDIP_TIME) state_timer <= state_timer + 1; else begin sys_state <= STATE_REVERSE; capture_l <= count_left; capture_r <= count_right; end
                    end
                    STATE_REVERSE: begin
                        lin1<=0; lin2<=1; rin1<=0; rin2<=1;
                        if (abs_diff32(count_left, capture_l) + abs_diff32(count_right, capture_r) >= 2000) begin 
                            next_state <= STATE_SPIN; sys_state <= STATE_BRAKE; 
                            state_timer <= 0; capture_l <= count_left; capture_r <= count_right; prev_wall_diff <= 32'hFFFFFFFF; 
                        end
                    end
                    STATE_CHECK_FWD: begin
                        lin1<=1; lin2<=0; rin1<=1; rin2<=0; if (abs_diff32(count_left, capture_l) + abs_diff32(count_right, capture_r) >= 1600) begin
                            if (ir_left_sync == 1'b0) begin 
                                manual_flag_counter <= manual_flag_counter + 1;
                                sys_state <= STATE_FLAG_ALIGN; state_timer <= 0; 
                            end
                            else begin sys_state <= STATE_CHECK_REV; capture_l <= count_left; capture_r <= count_right; end
                        end
                    end
                    STATE_FLAG_ALIGN: begin
                        lin1<=0; lin2<=1; rin1<=1; rin2<=0; if (state_timer < 5000000) state_timer <= state_timer + 1; else begin sys_state <= STATE_APPROACH; state_timer <= 0; end
                    end
                    STATE_CHECK_REV: begin
                        lin1<=0; lin2<=1; rin1<=0; rin2<=1; 
                        if (abs_diff32(count_left, capture_l) + abs_diff32(count_right, capture_r) >= 2000) begin 
                            next_state <= STATE_SPIN; sys_state <= STATE_BRAKE; 
                            state_timer <= 0; capture_l <= count_left; capture_r <= count_right; prev_wall_diff <= 32'hFFFFFFFF; 
                        end
                    end
                    STATE_SPIN: begin
                        lin1 <= dir_bits[3]; lin2 <= dir_bits[2]; rin1 <= dir_bits[1]; rin2 <= dir_bits[0];
                        if (move_wire == 3'b100) begin
                            if (abs_diff32(count_left, capture_l) + abs_diff32(count_right, capture_r) > 5500) begin
                                if (abs_diff32(distance_left, distance_right) > prev_wall_diff) begin next_state <= STATE_DRIVE; sys_state <= STATE_BRAKE; state_timer <= 0; end
                            end
                            prev_wall_diff <= abs_diff32(distance_left, distance_right);
                        end else begin
                            if (abs_diff32(count_left, capture_l) + abs_diff32(count_right, capture_r) > target_ticks_u32) begin 
                                if (distance_mid > 200) begin
                                    next_state <= STATE_DRIVE; sys_state <= STATE_BRAKE; state_timer <= 0; 
                                end
                            end
                        end
                    end
                    STATE_BRAKE: begin
                        lin1<=1; lin2<=1; rin1<=1; rin2<=1; if (state_timer < BRAKE_TIME) state_timer <= state_timer + 1; 
                        else begin state_timer <= 0; capture_l <= count_left; capture_r <= count_right; sys_state <= next_state; end
                    end
                    STATE_DRIVE: begin
                        lin1<=1; lin2<=0; rin1<=1; rin2<=0;
                        if (distance_mid <= 85) sys_state <= STATE_FINISH;
                        else begin
                            if (move_wire == 3'b100 && measurement_done_latch && (abs_diff32(count_left, capture_l) + abs_diff32(count_right, capture_r) >= 1500)) begin
                                if (distance_mid > 250) sys_state <= STATE_FINISH;
                            end
                            else if (abs_diff32(count_left, capture_l) + abs_diff32(count_right, capture_r) > FWD_MAX_TICKS) begin if (distance_mid > 250) sys_state <= STATE_FINISH; end
                        end
                    end
                    STATE_FINISH: begin lin1<=1; lin2<=1; rin1<=1; rin2<=1; stop_done_reg <= 1; end
                    default: sys_state <= STATE_IDLE;
                endcase
            end
        end
    end
endmodule
