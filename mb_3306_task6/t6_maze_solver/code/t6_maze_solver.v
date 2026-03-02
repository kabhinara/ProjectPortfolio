module t6_maze_solver (
    input clk_50M,
    // Encoders
    input len1, len2, ren1, ren2,
    // Ultrasonics
    input echo_mid, echo_left, echo_right,
    // IR sensors
    input ir_right, ir_left,
    // Motor outputs
    output lin1, lin2, rin1, rin2, enar, enal,
    // Ultrasonic triggers
    output trig_mid, trig_left, trig_right,
    // LEDs
    output [5:0] leds,
    // UART
    output tx,
    input rx,
    // Sensors
    inout dht_data,
    output adc_cs_n, adc_sck, adc_din,
    input adc_dout,
    output servo_pwm
);

    // ============================================
    // 0. POWER-ON RESET (20ms warmup @ 50MHz)
    // ============================================
    reg por_n = 0;
    reg [19:0] rst_cnt = 0;
    always @(posedge clk_50M) begin
        if (rst_cnt < 20'd1000000) begin
            rst_cnt <= rst_cnt + 1;
            por_n   <= 0;
        end else
            por_n <= 1;
    end

    // ============================================
    // 1. CLOCK GENERATION (3.125 MHz for UART)
    // ============================================
    wire clk_3125;
    frequency_scaling uart_clk_gen (
        .clk_16x(clk_50M),
        .rst_n(por_n),
        .clk_1x(clk_3125)
    );

    // ============================================
    // 2. SENSORS (DHT11, Moisture, Servo)
    // ============================================
    wire [7:0] humidity_int;
    wire [7:0] temp_int;
    wire [11:0] moisture_val;
    wire dht_valid;

    t2a_dht u_dht (
        .clk_50M(clk_50M),
        .reset(por_n),
        .sensor(dht_data),
        .T_integral(temp_int),
        .RH_integral(humidity_int),
        .T_decimal(),
        .RH_decimal(),
        .Checksum(),
        .data_valid(dht_valid)
    );

    adc_controller u_adc (
        .adc_sck(clk_3125), // Use slower clock for ADC SPI
        .rst_n(por_n),
        .dout(adc_dout),
        .adc_cs_n(adc_cs_n),
        .din(adc_din),
        .d_out_ch0(moisture_val),
        .led_ind()
    );
    assign adc_sck = clk_3125; // Connect clock directly

    servo_control u_servo (
        .clk_3125(clk_3125),
        .rst_n(por_n),
        .angle_index(servo_angle_reg),
        .servo_pwm(servo_pwm)
    );

    // ============================================
    // 3. UART RX
    // ============================================
    wire [7:0] rx_msg;
    wire rx_parity;
    wire rx_complete;

    uart_rx u_rx (
        .clk_3125(clk_3125),
        .rst_n(por_n),
        .rx(rx),
        .rx_msg(rx_msg),
        .rx_parity(rx_parity),
        .rx_complete(rx_complete)
    );

    // ============================================
    // 4. BLUETOOTH PARSER (bt_debug)
    // ============================================
    wire tx_bt;
    wire bt_busy;
    wire bt_ready_async;
    wire [3:0] deadend_count;
    
    // Debug Signals
    wire proceed_async;
    wire [7:0] pos_val;
    wire [2:0] move_val;
    wire [3:0] flag_idx_val;
    wire approach_trig_sig, measure_trig_sig, end_trig_sig;
    wire w_left, w_mid, w_right;
    wire stop_done_sig;
    
    wire stop_done_sync_bt;
    synchronizer sync_stop_done (
        .clk(clk_3125),
        .in(stop_done_sig),
        .out(stop_done_sync_bt)
    );

    wire app_sync_bt, meas_sync_bt, end_sync_bt;
    synchronizer sync_app (
        .clk(clk_3125),
        .in(approach_trig_sig),
        .out(app_sync_bt)
    );
    synchronizer sync_meas (
        .clk(clk_3125),
        .in(measure_trig_sig),
        .out(meas_sync_bt)
    );
    synchronizer sync_end (
        .clk(clk_3125),
        .in(end_trig_sig),
        .out(end_sync_bt)
    );

    wire [15:0] d_mid_val;

    bt_debug u_bt_debug (
        .clk_3125(clk_3125),
        .rst_n(por_n),
        .bt_debug_enable(1'b1),
        .rx_msg(rx_msg),
        .rx_complete(rx_complete),
        
        .move_done_pulse(stop_done_sync_bt),
        .approaching_pulse(app_sync_bt),
        .measure_done_pulse(meas_sync_bt),
        .end_trig(end_sync_bt),
        .pos(pos_val),
        .next_move(move_val),
        .wall_l(w_left), .wall_m(w_mid), .wall_r(w_right),
        .humidity(humidity_int),
        .temperature(temp_int),
        .moisture(moisture_val),
        .dist_mid(d_mid_val),
        .flag_idx(flag_idx_val),
        .proceed(proceed_async),

        .bt_ready(bt_ready_async),
        .deadend_count(deadend_count),
        .busy(bt_busy),
        .tx(tx_bt)
    );

    assign tx = tx_bt;

    // ============================================
    // 4. CDC SYNCHRONIZERS
    // ============================================
    wire bt_ready_sync;
    synchronizer sync_bt_ready (
        .clk(clk_50M),
        .in(bt_ready_async),
        .out(bt_ready_sync)
    );

    wire proceed_sync;
    synchronizer sync_proceed (
        .clk(clk_50M),
        .in(proceed_async),
        .out(proceed_sync)
    );

    // ============================================
    // 5. SYSTEM RESET
    //    Robot waits for BT "START-N-#" before moving
    // ============================================
    wire rst_n = por_n & bt_ready_sync;

    // Servo Control Logic
    reg first_move_done;
    wire [15:0] d_left, d_right;
    reg [3:0] servo_angle_reg;
    
    always @(posedge clk_50M or negedge por_n) begin
        if (!por_n) begin
            first_move_done <= 0;
        end else begin
            if (stop_done_sig && rst_n) first_move_done <= 1;
        end
    end

    always @(*) begin
        if (is_measuring_sig) begin
            servo_angle_reg = 4'd8; // Dip position (8125 pulses)
        end else if (is_spinning_sig) begin
            servo_angle_reg = 4'd0; // 33.75 deg Anticlockwise offset (3824 pulses)
        end else begin
            servo_angle_reg = 4'd1; // Default position (5120 pulses)
        end
    end

    // ============================================
    // 6. MOTOR DRIVER
    // ============================================
    motor_driver l298n (
        .clk_50M(clk_50M),
        .rst_n(rst_n),
        .len1(len1), .len2(len2),
        .ren1(ren1), .ren2(ren2),
        .echo_mid(echo_mid),
        .echo_left(echo_left),
        .echo_right(echo_right),
        .ir_right(ir_right),
        .ir_left(ir_left),
        .proceed(proceed_sync),
        .target_flags(deadend_count),
        .pos_out(pos_val),
        .lin1(lin1), .lin2(lin2),
        .rin1(rin1), .rin2(rin2),
        .enal(enal), .enar(enar),
        .trig_mid(trig_mid),
        .trig_left(trig_left),
        .trig_right(trig_right),
        .current_move(move_val),
        .stop_done(stop_done_sig),
        .flag_idx_out(flag_idx_val),
        .is_spinning(is_spinning_sig),
        .is_measuring(is_measuring_sig),
        .approach_trigger(approach_trig_sig),
        .measure_trigger(measure_trig_sig),
        .end_trigger(end_trig_sig),
        .wall_left(w_left),
        .wall_mid(w_mid),
        .wall_right(w_right),
        .dist_left_out(),
        .dist_right_out(),
        .dist_mid_out(d_mid_val)
    );

    // ============================================
    // 7. LEDs
    // ============================================
    assign leds[2:0] = move_val;       // Current move
    assign leds[3]   = w_right;        // Wall right
    assign leds[4]   = w_left;         // Wall left
    assign leds[5]   = bt_ready_sync;  // BT ready (robot enabled)

endmodule
