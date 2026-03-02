module bt_debug (
    input clk_3125,
    input rst_n,
    input bt_debug_enable,
    input [7:0] rx_msg,
    input rx_complete,
    
    // Debug Interface
    input move_done_pulse,
    input approaching_pulse,
    input measure_done_pulse,
    input end_trig,
    input [7:0] pos,
    input [2:0] next_move,
    input wall_l, wall_m, wall_r,
    input [7:0] humidity,
    input [7:0] temperature,
    input [11:0] moisture,
    input [15:0] dist_mid,
    input [3:0] flag_idx,
    output reg proceed,

    output reg bt_ready,
    output reg [3:0] deadend_count,
    output reg busy,
    output tx
);

    // Message buffer (max 32 chars)
    reg [7:0] msg_buf [0:31];
    reg [5:0] msg_len;
    reg [5:0] tx_idx;
    
    // TX interface
    reg [7:0] tx_data;
    reg tx_start;
    wire tx_done;
    
    // FSM states
    localparam S_IDLE        = 0;
    localparam S_BUFFERING   = 1;
    localparam S_VALIDATE    = 2;
    localparam S_SEND_OK     = 3;      
    localparam S_SEND_MSG    = 4;      
    localparam S_SEND_CRLF   = 5;      
    localparam S_TX_WAIT     = 6;
    localparam S_TX_GAP      = 7;      
    localparam S_SEND_DONE   = 8;      
    localparam S_SEND_MPIM   = 9;      
    localparam S_SEND_MM     = 10;
    localparam S_SEND_TH     = 11;
    localparam S_SEND_END    = 12;
    localparam S_HALTED      = 13;

    reg [3:0] state;
    reg [3:0] return_state;
    reg [3:0] post_crlf_state; // State after CRLF
    reg [5:0] ok_idx;  
    reg [5:0] done_idx; 
    reg crlf_idx;      
    reg [11:0] gap_timer; 
    reg move_done_prev;
    reg already_sent_done;
    reg [7:0] report_delay_cnt; 
    reg [2:0] last_valid_move;
    reg report_pending;
    reg mpim_pending;
    reg vals_pending;
    reg end_pending;
    reg [11:0] captured_moisture;

    // Hex Helper
    function [7:0] to_hex;
        input [3:0] val;
        begin
            if (val < 10) to_hex = val + 48;
            else to_hex = val + 55;
        end
    endfunction
    
    uart_tx tx_inst (
        .clk_3125(clk_3125),
        .rst_n(rst_n),
        .data(tx_data),
        .parity_type(1'b0),
        .tx_start(tx_start),
        .tx(tx),
        .tx_done(tx_done)
    );

    always @(posedge clk_3125 or negedge rst_n) begin
        if (!rst_n) begin
            state <= S_IDLE;
            msg_len <= 0;
            tx_idx <= 0;
            tx_start <= 0;
            busy <= 0;
            ok_idx <= 0;
            crlf_idx <= 0;
            bt_ready <= 0;
            deadend_count <= 0;
            gap_timer <= 0;
            proceed <= 0;
            move_done_prev <= 0;
            already_sent_done <= 0;
            report_delay_cnt <= 0;
            report_pending <= 0;
            mpim_pending <= 0;
            vals_pending <= 0;
            end_pending <= 0;
            captured_moisture <= 0;
            last_valid_move <= 0;
        end else begin
            tx_start <= 0;
            if (tx_done || state != S_IDLE) proceed <= 0; 

            move_done_prev <= move_done_pulse;
            if (!move_done_pulse && move_done_prev) already_sent_done <= 0;

            if (approaching_pulse) mpim_pending <= 1;
            if (measure_done_pulse) begin
                vals_pending <= 1;
                captured_moisture <= moisture;
            end
            if (end_trig) end_pending <= 1;

            last_valid_move <= next_move;

            case (state)
                S_IDLE: begin
                    busy <= 0; msg_len <= 0; crlf_idx <= 0; post_crlf_state <= S_IDLE;
                    if (end_pending) begin busy <= 1; done_idx <= 0; end_pending <= 0; state <= S_SEND_END; end
                    else if (mpim_pending) begin busy <= 1; done_idx <= 0; mpim_pending <= 0; state <= S_SEND_MPIM; end
                    else if (vals_pending) begin busy <= 1; done_idx <= 0; vals_pending <= 0; state <= S_SEND_MM; post_crlf_state <= S_SEND_TH; end
                    else if (bt_debug_enable && rx_complete) begin msg_buf[0] <= rx_msg; msg_len <= 1; state <= S_BUFFERING; end
                end
                
                S_BUFFERING: begin
                    if (rx_complete && bt_debug_enable) begin
                        if (msg_len < 31) begin msg_buf[msg_len] <= rx_msg; msg_len <= msg_len + 1; end
                        if (rx_msg == "#") begin busy <= 1; state <= S_VALIDATE; end
                    end
                end

                S_VALIDATE: begin
                    if (msg_buf[0] == "S" && msg_buf[1] == "T" && msg_buf[2] == "A" && 
                        msg_buf[3] == "R" && msg_buf[4] == "T" && msg_buf[5] == "-") begin
                        deadend_count <= msg_buf[6] - 8'h30; bt_ready <= 1; ok_idx <= 0; state <= S_SEND_OK;
                    end else if (msg_buf[0] == "O" && msg_buf[1] == "K") begin
                        proceed <= 1; ok_idx <= 0; state <= S_SEND_OK;
                    end else state <= S_IDLE;
                end

                S_SEND_MPIM: begin
                    case (done_idx)
                        0: tx_data <= "M"; 1: tx_data <= "P"; 2: tx_data <= "I"; 3: tx_data <= "M";
                        4: tx_data <= "-"; 5: tx_data <= to_hex(flag_idx); 6: tx_data <= "-"; 7: tx_data <= "#";
                    endcase
                    tx_start <= 1;
                    return_state <= (done_idx == 7) ? S_SEND_CRLF : S_SEND_MPIM;
                    if (done_idx < 7) done_idx <= done_idx + 1;
                    state <= S_TX_WAIT;
                end

                S_SEND_MM: begin
                    case (done_idx)
                        0: tx_data <= "M"; 1: tx_data <= "M"; 2: tx_data <= "-";
                        3: tx_data <= to_hex(flag_idx); 4: tx_data <= "-";
                        5: tx_data <= (captured_moisture < 12'd1000) ? "M" : "D";
                        6: tx_data <= "-"; 7: tx_data <= "#";
                    endcase
                    tx_start <= 1;
                    if (done_idx == 7) begin
                        done_idx <= 0; return_state <= S_SEND_CRLF; post_crlf_state <= S_SEND_TH;
                    end else begin done_idx <= done_idx + 1; return_state <= S_SEND_MM; end
                    state <= S_TX_WAIT;
                end

                S_SEND_TH: begin
                    case (done_idx)
                        0: tx_data <= "T"; 1: tx_data <= "H"; 2: tx_data <= "-";
                        3: tx_data <= to_hex(flag_idx); 4: tx_data <= "-";
                        5: tx_data <= (temperature / 10) + 8'd48;
                        6: tx_data <= (temperature % 10) + 8'd48;
                        7: tx_data <= "-";
                        8: tx_data <= (humidity / 10) + 8'd48;
                        9: tx_data <= (humidity % 10) + 8'd48;
                        10: tx_data <= "-"; 11: tx_data <= "#";
                    endcase
                    tx_start <= 1;
                    return_state <= (done_idx == 11) ? S_SEND_CRLF : S_SEND_TH;
                    if (done_idx < 11) done_idx <= done_idx + 1;
                    else post_crlf_state <= S_IDLE;
                    state <= S_TX_WAIT;
                end

                S_SEND_END: begin
                    case (done_idx)
                        0: tx_data <= "E"; 1: tx_data <= "N"; 2: tx_data <= "D";
                        3: tx_data <= "-"; 4: tx_data <= "#";
                    endcase
                    tx_start <= 1;
                    return_state <= (done_idx == 4) ? S_HALTED : S_SEND_END;
                    if (done_idx < 4) done_idx <= done_idx + 1;
                    state <= S_TX_WAIT;
                end
                
                S_HALTED: begin busy <= 1; end

                S_SEND_OK: begin
                    case (ok_idx)
                        0: tx_data <= "O"; 1: tx_data <= "K"; 2: tx_data <= "-";
                    endcase
                    tx_start <= 1;
                    if (ok_idx == 2) begin ok_idx <= 0; tx_idx <= 0; state <= S_TX_WAIT; return_state <= S_SEND_MSG; end
                    else begin ok_idx <= ok_idx + 1; state <= S_TX_WAIT; return_state <= S_SEND_OK; end
                end
                
                S_SEND_MSG: begin
                    if (tx_idx < msg_len) begin
                        tx_data <= msg_buf[tx_idx]; tx_start <= 1; tx_idx <= tx_idx + 1;
                        return_state <= S_SEND_MSG; state <= S_TX_WAIT;
                    end else begin crlf_idx <= 0; tx_start <= 1; tx_data <= 8'h23; state <= S_SEND_CRLF; end
                end
                
                S_SEND_CRLF: begin
                    tx_data <= (crlf_idx == 0) ? 8'h0D : 8'h0A;
                    tx_start <= 1;
                    return_state <= (crlf_idx == 0) ? S_SEND_CRLF : post_crlf_state;
                    crlf_idx <= 1; state <= S_TX_WAIT;
                end
                
                S_TX_WAIT: if (tx_done) begin gap_timer <= 0; state <= S_TX_GAP; end
                S_TX_GAP: if (gap_timer < 1000) gap_timer <= gap_timer + 1; else state <= return_state;
            endcase
        end
    end
endmodule
