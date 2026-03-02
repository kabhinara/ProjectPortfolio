module t2a_dht(
    input clk_50M,
    input reset,
    inout sensor,
    output reg [7:0] T_integral,
    output reg [7:0] RH_integral,
    output reg [7:0] T_decimal,
    output reg [7:0] RH_decimal,
    output reg [7:0] Checksum,
    output reg data_valid
);

    // Reverting to Turn 33 Logic (The "14" version)
    localparam IDLE           = 3'd0;
    localparam START_LOW      = 3'd1;
    localparam START_HIGH     = 3'd2;
    localparam WAIT_RESP      = 3'd3;
    localparam READ_BIT_LOW   = 3'd4;
    localparam READ_BIT_HIGH  = 3'd5;
    localparam VALIDATE       = 3'd6;

    localparam COUNT_18MS     = 1000000;
    localparam COUNT_40US     = 2000;
    localparam BIT_THRESHOLD  = 2500; // 50us
    localparam POLL_DELAY     = 100000000; // 2s

    reg [2:0] state;
    reg [31:0] timer;
    reg [39:0] buffer;
    reg [5:0] bit_counter;
    
    // Synchronizers
    reg s_sync_0, s_sync_1;
    always @(posedge clk_50M) begin
        s_sync_0 <= sensor;
        s_sync_1 <= s_sync_0;
    end
    
    reg s_prev;
    always @(posedge clk_50M) s_prev <= s_sync_1;
    wire s_negedge = (s_prev && !s_sync_1);
    wire s_posedge = (!s_prev && s_sync_1);

    reg sensor_out, sensor_oe;
    assign sensor = sensor_oe ? sensor_out : 1'bz;

    always @(posedge clk_50M or negedge reset) begin
        if (!reset) begin
            state <= IDLE; timer <= 0; data_valid <= 0;
            T_integral <= 0; RH_integral <= 0; sensor_oe <= 0;
        end else begin
            case (state)
                IDLE: begin
                    sensor_oe <= 0;
                    if (timer < POLL_DELAY) timer <= timer + 1;
                    else begin timer <= 0; state <= START_LOW; end
                end

                START_LOW: begin
                    sensor_oe <= 1; sensor_out <= 0;
                    if (timer < COUNT_18MS) timer <= timer + 1;
                    else begin timer <= 0; state <= START_HIGH; end
                end

                START_HIGH: begin
                    sensor_oe <= 1; sensor_out <= 1;
                    if (timer < COUNT_40US) timer <= timer + 1;
                    else begin timer <= 0; sensor_oe <= 0; state <= WAIT_RESP; end
                end

                WAIT_RESP: begin
                    if (s_negedge) begin
                        timer <= 0; bit_counter <= 0; state <= READ_BIT_LOW;
                    end else if (timer > 50000) begin
                        timer <= 0; state <= IDLE;
                    end else timer <= timer + 1;
                end

                READ_BIT_LOW: begin
                    if (s_posedge) begin timer <= 0; state <= READ_BIT_HIGH; end
                end

                READ_BIT_HIGH: begin
                    if (s_negedge) begin
                        buffer <= {buffer[38:0], (timer > BIT_THRESHOLD ? 1'b1 : 1'b0)};
                        bit_counter <= bit_counter + 1;
                        timer <= 0;
                        if (bit_counter == 39) state <= VALIDATE;
                        else state <= READ_BIT_LOW;
                    end else timer <= timer + 1;
                end

                VALIDATE: begin
                    // SHIFTING FIX: Shift left by 1 bit to correct the handshake slip
                    RH_integral <= buffer[38:31];
                    T_integral  <= buffer[22:15];
                    data_valid  <= 1;
                    timer <= 0; state <= IDLE;
                end
                
                default: state <= IDLE;
            endcase
        end
    end
endmodule
