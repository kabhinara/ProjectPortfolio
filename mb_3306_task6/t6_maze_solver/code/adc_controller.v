module adc_controller(
    input dout, adc_sck,
    input rst_n, // Added reset
    output adc_cs_n, din, 
    output reg [11:0] d_out_ch0,
    output reg [7:0] led_ind
);
    parameter MIN = 640;
    parameter MAX = 1390;
    parameter STEP = (MAX - MIN) / 8;

    reg [3:0] din_counter;
    reg [3:0] sp_counter;
    reg adc_cs;
    reg din_temp; 
    reg [11:0] dout_chx;
    integer count;
    
    // data writing on negedge.
    always @(negedge adc_sck or negedge rst_n) begin
        if (!rst_n) begin
            din_counter <= 0;
            adc_cs <= 1;
            din_temp <= 0;
        end else begin
            din_counter <= din_counter + 1;
            // chip select
            if(din_counter == 0) begin
                adc_cs <= !adc_cs;
            end
        end
    end

    always @(posedge adc_sck or negedge rst_n) begin
        if (!rst_n) begin
            sp_counter <= 0;
            dout_chx <= 0;
        end else begin
            // read the adc value in between 4th sclk cycle and 15th sclk cylce.
            if((sp_counter >= 4) && (sp_counter <= 15)) begin
                dout_chx[15 - sp_counter] <= dout; // fill in the data
            end else begin
                dout_chx <= 0; // reset the dout_chx
            end
            sp_counter <= sp_counter + 1'b1;
        end
    end

    // Latch data safely when frame rolls over to 0
    always @(posedge adc_sck or negedge rst_n) begin
        if (!rst_n) begin
            d_out_ch0 <= 0;
        end else if ((sp_counter == 0) && (!adc_cs)) begin
            d_out_ch0 <= dout_chx;
        end
    end

    always @(*) begin
        if (d_out_ch0 < MIN)
            count = 1;
        else if (d_out_ch0 >= MAX)
            count = 8;
        else
            count = (d_out_ch0 - MIN) / STEP + 1;

        led_ind = (8'hFF >> (8 - count));  // generates 0000_0001 -> 1111_1111
    end

    assign adc_cs_n = adc_cs;
    assign din = din_temp;

endmodule