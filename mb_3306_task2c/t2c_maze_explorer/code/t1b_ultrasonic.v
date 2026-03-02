module t1b_ultrasonic(
    input clk_50M, reset, echo_rx,
    output reg trig,
    output op,
    output wire [15:0] distance_out
);

initial begin
    trig = 0;
end

reg [19:0] counter = 0;        // Reduced from 32 to 20 bits (enough for 12ms)
reg [17:0] echo_counter = 0;   // Reduced from 32 to 18 bits (max ~4.4m range)
reg object_present = 0;
reg echo_prev = 0;
reg measuring = 0;
reg [15:0] stored_distance = 0;
reg started = 0;
reg [2:0] pulse_index = 0;     // Reduced from 32 to 3 bits (8 shifts max)

// Distance calculation
wire [15:0] calculated_distance = (echo_counter * 34) / 10000;
localparam SHIFT_STEP = 554;
localparam THRESHOLD_MM = 70;

always @(posedge clk_50M or negedge reset) begin
    if (!reset) begin
        counter <= 0;
        echo_counter <= 0;
        object_present <= 0;
        echo_prev <= 0;
        measuring <= 0;
        trig <= 0;
        stored_distance <= 0;
        started <= 0;
        pulse_index <= 0;
    end else begin
        echo_prev <= echo_rx;
        counter <= counter + 1;

        if (!started && counter >= 50) begin
            started <= 1;
            counter <= 0;
        end

        if (started) begin
            // Trigger pulse
            if ((counter >= (pulse_index * SHIFT_STEP)) && (counter < (pulse_index * SHIFT_STEP + 500)))
                trig <= 1;
            else
                trig <= 0;

            // Rising edge → start measurement
            if (!echo_prev && echo_rx && !measuring) begin
                measuring <= 1;
                echo_counter <= 0;
                object_present <= 0;
            end

            // Count echo width
            if (measuring && echo_rx)
                echo_counter <= echo_counter + 1;

            // Falling edge → calculate distance
            if (echo_prev && !echo_rx && measuring) begin
                measuring <= 0;
                stored_distance <= calculated_distance;

                // 70 mm threshold for obstacle detection
                if (calculated_distance < THRESHOLD_MM)
                    object_present <= 1;
                else
                    object_present <= 0;
            end

            // Reset distance if echo not received
            if (counter == (pulse_index * SHIFT_STEP + 500) && !echo_rx && !measuring) begin
                stored_distance <= 0;
                object_present <= 0;
            end

            // Increment pulse index every cycle
            if (counter >= 599999) begin
                counter <= 0;
                if (pulse_index < 7)  // Prevent overflow
                    pulse_index <= pulse_index + 1;
            end
        end
    end
end

assign op = object_present;
assign distance_out = stored_distance;

endmodule