`default_nettype none

module dht11_min_handshake5(
    input  wire clk,       // 12 MHz clock
    input  wire reset_n,   // Active-low reset
    inout  wire dht_data,  // DHT11 sensor line (bidirectional)
    output reg  done,      // Goes high when reading is complete
    output reg [31:0] raw_data  // 32-bit sensor reading (4 bytes)
);

    // Timing parameters for a 12 MHz clock
    localparam integer CLK_FREQ             = 12_000_000;
    localparam integer T_START_MS           = 18;
    localparam integer T_START_CYCLES       = (CLK_FREQ / 1000) * T_START_MS;  // ~216,000 cycles

    localparam integer T_RELEASE_US         = 8;
    localparam integer T_RELEASE_CYCLES     = (CLK_FREQ / 1_000_000) * T_RELEASE_US; // ~360 cycles

    localparam integer T_BIT_THRESHOLD_US   = 50;
    localparam integer T_BIT_THRESHOLD_CYCLES = (CLK_FREQ / 1_000_000) * T_BIT_THRESHOLD_US; // ~600 cycles

    localparam integer NUM_BITS             = 32;
    localparam integer MIN_HIGH_US          = 80;
    localparam integer MIN_HIGH_CYCLES      = (CLK_FREQ / 1_000_000) * MIN_HIGH_US; // e.g., ~960 cycles
    localparam integer T_LOW_BIT_US         = 54;
    localparam integer T_LOW_BIT_CYCLES     = (CLK_FREQ / 1_000_000) * T_LOW_BIT_US;
    localparam integer T_LOW_BIT_MARGIN     = 8;
    
    // FSM States
    localparam S_IDLE           = 3'd0,
               S_START          = 3'd1,
               S_START_RELEASE  = 3'd2,
               S_WAIT_RESP_LOW  = 3'd3,
               S_WAIT_RESP_HIGH = 3'd4,
               S_READ_BIT_START = 3'd5,
               S_READ_BIT_HIGH  = 3'd6,
               S_DONE           = 3'd7;

    reg [2:0] state;
    reg [17:0] counter;
    reg [4:0]  bit_index;
    
    //  Internal signals to drive the bidirectional line
    wire d_in;
    reg  d_out;
    reg  oe;

    // Instantiate the SB_IO primitive for open-drain operation.
    // (For the Lattice iCE40, this primitive maps directly to the device’s I/O hardware.)
    SB_IO #(
        .PIN_TYPE(6'b1010_01),
        .PULLUP(1'b1)
    ) dht_io (
        .PACKAGE_PIN(dht_data),
        .OUTPUT_ENABLE(oe),
        .D_OUT_0(d_out),
        .D_IN_0(d_in)
    );
    
    // Sensor acquisition FSM
    always @(posedge clk or negedge reset_n) begin
        if (!reset_n) begin
            state     <= S_IDLE;
            counter   <= 0;
            bit_index <= 0;
            raw_data  <= 32'd0;
            d_out     <= 1'b1; 
            oe        <= 1'b0;
            done      <= 1'b0;
        end else begin
            case (state)
                S_IDLE: begin
                    done    <= 1'b0;
                    state   <= S_START;
                    oe      <= 1'b1;      // Drive output
                    d_out   <= 1'b0;      // Pull line low to start
                    counter <= 0;
                end

                S_START: begin
                    if (counter < (T_START_CYCLES - 1))
                        counter <= counter + 1;
                    else begin
                        counter <= 0;
                        d_out   <= 1'b1;  // Release the line
                        state   <= S_START_RELEASE;
                    end
                end

                S_START_RELEASE: begin
                    if (counter == 0) begin
                        oe    <= 1'b0;  // Tri-state the output (allow sensor to drive)
                        d_out <= 1'b1;
                    end
                    if (counter < (T_RELEASE_CYCLES - 1))
                        counter <= counter + 1;
                    else begin
                        counter <= 0;
                        state   <= S_WAIT_RESP_LOW;
                    end
                end

                S_WAIT_RESP_LOW: begin
                    if (d_in == 1'b0) begin
                        state   <= S_WAIT_RESP_HIGH;
                        counter <= 0;
                    end
                end

                S_WAIT_RESP_HIGH: begin
                    if (d_in == 1'b1)
                        counter <= counter + 1;
                    else begin
                        if (counter >= MIN_HIGH_CYCLES) begin
                            state     <= S_READ_BIT_START;
                            bit_index <= 0;
                            raw_data  <= 32'd0;
                            counter   <= 0;
                        end else begin
                            counter <= 0;
                        end
                    end
                end

                S_READ_BIT_START: begin
                    if (d_in == 1'b0) begin
                        if (counter < T_LOW_BIT_CYCLES)
                            counter <= counter + 1;
                        else
                            counter <= T_LOW_BIT_CYCLES;
                    end else begin
                        if (counter >= (T_LOW_BIT_CYCLES - T_LOW_BIT_MARGIN)) begin
                            counter <= 0;
                            state   <= S_READ_BIT_HIGH;
                        end else begin
                            state   <= S_IDLE; // Error: pulse too short; reset cycle.
                        end
                    end
                end

                S_READ_BIT_HIGH: begin
                    if (d_in == 1'b1)
                        counter <= counter + 1;
                    else begin
                        // Shift in a 0 or 1 based on whether the pulse length exceeded the threshold.
                        if (counter < T_BIT_THRESHOLD_CYCLES)
                            raw_data <= { raw_data[30:0], 1'b0 };
                        else
                            raw_data <= { raw_data[30:0], 1'b1 };
                        bit_index <= bit_index + 1;
                        counter   <= 0;
                        if (bit_index == (NUM_BITS - 1))
                            state <= S_DONE;
                        else
                            state <= S_READ_BIT_START;
                    end
                end

                S_DONE: begin
                    done <= 1'b1;  // Assert done once complete.
                    // Optionally, you can remain here (if you want to latch the reading)
                    // or reset to S_IDLE for repeated readings.
                    state <= S_DONE;
                end

                default: state <= S_IDLE;
            endcase
        end
    end

endmodule
