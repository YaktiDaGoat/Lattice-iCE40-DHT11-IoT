`default_nettype none

module dht11_interface(
    input  wire clk,       // 12 MHz clock
    input  wire reset_n,   // Active-low reset
    inout  wire dht_data,  // DHT11 sensor line
   //output wire debug_in,       // For observation (optional)
    //output [2:0] debug_state,   // 3-bit debug bus
    output reg        data_valid,   // Asserted when measurement is complete
    output reg [31:0] sensor_data   // The 32-bit measurement result
);
    // -----------------------------------------------------
    // Timing parameters (can be adjusted as needed)
    // -----------------------------------------------------
    localparam integer CLK_FREQ = 12_000_000;
    localparam integer T_START_MS     = 18;
    localparam integer T_START_CYCLES = (CLK_FREQ / 1000) * T_START_MS; // ~216,000 cycles
    
    localparam integer T_RELEASE_US   = 8;
    localparam integer T_RELEASE_CYCLES = (CLK_FREQ / 1_000_000) * T_RELEASE_US; // ~360 cycles
    
    localparam integer T_BIT_THRESHOLD_US = 50;
    localparam integer T_BIT_THRESHOLD_CYCLES = (CLK_FREQ / 1_000_000) * T_BIT_THRESHOLD_US; // ~600 cycles
    
    localparam integer NUM_BITS = 32;
    
    localparam integer MIN_HIGH_US = 80;  
    localparam integer MIN_HIGH_CYCLES = (CLK_FREQ / 1_000_000) * MIN_HIGH_US; // e.g. 960 cycles
    
    localparam integer T_LOW_BIT_US = 54;
    localparam integer T_LOW_BIT_CYCLES = (CLK_FREQ / 1_000_000) * T_LOW_BIT_US;
    
    localparam integer T_LOW_BIT_MARGIN = 8;
    
    // -----------------------------------------------------
    // DHT11 I/O via tri-state (SB_IO primitive for open-drain)
    // -----------------------------------------------------
    wire d_in;
    reg  d_out;
    reg  oe;
    
    SB_IO #(
        .PIN_TYPE(6'b1010_01),
        .PULLUP(1'b1)
    ) dht_data_io (
        .PACKAGE_PIN(dht_data),
        .OUTPUT_ENABLE(oe),
        .D_OUT_0(d_out),
        .D_IN_0(d_in)
    );
    
    //assign debug_in = d_in;
    
    // -----------------------------------------------------
    // Main FSM States
    // -----------------------------------------------------
    reg [2:0] state;
    //assign debug_state = state;
    
    localparam S_IDLE           = 3'd0,
               S_START          = 3'd1,
               S_START_RELEASE  = 3'd2,
               S_WAIT_RESP_LOW  = 3'd3,
               S_WAIT_RESP_HIGH = 3'd4,
               S_READ_BIT_START = 3'd5,
               S_READ_BIT_HIGH  = 3'd6,
               S_DONE           = 3'd7;
    
    // -----------------------------------------------------
    // FSM signals: counters, bit index, raw data register
    // -----------------------------------------------------
    reg [17:0] counter;
    reg [4:0]  bit_index;
    reg [31:0] raw_data;
    
    // -----------------------------------------------------
    // Main FSM: Initiate handshake, then read NUM_BITS bits
    // -----------------------------------------------------
    always @(posedge clk or negedge reset_n) begin
        if (!reset_n) begin
            state       <= S_IDLE;
            counter     <= 0;
            bit_index   <= 0;
            raw_data    <= 32'd0;
            sensor_data <= 32'd0;
            data_valid  <= 1'b0;
            d_out       <= 1'b1; 
            oe          <= 1'b0;
        end else begin
            case (state)
                S_IDLE: begin
                    data_valid <= 1'b0;  // clear valid flag
                    state   <= S_START;
                    oe      <= 1'b1;  // drive line
                    d_out   <= 1'b0;  // pull line low to start
                    counter <= 0;
                end
                
                S_START: begin
                    if (counter < (T_START_CYCLES - 1))
                        counter <= counter + 1;
                    else begin
                        counter <= 0;
                        d_out   <= 1'b1;  // release line
                        state   <= S_START_RELEASE;
                    end
                end
                
                S_START_RELEASE: begin
                    if (counter == 0) begin
                        oe <= 1'b0; // tri-state for sensor response
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
                    if (d_in == 1'b1) begin
                        counter <= counter + 1;
                    end else begin
                        if (counter >= MIN_HIGH_CYCLES) begin
                            state <= S_READ_BIT_START;
                            bit_index <= 0;
                            raw_data  <= 32'd0;
                            counter   <= 0;
                        end else begin
                            counter <= 0;
                        end
                    end
                end
                
                S_READ_BIT_START: begin
                    // Measure low time before a bit’s high pulse
                    if (d_in == 1'b0) begin
                        if (counter < T_LOW_BIT_CYCLES)
                            counter <= counter + 1;
                        else
                            counter <= T_LOW_BIT_CYCLES; // saturate counter
                    end else begin
                        if (counter >= (T_LOW_BIT_CYCLES - T_LOW_BIT_MARGIN)) begin
                            counter <= 0;
                            state   <= S_READ_BIT_HIGH;
                        end else begin
                            state <= S_IDLE;  // error: pulse too short
                        end
                    end
                end
                
                S_READ_BIT_HIGH: begin
                    if (d_in == 1'b1)
                        counter <= counter + 1;
                    else begin
                        // Use pulse width to decide 0 or 1.
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
                    data_valid  <= 1'b1;
                    sensor_data <= raw_data;
                    // Remain here until external circuitry resets or re-triggers a measurement.
                end
                
                default: state <= S_IDLE;
            endcase
        end
    end
endmodule
