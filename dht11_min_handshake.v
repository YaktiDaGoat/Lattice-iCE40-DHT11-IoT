`default_nettype none

module dht11_min_handshake(
    input  wire clk,       // 12 MHz clock
    input  wire reset_n,   // Active-low reset
    inout  wire dht_data,  // The DHT11 line
    output wire debug_in,
    output [2:0] debug_state,  // <--- 3-bit debug bus
    output wire uart_tx
);
    /* 
        -------------------------------------
                DHT FSM PARAMETERS
        -------------------------------------
    */
    localparam integer CLK_FREQ = 12_000_000;
    localparam integer T_START_MS     = 18;
    localparam integer T_START_CYCLES = CLK_FREQ / 1000 * T_START_MS; // ~216,000

    localparam integer T_RELEASE_US   = 8;
    localparam integer T_RELEASE_CYCLES = CLK_FREQ / 1_000_000 * T_RELEASE_US; // ~360

    localparam integer T_BIT_THRESHOLD_US = 50;
    localparam integer T_BIT_THRESHOLD_CYCLES = (CLK_FREQ / 1_000_000) * T_BIT_THRESHOLD_US; // ~600

    localparam integer NUM_BITS = 32;

    localparam integer MIN_HIGH_US = 80;  // desired minimum high pulse (in µs)
    localparam integer MIN_HIGH_CYCLES = (CLK_FREQ / 1_000_000) * MIN_HIGH_US; // e.g. 12*80 = 960 cycles    localparam integer MIN_
   
    localparam integer T_LOW_BIT_US = 54;
    localparam integer T_LOW_BIT_CYCLES = (CLK_FREQ / 1_000_000) * T_LOW_BIT_US;

    localparam integer T_LOW_BIT_MARGIN = 8;
    
    /* 
        -----------------------------------------------------
        DHT I/O (SB_IO Primitive for Open-Drain / tri-state)
        -----------------------------------------------------
    */

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

    assign debug_in = d_in;

    /* 
        ----------------
        Main FSM States
        ----------------
    */
    
    reg [2:0] state;
    assign debug_state = state;  // <--- map lower 3 bits to debug output pins

    localparam S_IDLE           = 3'd0,
               S_START          = 3'd1,
               S_START_RELEASE  = 3'd2,
               S_WAIT_RESP_LOW  = 3'd3,
               S_WAIT_RESP_HIGH = 3'd4,
               S_READ_BIT_START = 3'd5,
               S_READ_BIT_HIGH  = 3'd6,
               S_DONE           = 3'd7;

    /* 
        ----------------
        Main FSM Signal
        ----------------
    */
    reg [17:0] counter;
    reg [4:0]  bit_index;
    reg [31:0] raw_data;
    //reg [15:0] delay_counter;  // New delay counter

    /*wire [7:0] rh_int     = raw_data[39:32];
    wire [7:0] rh_dec     = raw_data[31:24];
    wire [7:0] t_int      = raw_data[23:16];
    wire [7:0] t_dec      = raw_data[15: 8];
    wire [7:0] checksum   = raw_data[ 7: 0];*/

    wire [7:0] rh_int     = raw_data[31:24];
    wire [7:0] rh_dec     = raw_data[23:16];
    wire [7:0] t_int      = raw_data[15: 8];
    wire [7:0] t_dec      = raw_data[ 7: 0];

    /* 
        ----------------
        Main FSM
        ----------------
    */
    always @(posedge clk or negedge reset_n) begin
        if (!reset_n) begin
            state       <= S_IDLE;
            counter     <= 0;
            bit_index   <= 0;
            raw_data    <= 32'd0;
            d_out       <= 1'b1; 
            oe          <= 1'b0;
        end else begin
            case (state)
                S_IDLE: begin
                    state   <= S_START;
                    oe      <= 1'b1;  
                    d_out   <= 1'b0;  
                    counter <= 0;
                end

                S_START: begin
                    if (counter < (T_START_CYCLES - 1))
                        counter <= counter + 1;
                    else begin
                        counter <= 0;
                        d_out   <= 1'b1; 
                        state   <= S_START_RELEASE;
                    end
                end

                S_START_RELEASE: begin
                    if (counter == 0) begin
                        oe <= 1'b0; // tri-state
                        d_out <= 1'b1;
                    end
                    if (counter < (T_RELEASE_CYCLES - 1)) begin
                        counter <= counter + 1;
                    end else begin
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
                        /*state     <= S_READ_BIT_START;
                        bit_index <= 0;
                        raw_data  <= 40'd0;
                        counter   <= 0;*/
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
                    // <-- CHANGED logic here
                    if (d_in == 1'b0) begin
                        // Count how long it stays low, up to a limit
                        if (counter < T_LOW_BIT_CYCLES) begin
                            counter <= counter + 1;
                        end else begin
                            // If the sensor holds it low even longer,
                            // just saturate (don’t let counter roll over)
                            counter <= T_LOW_BIT_CYCLES;
                        end
                    end else begin
                        // The line went high; see if we got enough low time
                        if (counter >= (T_LOW_BIT_CYCLES - T_LOW_BIT_MARGIN)) begin
                            // Good enough => proceed
                            counter <= 0;
                            state   <= S_READ_BIT_HIGH;
                        end else begin
                            // The low pulse was too short => out of spec
                            // You can reset or handle an error:
                            state   <= S_IDLE;
                        end
                    end
                end
                    /*if (d_in == 1'b0) begin
                        if( counter < T_LOW_BIT_CYCLES)
                            counter <= counter + 1;
                        else begin
                            counter <= 0;
                            state   <= S_READ_BIT_HIGH;
                        end
                    end else begin
                        counter <= 0;
                    end 
                end*/

                S_READ_BIT_HIGH: begin
                    if (d_in == 1'b1) begin
                        counter <= counter + 1;
                    end else begin
                        if (counter < T_BIT_THRESHOLD_CYCLES)
                            raw_data <= { raw_data[30:0], 1'b0 };
                        else
                            raw_data <= { raw_data[30:0], 1'b1 };

                        bit_index <= bit_index + 1;
                        counter   <= 0;

                        if (bit_index == (NUM_BITS-1))
                            state <= S_DONE;
                        else
                            state <= S_READ_BIT_START;
                    end
                end

                S_DONE: begin
                end

                default: state <= S_IDLE;
            endcase
        end
    end

    // -----------------------------------------------------------
    // UART Transmit FSM: Send the 5 bytes (rh_int, rh_dec, t_int, t_dec, checksum)
    // over UART once the main FSM reaches S_DONE.
    // -----------------------------------------------------------
    // We'll use a simple UART transmitter module (defined below) with BAUD_RATE = 115200.
    //
    // This FSM loads the five bytes into an array and then sends them one by one.
    //
    localparam UART_IDLE     = 3'd0,
               UART_LOAD     = 3'd1,
               UART_SEND     = 3'd2,
               UART_WAIT     = 3'd3,
               UART_FINISHED = 3'd4;
    reg [2:0] uart_state;
    reg [1:0] byte_index; // from 0 to 4
    reg [7:0] byte_array [0:3];
    reg [7:0] uart_data;
    reg       uart_send;
    wire      uart_busy;

    always @(posedge clk or negedge reset_n) begin
        if (!reset_n) begin
            uart_state <= UART_IDLE;
            byte_index <= 0;
            uart_send  <= 1'b0;
            uart_data  <= 8'd0;
        end else begin
            case (uart_state)
                UART_IDLE: begin
                    // Wait until the main FSM is in S_DONE.
                    if (state == S_DONE) begin
                        // Load the 4 bytes from raw_data.
                        byte_array[0] <= rh_int;
                        byte_array[1] <= rh_dec;
                        byte_array[2] <= t_int;
                        byte_array[3] <= t_dec;
                        byte_index <= 0;
                        uart_state <= UART_LOAD;
                    end
                end
                UART_LOAD: begin
                    uart_data <= byte_array[byte_index];
                    uart_state <= UART_SEND;
                end
                UART_SEND: begin
                    if (!uart_busy && !uart_send) begin
                        uart_send <= 1'b1; // Trigger transmission of current byte.
                    end else begin
                        uart_send <= 1'b0;
                        if (uart_busy)
                            uart_state <= UART_WAIT;
                    end
                end
                UART_WAIT: begin
                    if (!uart_busy && !uart_send) begin
                        if (byte_index == 3)
                            uart_state <= UART_FINISHED;
                        else begin
                            byte_index <= byte_index + 1;
                            uart_state <= UART_LOAD;
                        end
                    end
                end
                UART_FINISHED: begin
                    // Once finished, remain here.
                    uart_state <= UART_FINISHED;
                end
                default: uart_state <= UART_IDLE;
            endcase
        end
    end

    // -----------------------------------------------------------
    // UART Transmitter Module Instantiation
    // -----------------------------------------------------------
    uart_tx #(
        .CLK_FREQ(CLK_FREQ),
        .BAUD_RATE(115200)
    ) uart_transmitter (
        .clk(clk),
        .reset_n(reset_n),
        .data(uart_data),
        .send(uart_send),
        .tx(uart_tx),
        .busy(uart_busy)
    );

endmodule

