`default_nettype none

module dht11_min_handshake(
    input  wire clk,       // 12 MHz clock
    input  wire reset_n,   // Active–low reset
    inout  wire dht_data,  // DHT11 data line
    output wire debug_in,
    output wire [3:0] debug_state,  // 4–bit debug bus
    output wire uart_tx
);

    // -----------------------------------------------------------
    // Timing Parameters (derived from a 12 MHz clock)
    // -----------------------------------------------------------
    localparam integer CLK_FREQ                 = 12_000_000;
    localparam integer T_START_MS               = 18;
    localparam integer T_START_CYCLES           = (CLK_FREQ / 1000) * T_START_MS; // ~216,000 cycles
    localparam integer T_RELEASE_US             = 8;
    localparam integer T_RELEASE_CYCLES         = (CLK_FREQ / 1_000_000) * T_RELEASE_US; // ~360 cycles
    localparam integer T_BIT_THRESHOLD_US       = 50;
    localparam integer T_BIT_THRESHOLD_CYCLES   = (CLK_FREQ / 1_000_000) * T_BIT_THRESHOLD_US; // ~600 cycles
    localparam integer NUM_BITS                 = 32; // Full 32 bits from the DHT11.
    localparam integer T_LOW_BIT_US             = 54;
    localparam integer T_LOW_BIT_CYCLES         = (CLK_FREQ / 1_000_000) * T_LOW_BIT_US;
    localparam integer T_LOW_BIT_MARGIN         = 8;

    // -----------------------------------------------------------
    // DHT I/O (using SB_IO for open–drain operation)
    // -----------------------------------------------------------
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

    // -----------------------------------------------------------
    // FSM States (expanded to 4 bits for additional states)
    // -----------------------------------------------------------
    localparam S_IDLE           = 4'd0,
               S_START          = 4'd1,
               S_START_RELEASE  = 4'd2,
               S_WAIT_RESP_LOW  = 4'd3,
               S_WAIT_RESP_HIGH = 4'd4,
               S_READ_BIT_START = 4'd5,
               S_READ_BIT_HIGH  = 4'd6,
               S_UPDATE         = 4'd7,
               S_WAIT_WRITE     = 4'd8,
               S_DONE           = 4'd9;

    reg [3:0] state;
    assign debug_state = state;

    // Counters and Bit Index (6 bits needed for 32 bits)
    reg [17:0] counter;
    reg [5:0]  bit_index;

    // -----------------------------------------------------------
    // BRAM Signals for 32–bit accumulator (split into two 16–bit parts)
    // -----------------------------------------------------------
    wire [15:0] bram_hi_dout, bram_lo_dout;
    reg  [15:0] bram_hi_din, bram_lo_din;
    reg         ram_we;  // Write enable for both BRAMs

    // New bit computed from the pulse width.
    reg         new_bit;

    // Instantiate two 16–bit block RAMs: one for the high word and one for the low word.
    bram_16bit bram_hi (
        .clk(clk),
        .we(ram_we),
        .addr(1'b0),
        .din(bram_hi_din),
        .dout(bram_hi_dout)
    );

    bram_16bit bram_lo (
        .clk(clk),
        .we(ram_we),
        .addr(1'b0),
        .din(bram_lo_din),
        .dout(bram_lo_dout)
    );

    // -----------------------------------------------------------
    // Main FSM: DHT11 handshake and direct update of the 32–bit accumulator.
    // -----------------------------------------------------------
    always @(posedge clk or negedge reset_n) begin
        if (!reset_n) begin
            state      <= S_IDLE;
            counter    <= 0;
            bit_index  <= 0;
            d_out      <= 1'b1;
            oe         <= 1'b0;
            ram_we     <= 1'b0;
            bram_hi_din<= 16'd0;
            bram_lo_din<= 16'd0;
            new_bit    <= 1'b0;
        end else begin
            case (state)
                S_IDLE: begin
                    state   <= S_START;
                    oe      <= 1'b1;  // Drive line low to start DHT11 transaction.
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
                        oe    <= 1'b0;  // Release the line (tri–state)
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
                        if (counter >= T_LOW_BIT_CYCLES) begin
                            state     <= S_READ_BIT_START;
                            bit_index <= 0;
                            counter   <= 0;
                        end else
                            counter <= 0;
                    end
                end

                S_READ_BIT_START: begin
                    // Wait until the sensor drives the line high for bit measurement.
                    if (d_in == 1'b1) begin
                        counter <= 0;
                        state   <= S_READ_BIT_HIGH;
                    end
                end

                S_READ_BIT_HIGH: begin
                    if (d_in == 1'b1)
                        counter <= counter + 1;
                    else begin
                        // Compare the pulse width to decide if bit is 0 or 1.
                        if (counter < T_BIT_THRESHOLD_CYCLES)
                            new_bit <= 1'b0;
                        else
                            new_bit <= 1'b1;

                        bit_index <= bit_index + 1;
                        counter   <= 0;
                        state     <= S_UPDATE;
                    end
                end

                S_UPDATE: begin
                    // Update the 32–bit accumulator (split across two BRAMs)
                    // New accumulator = { bram_hi_dout[14:0], bram_lo_dout[15],
                    //                     bram_lo_dout[14:0], new_bit }
                    bram_hi_din <= { bram_hi_dout[14:0], bram_lo_dout[15] };
                    bram_lo_din <= { bram_lo_dout[14:0], new_bit };
                    ram_we      <= 1'b1;  // Write the updated value.
                    state       <= S_WAIT_WRITE;
                end

                S_WAIT_WRITE: begin
                    ram_we <= 1'b0;
                    if (bit_index == NUM_BITS)
                        state <= S_DONE;
                    else
                        state <= S_READ_BIT_START;
                end

                S_DONE: begin
                    // Final accumulator (32 bits) is now available via {bram_hi_dout, bram_lo_dout}.
                    state <= S_DONE;
                end

                default: state <= S_IDLE;
            endcase
        end
    end

    // -----------------------------------------------------------
    // UART Transmit FSM: Send 4 bytes (32 bits) over UART.
    // -----------------------------------------------------------
    localparam UART_IDLE     = 3'd0,
               UART_LOAD     = 3'd1,
               UART_SEND     = 3'd2,
               UART_WAIT     = 3'd3,
               UART_FINISHED = 3'd4;

    reg [2:0] uart_state;
    reg [1:0] byte_index;  // 0 to 3: four bytes.
    reg [7:0] uart_data;
    reg       uart_send;
    wire      uart_busy;

    always @(posedge clk or negedge reset_n) begin
        if (!reset_n) begin
            uart_state <= UART_IDLE;
            byte_index <= 2'd0;
            uart_send  <= 1'b0;
            uart_data  <= 8'd0;
        end else begin
            case (uart_state)
                UART_IDLE: begin
                    if (state == S_DONE) begin
                        // Load first byte: upper 8 bits of the high word.
                        uart_data  <= bram_hi_dout[15:8];
                        byte_index <= 2'd0;
                        uart_state <= UART_LOAD;
                    end
                end
                UART_LOAD: begin
                    uart_state <= UART_SEND;
                end
                UART_SEND: begin
                    if (!uart_busy && !uart_send) begin
                        uart_send <= 1'b1;
                    end else begin
                        uart_send <= 1'b0;
                        if (uart_busy)
                            uart_state <= UART_WAIT;
                    end
                end
                UART_WAIT: begin
                    if (!uart_busy && !uart_send) begin
                        if (byte_index == 2'd3)
                            uart_state <= UART_FINISHED;
                        else begin
                            byte_index <= byte_index + 1;
                            case (byte_index + 1)
                                2'd1: uart_data <= bram_hi_dout[7:0];   // second byte (low part of high word)
                                2'd2: uart_data <= bram_lo_dout[15:8];  // third byte (upper part of low word)
                                2'd3: uart_data <= bram_lo_dout[7:0];   // fourth byte (low part of low word)
                            endcase
                            uart_state <= UART_LOAD;
                        end
                    end
                end
                UART_FINISHED: begin
                    uart_state <= UART_FINISHED;
                end
                default: uart_state <= UART_IDLE;
            endcase
        end
    end

    // -----------------------------------------------------------
    // UART Transmitter Instantiation
    // Assumes the presence of a compatible uart_tx module.
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
