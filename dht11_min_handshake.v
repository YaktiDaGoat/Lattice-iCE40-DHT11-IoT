`default_nettype none

module dht11_min_handshake(
    input  wire clk,       // 12 MHz clock
    input  wire reset_n,   // Active–low reset
    inout  wire dht_data,  // The DHT11 data line
    output wire debug_in,
    output wire [3:0] debug_state,  // 4–bit debug bus (more states)
    output wire uart_tx
);

    // -----------------------------------------------------------
    // DHT Timing Parameters (derived from a 12 MHz clock)
    // -----------------------------------------------------------
    localparam integer CLK_FREQ                 = 12_000_000;
    localparam integer T_START_MS               = 18;
    localparam integer T_START_CYCLES           = (CLK_FREQ / 1000) * T_START_MS; // ~216000 cycles
    localparam integer T_RELEASE_US             = 8;
    localparam integer T_RELEASE_CYCLES         = (CLK_FREQ / 1_000_000) * T_RELEASE_US; // ~360 cycles
    localparam integer T_BIT_THRESHOLD_US       = 50;
    localparam integer T_BIT_THRESHOLD_CYCLES   = (CLK_FREQ / 1_000_000) * T_BIT_THRESHOLD_US; // ~600 cycles
    localparam integer NUM_BITS                 = 16; // capturing first 16 bits (2 bytes)
    localparam integer T_LOW_BIT_US             = 54;
    localparam integer T_LOW_BIT_CYCLES         = (CLK_FREQ / 1_000_000) * T_LOW_BIT_US;
    localparam integer T_LOW_BIT_MARGIN         = 8;

    // -----------------------------------------------------------
    // DHT I/O -- SB_IO primitive instantiation for open–drain behavior
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
    // FSM States: expanded to 4 bits to accommodate additional states.
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

    // Counters and bit index.
    reg [17:0] counter;
    reg [4:0]  bit_index;

    // -----------------------------------------------------------
    // Block RAM signals for accumulating sensor data.
    // -----------------------------------------------------------
    // We use a 16-bit wide dedicated memory to hold the accumulator.
    // The current content of the BRAM is continuously read as "bram_dout"
    // and used in the shift–update operation.
    wire [15:0] bram_dout;
    reg  [15:0] bram_din;
    reg         ram_we;

    // "new_bit" is computed based on the high–pulse width.
    reg         new_bit;

    // Instantiate the 16-bit BRAM module.
    bram_16bit bram_inst (
        .clk(clk),
        .we(ram_we),
        .addr(1'b0),      // Using a fixed address (only one accumulator)
        .din(bram_din),
        .dout(bram_dout)
    );

    // -----------------------------------------------------------
    // Main FSM: DHT11 handshake, bit–reading, and direct BRAM update
    // -----------------------------------------------------------
    always @(posedge clk or negedge reset_n) begin
        if (!reset_n) begin
            state      <= S_IDLE;
            counter    <= 0;
            bit_index  <= 0;
            d_out      <= 1'b1;
            oe         <= 1'b0;
            ram_we     <= 1'b0;
            bram_din   <= 16'd0;  // Reset accumulator to zero.
            new_bit    <= 1'b0;
        end else begin
            case (state)
                S_IDLE: begin
                    state   <= S_START;
                    oe      <= 1'b1;  // Drive line low to initiate DHT11 start.
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
                            state <= S_READ_BIT_START;
                            bit_index <= 0;
                            counter   <= 0;
                        end else begin
                            counter <= 0;
                        end
                    end
                end

                S_READ_BIT_START: begin
                    // Wait until the sensor drives the line high to start measuring the bit pulse.
                    if (d_in == 1'b1) begin
                        counter <= 0;
                        state   <= S_READ_BIT_HIGH;
                    end
                end

                S_READ_BIT_HIGH: begin
                    if (d_in == 1'b1) begin
                        counter <= counter + 1;
                    end else begin
                        // Determine the bit value based on pulse duration.
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
                    // Directly update the BRAM accumulator:
                    // new value = (previous value << 1) | new_bit.
                    // Note that bram_dout holds the current stored value (available one cycle later).
                    bram_din <= { bram_dout[14:0], new_bit };
                    ram_we   <= 1'b1;  // Assert write–enable.
                    state    <= S_WAIT_WRITE;
                end

                S_WAIT_WRITE: begin
                    // One clock cycle for the write to take effect.
                    ram_we <= 1'b0;
                    if (bit_index == NUM_BITS)
                        state <= S_DONE;
                    else
                        state <= S_READ_BIT_START;
                end

                S_DONE: begin
                    // Acquisition complete. The accumulated 16–bit data is now stored in bram_dout.
                    state <= S_DONE;
                end

                default: state <= S_IDLE;
            endcase
        end
    end

    // -----------------------------------------------------------
    // UART Transmit FSM: Send the 2 bytes (upper and lower) over UART
    // -----------------------------------------------------------
    localparam UART_IDLE     = 3'd0,
               UART_LOAD     = 3'd1,
               UART_SEND     = 3'd2,
               UART_WAIT     = 3'd3,
               UART_FINISHED = 3'd4;

    reg [2:0] uart_state;
    reg       uart_send;
    reg [7:0] uart_data;
    reg       byte_index;  // 0: upper byte; 1: lower byte.
    wire      uart_busy;

    always @(posedge clk or negedge reset_n) begin
        if (!reset_n) begin
            uart_state <= UART_IDLE;
            byte_index <= 1'b0;
            uart_send  <= 1'b0;
            uart_data  <= 8'd0;
        end else begin
            case (uart_state)
                UART_IDLE: begin
                    if (state == S_DONE) begin
                        // Load first byte (upper 8 bits from the BRAM accumulator).
                        uart_data  <= bram_dout[15:8];
                        byte_index <= 1'b0;
                        uart_state <= UART_LOAD;
                    end
                end
                UART_LOAD: begin
                    uart_state <= UART_SEND;
                end
                UART_SEND: begin
                    if (!uart_busy && !uart_send) begin
                        uart_send <= 1'b1;  // Trigger transmission.
                    end else begin
                        uart_send <= 1'b0;
                        if (uart_busy)
                            uart_state <= UART_WAIT;
                    end
                end
                UART_WAIT: begin
                    if (!uart_busy && !uart_send) begin
                        if (byte_index == 1'b1)
                            uart_state <= UART_FINISHED;
                        else begin
                            byte_index <= 1'b1;
                            // Load second byte (lower 8 bits)
                            uart_data  <= bram_dout[7:0];
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
    // UART Transmitter Module Instantiation.
    // (Assumes you have a uart_tx module defined elsewhere.)
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
