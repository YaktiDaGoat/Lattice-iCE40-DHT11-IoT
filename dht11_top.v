module dht11_top(
    input  wire clk,        // 12 MHz clock
    input  wire reset_n,    // Active-low reset
    inout  wire dht_data1,  // Sensor 1 DHT11 data pin
    inout  wire dht_data2,  // Sensor 2 DHT11 data pin
    output wire uart_tx,    // UART transmit line
    output [2:0] debug_state  // 3-bit debug state output from TX FSM
);

    // -----------------------------------------------------
    // Instantiate sensor interface for sensor 1
    // -----------------------------------------------------
    wire data_valid1;
    wire [31:0] sensor_data1;
    dht11_interface sensor1 (
        .clk(clk),
        .reset_n(reset_n),
        .dht_data(dht_data1),
        .data_valid(data_valid1),
        .sensor_data(sensor_data1)
    );
    
    // -----------------------------------------------------
    // Instantiate sensor interface for sensor 2
    // -----------------------------------------------------
    wire data_valid2;
    wire [31:0] sensor_data2;
    dht11_interface sensor2 (
        .clk(clk),
        .reset_n(reset_n),
        .dht_data(dht_data2),
        .data_valid(data_valid2),
        .sensor_data(sensor_data2)
    );

    // -----------------------------------------------------
    // UART transmitter instantiation signals
    // -----------------------------------------------------
    reg [7:0] data_byte;
    reg send;
    wire uart_busy;
    
    uart_tx #(
         .CLK_FREQ(12000000),
         .BAUD_RATE(9600)
    ) uart_transmitter (
         .clk(clk),
         .reset_n(reset_n),
         .data(data_byte),
         .send(send),
         .tx(uart_tx),
         .busy(uart_busy)
    );
    
    // -----------------------------------------------------
    // Main FSM states for sequencing sensor data transmissions.
    // Now using 3-bit states.
    // -----------------------------------------------------
    localparam TX_IDLE    = 3'd0,  // Wait for both sensors to have valid data.
               TX_PRE     = 3'd1,  // Wait 10 clock cycles before starting UART output.
               TX_SENSOR1 = 3'd2,  // Transmit sensor1's 4 bytes.
               TX_DELAY   = 3'd3,  // Wait exactly 10 clock cycles.
               TX_SENSOR2 = 3'd4,  // Transmit sensor2's 4 bytes.
               TX_DONE    = 3'd5;  // Final state; the FSM stops here.

    reg [2:0] tx_state;
    assign debug_state = tx_state;  // Debug output shows the current state.
    
    // -----------------------------------------------------
    // Sub-FSM for transmitting one byte via uart_tx.
    // This sub-FSM produces a one-clock send pulse and waits for uart_busy
    // to go active then inactive, indicating the byte has been transmitted.
    // -----------------------------------------------------
    localparam SUB_IDLE      = 2'd0,
               SUB_TRIGGER   = 2'd1,
               SUB_WAIT_DONE = 2'd2;
               
    reg [1:0] sub_state;  // Sub-FSM state for byte transmission.
    reg [1:0] byte_index; // Index for the 4 bytes (0 to 3).
    
    // Latch sensor data once both signals are valid.
    reg [31:0] sensor1_data_reg, sensor2_data_reg;
    reg sensor_data_latched;
    
    // A delay counter for the TX_DELAY state.
    reg [3:0] delay_cnt;  // Now wide enough for 10-cycle delay.
    
    // A pre-delay counter for TX_PRE state (10 cycles).
    reg [3:0] pre_delay_cnt;
    
    // For edge detection: capture the previous value of uart_busy.
    reg prev_uart_busy;
    
    // -----------------------------------------------------
    // Main FSM with integrated sub-FSM for byte transmission.
    // -----------------------------------------------------
    always @(posedge clk or negedge reset_n) begin
        if (!reset_n) begin
            tx_state            <= TX_IDLE;
            sub_state           <= SUB_IDLE;
            byte_index          <= 0;
            send                <= 1'b0;
            sensor_data_latched <= 1'b0;
            sensor1_data_reg    <= 32'd0;
            sensor2_data_reg    <= 32'd0;
            pre_delay_cnt       <= 0;
            delay_cnt           <= 0;
            data_byte           <= 8'd0;
            prev_uart_busy      <= 1'b0;
        end else begin
            // Capture previous uart_busy for edge detection.
            prev_uart_busy <= uart_busy;
            
            case (tx_state)
            
                TX_IDLE: begin
                    // Wait until both sensor interfaces signal valid data.
                    if (data_valid1 && data_valid2 && !sensor_data_latched) begin
                        sensor1_data_reg    <= sensor_data1;
                        sensor2_data_reg    <= sensor_data2;
                        sensor_data_latched <= 1'b1;
                        byte_index          <= 0;
                        sub_state           <= SUB_IDLE;
                        pre_delay_cnt       <= 0;
                        tx_state            <= TX_PRE;  // Proceed to pre-delay state.
                    end
                end
                
                TX_PRE: begin
                    // Wait a total of 10 clock cycles before starting UART output.
                    if (pre_delay_cnt < 10 - 1) begin
                        pre_delay_cnt <= pre_delay_cnt + 1;
                    end else begin
                        pre_delay_cnt <= 0;
                        tx_state      <= TX_SENSOR1;
                    end
                end
                
                TX_SENSOR1: begin
                    // Use the sub-FSM to transmit one byte from sensor1_data_reg.
                    case (sub_state)
                        SUB_IDLE: begin
                            if (!uart_busy) begin
                                case (byte_index)
                                    2'd0: data_byte <= sensor1_data_reg[31:24];
                                    2'd1: data_byte <= sensor1_data_reg[23:16];
                                    2'd2: data_byte <= sensor1_data_reg[15:8];
                                    2'd3: data_byte <= sensor1_data_reg[7:0];
                                    default: data_byte <= 8'd0;
                                endcase
                                send       <= 1'b1; // Issue the send pulse.
                                sub_state  <= SUB_TRIGGER;
                            end
                        end
                        
                        SUB_TRIGGER: begin
                            send       <= 1'b0;
                            sub_state  <= SUB_WAIT_DONE;
                        end
                        
                        SUB_WAIT_DONE: begin
                            // Wait for the UART to assert busy (rising edge) then deassert (falling edge).
                            if (prev_uart_busy && !uart_busy) begin
                                if (byte_index == 3) begin
                                    // Finished transmitting sensor1's bytes; move to TX_DELAY.
                                    byte_index <= 0;
                                    delay_cnt  <= 0;
                                    sub_state  <= SUB_IDLE;
                                    tx_state   <= TX_DELAY;
                                end else begin
                                    byte_index <= byte_index + 1;
                                    sub_state  <= SUB_IDLE;
                                end
                            end
                        end
                    endcase
                end
                
                TX_DELAY: begin
                    // Wait exactly 10 clock cycles between sensor transmissions.
                    if (delay_cnt < 10 - 1) begin
                        delay_cnt <= delay_cnt + 1;
                    end else begin
                        delay_cnt <= 0;
                        byte_index <= 0;
                        sub_state  <= SUB_IDLE;
                        tx_state   <= TX_SENSOR2;
                    end
                end
                
                TX_SENSOR2: begin
                    // Use the sub-FSM to transmit one byte from sensor2_data_reg.
                    case (sub_state)
                        SUB_IDLE: begin
                            if (!uart_busy) begin
                                case (byte_index)
                                    2'd0: data_byte <= sensor2_data_reg[31:24];
                                    2'd1: data_byte <= sensor2_data_reg[23:16];
                                    2'd2: data_byte <= sensor2_data_reg[15:8];
                                    2'd3: data_byte <= sensor2_data_reg[7:0];
                                    default: data_byte <= 8'd0;
                                endcase
                                send      <= 1'b1;
                                sub_state <= SUB_TRIGGER;
                            end
                        end
                        
                        SUB_TRIGGER: begin
                            send      <= 1'b0;
                            sub_state <= SUB_WAIT_DONE;
                        end
                        
                        SUB_WAIT_DONE: begin
                            if (prev_uart_busy && !uart_busy) begin
                                if (byte_index == 3) begin
                                    // Finished sensor2 transmission; move to TX_DONE.
                                    sensor_data_latched <= 1'b0;  // Optionally, allow new data to be latched.
                                    byte_index <= 0;
                                    sub_state  <= SUB_IDLE;
                                    tx_state   <= TX_DONE;
                                end else begin
                                    byte_index <= byte_index + 1;
                                    sub_state  <= SUB_IDLE;
                                end
                            end
                        end
                    endcase
                end
                
                TX_DONE: begin
                    // Remain in TX_DONE (final state).
                    tx_state <= TX_DONE;
                end
                
                default: tx_state <= TX_IDLE;
            endcase
        end
    end
endmodule
