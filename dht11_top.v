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
        //.debug_in(),          // not routed
        //.debug_state(),       // not routed
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
        //.debug_in(),
        //.debug_state(),
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
         .BAUD_RATE(115200)
    ) uart_transmitter (
         .clk(clk),
         .reset_n(reset_n),
         .data(data_byte),
         .send(send),
         .tx(uart_tx),
         .busy(uart_busy)
    );
    
    // -----------------------------------------------------
    // Main FSM states for sequencing sensor data transmissions
    // (Now using 3 bits)
    // -----------------------------------------------------
    localparam TX_IDLE    = 3'd0,  // Wait for both sensors to have valid data and latch them.
               TX_SENSOR1 = 3'd1,  // Transmit sensor1’s 4 bytes.
               TX_DELAY   = 3'd2,  // 2-clock cycle delay.
               TX_SENSOR2 = 3'd3,  // Transmit sensor2’s 4 bytes.
               TX_DONE    = 3'd4;  // Final state; do nothing (freeze).
               
    reg [2:0] tx_state;
    assign debug_state = tx_state;  // Debug port (0: idle, 1: sensor1, 2: delay, 3: sensor2, 4: done)
    
    // -----------------------------------------------------
    // Sub-FSM for transmitting one byte using uart_tx.
    // We'll use these three states to generate a one-clock send pulse
    // and wait for UART busy to rise then fall.
    // -----------------------------------------------------
    localparam SUB_IDLE      = 2'd0,
               SUB_TRIGGER   = 2'd1,
               SUB_WAIT_DONE = 2'd2;
    
    reg [1:0] sub_state;  // Sub-FSM state for byte transmission
    reg [1:0] byte_index; // To index the 4 bytes (0 to 3)
    
    // We'll latch sensor data in TX_IDLE.
    reg [31:0] sensor1_data_reg, sensor2_data_reg;
    reg sensor_data_latched;
    
    // A simple delay counter for TX_DELAY state.
    reg [1:0] delay_cnt;
    
    // To detect the completion of a byte, capture uart_busy from the previous clock.
    reg prev_uart_busy;
    
    // -----------------------------------------------------
    // Main FSM with integrated sub-FSM for byte transmissions.
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
            delay_cnt           <= 0;
            data_byte           <= 8'd0;
            prev_uart_busy      <= 1'b0;
        end else begin
            // Capture previous uart_busy value for edge detection.
            prev_uart_busy <= uart_busy;
            
            case (tx_state)
            
                TX_IDLE: begin
                    // Wait for both sensors to have valid data.
                    if (data_valid1 && data_valid2 && !sensor_data_latched) begin
                        sensor1_data_reg    <= sensor_data1;
                        sensor2_data_reg    <= sensor_data2;
                        sensor_data_latched <= 1'b1;
                        byte_index          <= 0;
                        sub_state           <= SUB_IDLE;
                        tx_state            <= TX_SENSOR1;  // start with sensor1 transmission.
                    end
                end
                
                TX_SENSOR1: begin
                    // Sub-FSM to transmit one byte from sensor1_data_reg.
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
                                send       <= 1'b1;          // Trigger UART transmission.
                                sub_state  <= SUB_TRIGGER;
                            end
                        end
                        
                        SUB_TRIGGER: begin
                            // End the send pulse.
                            send <= 1'b0;
                            sub_state <= SUB_WAIT_DONE;
                        end
                        
                        SUB_WAIT_DONE: begin
                            // Wait for UART busy to go high then fall (byte complete)
                            if (prev_uart_busy && !uart_busy) begin
                                if (byte_index == 3) begin
                                    // Finished sensor1; move to delay state.
                                    byte_index <= 0;
                                    delay_cnt  <= 0;
                                    sub_state  <= SUB_IDLE;
                                    tx_state   <= TX_DELAY;
                                end else begin
                                    // Move on to the next byte.
                                    byte_index <= byte_index + 1;
                                    sub_state  <= SUB_IDLE;
                                end
                            end
                        end
                    endcase
                end
                
                TX_DELAY: begin
                    if (delay_cnt < 2 - 1) begin
                        delay_cnt <= delay_cnt + 1;
                    end else begin
                        byte_index <= 0;
                        sub_state  <= SUB_IDLE;
                        tx_state   <= TX_SENSOR2;
                    end
                end
                
                TX_SENSOR2: begin
                    // Sub-FSM to transmit one byte from sensor2_data_reg.
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
                                    // Finished sensor2 transmission.
                                    sensor_data_latched <= 1'b0;  // Not latching new data.
                                    byte_index <= 0;
                                    sub_state  <= SUB_IDLE;
                                    tx_state   <= TX_DONE;         // Transition to final state.
                                end else begin
                                    byte_index <= byte_index + 1;
                                    sub_state  <= SUB_IDLE;
                                end
                            end
                        end
                    endcase
                end
                
                TX_DONE: begin
                    // Stay here (freeze). You could also drive a done flag if desired.
                    tx_state <= TX_DONE;
                end
                
                default: tx_state <= TX_IDLE;
            endcase
        end
    end
endmodule
