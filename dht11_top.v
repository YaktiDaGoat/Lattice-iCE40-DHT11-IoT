module dht11_top(
    input  wire clk,         // 12 MHz clock
    input  wire reset_n,     // Active-low reset
    inout  wire dht_data1,   // Sensor 1 DHT11 data pin
    inout  wire dht_data2,   // Sensor 2 DHT11 data pin
    inout  wire dht_data3,   // Sensor 3 DHT11 data pin
    output wire uart_tx,     // UART transmit line
    output [2:0] debug_state // 3-bit debug state output from TX FSM
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
    // Instantiate sensor interface for sensor 3
    // -----------------------------------------------------
    wire data_valid3;
    wire [31:0] sensor_data3;
    dht11_interface sensor3 (
        .clk(clk),
        .reset_n(reset_n),
        .dht_data(dht_data3),
        .data_valid(data_valid3),
        .sensor_data(sensor_data3)
    );
    
    // -----------------------------------------------------
    // UART transmitter instantiation (configured for 9600 baud)
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
    // 8 states coded in 3 bits:
    //     TX_IDLE    = 3'd0 : Wait for all 3 sensors to be valid.
    //     TX_PRE     = 3'd1 : Wait 10 clock cycles before starting TX.
    //     TX_SENSOR1 = 3'd2 : Transmit sensor1’s 4 bytes.
    //     TX_DELAY1  = 3'd3 : Wait 10 cycles after sensor1.
    //     TX_SENSOR2 = 3'd4 : Transmit sensor2’s 4 bytes.
    //     TX_DELAY2  = 3'd5 : Wait 10 cycles after sensor2.
    //     TX_SENSOR3 = 3'd6 : Transmit sensor3’s 4 bytes.
    //     TX_DONE    = 3'd7 : Final state (freeze).
    // -----------------------------------------------------
    localparam TX_IDLE    = 3'd0,
               TX_PRE     = 3'd1,
               TX_SENSOR1 = 3'd2,
               TX_DELAY1  = 3'd3,
               TX_SENSOR2 = 3'd4,
               TX_DELAY2  = 3'd5,
               TX_SENSOR3 = 3'd6,
               TX_DONE    = 3'd7;
               
    reg [2:0] tx_state;
    assign debug_state = tx_state;  // For external debugging: 0..7 indicates current state.
    
    // -----------------------------------------------------
    // Sub-FSM for transmitting one byte via uart_tx.
    // Generates a one-clock send pulse and waits for uart_busy (rising then falling edge).
    // -----------------------------------------------------
    localparam SUB_IDLE      = 2'd0,
               SUB_TRIGGER   = 2'd1,
               SUB_WAIT_DONE = 2'd2;
               
    reg [1:0] sub_state;   // Sub-FSM state for byte transmission.
    reg [1:0] byte_index;  // Byte index (0 to 3) for a 32-bit word.
    
    // -----------------------------------------------------
    // Latch sensor data once all signals are valid.
    // -----------------------------------------------------
    reg [31:0] sensor1_data_reg, sensor2_data_reg, sensor3_data_reg;
    reg sensor_data_latched;
    
    // -----------------------------------------------------
    // Delay counters for the pre-delay and inter-sensor delays.
    // Each delay is 10 clock cycles.
    // -----------------------------------------------------
    reg [3:0] pre_delay_cnt;  // for TX_PRE state
    reg [3:0] delay_cnt;      // for TX_DELAY states
    
    // -----------------------------------------------------
    // For edge detection: capture previous uart_busy.
    // -----------------------------------------------------
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
            sensor3_data_reg    <= 32'd0;
            pre_delay_cnt       <= 0;
            delay_cnt           <= 0;
            data_byte           <= 8'd0;
            prev_uart_busy      <= 1'b0;
        end else begin
            // Edge detection for the UART busy signal.
            prev_uart_busy <= uart_busy;
            
            case (tx_state)
            
                TX_IDLE: begin
                    // Latch data once all three sensors report valid data.
                    if (data_valid1 && data_valid2 && data_valid3 && !sensor_data_latched) begin
                        sensor1_data_reg    <= sensor_data1;
                        sensor2_data_reg    <= sensor_data2;
                        sensor3_data_reg    <= sensor_data3;
                        sensor_data_latched <= 1'b1;
                        byte_index          <= 0;
                        sub_state           <= SUB_IDLE;
                        pre_delay_cnt       <= 0;
                        tx_state            <= TX_PRE;
                    end
                end
                
                TX_PRE: begin
                    // Wait 10 clock cycles before starting to transmit sensor1's data.
                    if (pre_delay_cnt < 10 - 1) begin
                        pre_delay_cnt <= pre_delay_cnt + 1;
                    end else begin
                        pre_delay_cnt <= 0;
                        tx_state      <= TX_SENSOR1;
                    end
                end
                
                TX_SENSOR1: begin
                    // Transmit sensor1's 4 bytes using sub-FSM.
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
                                send       <= 1'b1;
                                sub_state  <= SUB_TRIGGER;
                            end
                        end
                        
                        SUB_TRIGGER: begin
                            send       <= 1'b0;
                            sub_state  <= SUB_WAIT_DONE;
                        end
                        
                        SUB_WAIT_DONE: begin
                            if (prev_uart_busy && !uart_busy) begin
                                if (byte_index == 3) begin
                                    // Finished sensor1; reset index, clear sub FSM and delay counter.
                                    byte_index <= 0;
                                    delay_cnt  <= 0;
                                    sub_state  <= SUB_IDLE;
                                    tx_state   <= TX_DELAY1;
                                end else begin
                                    byte_index <= byte_index + 1;
                                    sub_state  <= SUB_IDLE;
                                end
                            end
                        end
                    endcase
                end
                
                TX_DELAY1: begin
                    // Wait 10 clock cycles between sensor1 and sensor2 transmissions.
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
                    // Transmit sensor2's 4 bytes.
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
                                send       <= 1'b1;
                                sub_state  <= SUB_TRIGGER;
                            end
                        end
                        
                        SUB_TRIGGER: begin
                            send       <= 1'b0;
                            sub_state  <= SUB_WAIT_DONE;
                        end
                        
                        SUB_WAIT_DONE: begin
                            if (prev_uart_busy && !uart_busy) begin
                                if (byte_index == 3) begin
                                    byte_index <= 0;
                                    delay_cnt  <= 0;
                                    sub_state  <= SUB_IDLE;
                                    tx_state   <= TX_DELAY2;
                                end else begin
                                    byte_index <= byte_index + 1;
                                    sub_state  <= SUB_IDLE;
                                end
                            end
                        end
                    endcase
                end
                
                TX_DELAY2: begin
                    // Wait 10 clock cycles between sensor2 and sensor3 transmissions.
                    if (delay_cnt < 10 - 1) begin
                        delay_cnt <= delay_cnt + 1;
                    end else begin
                        delay_cnt <= 0;
                        byte_index <= 0;
                        sub_state  <= SUB_IDLE;
                        tx_state   <= TX_SENSOR3;
                    end
                end
                
                TX_SENSOR3: begin
                    // Transmit sensor3's 4 bytes.
                    case (sub_state)
                        SUB_IDLE: begin
                            if (!uart_busy) begin
                                case (byte_index)
                                    2'd0: data_byte <= sensor3_data_reg[31:24];
                                    2'd1: data_byte <= sensor3_data_reg[23:16];
                                    2'd2: data_byte <= sensor3_data_reg[15:8];
                                    2'd3: data_byte <= sensor3_data_reg[7:0];
                                    default: data_byte <= 8'd0;
                                endcase
                                send       <= 1'b1;
                                sub_state  <= SUB_TRIGGER;
                            end
                        end
                        
                        SUB_TRIGGER: begin
                            send       <= 1'b0;
                            sub_state  <= SUB_WAIT_DONE;
                        end
                        
                        SUB_WAIT_DONE: begin
                            if (prev_uart_busy && !uart_busy) begin
                                if (byte_index == 3) begin
                                    // Finished sensor3; allow new data to be latched.
                                    sensor_data_latched <= 1'b0;
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
                    // Freeze in TX_DONE (final state).
                    tx_state <= TX_DONE;
                end
                
                default: tx_state <= TX_IDLE;
            endcase
        end
    end
endmodule
