module dht11_top (
    input  wire clk,
    input  wire reset_n,
    inout  wire dht_data0, // DHT11 sensor 0
    inout  wire dht_data1, // DHT11 sensor 1
    inout  wire dht_data2, // DHT11 sensor 2
    inout  wire dht_data3,
    inout  wire dht_data4,
    inout  wire dht_data5,
    output wire uart_tx
);

    // ---------------------------------------------------------
    // Sensor interface signals
    // ---------------------------------------------------------
    wire sensor0_done, sensor1_done, sensor2_done, sensor3_done, sensor4_done, sensor5_done;
    wire [31:0] sensor0_data, sensor1_data, sensor2_data, sensor3_data, sensor4_data, sensor5_data;

    // Instantiate three sensor modules
    dht11_min_handshake0 sensor0 (
        .clk(clk),
        .reset_n(reset_n),
        .dht_data(dht_data0),
        .done(sensor0_done),
        .raw_data(sensor0_data)
    );

    dht11_min_handshake1 sensor1 (
        .clk(clk),
        .reset_n(reset_n),
        .dht_data(dht_data1),
        .done(sensor1_done),
        .raw_data(sensor1_data)
    );

    dht11_min_handshake2 sensor2 (
        .clk(clk),
        .reset_n(reset_n),
        .dht_data(dht_data2),
        .done(sensor2_done),
        .raw_data(sensor2_data)
    );

    dht11_min_handshake3 sensor3 (
        .clk(clk),
        .reset_n(reset_n),
        .dht_data(dht_data3),
        .done(sensor3_done),
        .raw_data(sensor3_data)
    );

    dht11_min_handshake4 sensor4 (
        .clk(clk),
        .reset_n(reset_n),
        .dht_data(dht_data4),
        .done(sensor4_done),
        .raw_data(sensor4_data)
    );

    dht11_min_handshake5 sensor5 (
        .clk(clk),
        .reset_n(reset_n),
        .dht_data(dht_data5),
        .done(sensor5_done),
        .raw_data(sensor5_data)
    );

    // ---------------------------------------------------------
    // Aggregator FSM: collects sensor data and transmits over UART.
    // Packet format: Sensor0 (4 bytes), Sensor1 (4 bytes), Sensor2 (4 bytes) = 12 bytes total.
    // ---------------------------------------------------------
    // Declare aggregator FSM states.
    localparam AGG_WAIT       = 3'd0;  // Wait for all sensor "done"
    localparam AGG_LOAD       = 3'd1;  // Load sensor data into packet memory
    localparam AGG_UART_LOAD  = 3'd2;  // Load a packet byte into UART interface
    localparam AGG_UART_SEND  = 3'd3;  // Assert send to UART
    localparam AGG_UART_WAIT  = 3'd4;  // Wait for UART transmitter to finish sending current byte
    localparam AGG_DONE       = 3'd5;  // Packet transmitted; go back to wait

    reg [2:0] agg_state;
    // We'll use a 4-bit index to count through our 12-byte packet.
    reg [3:0] packet_index;
    // Define a packet memory for 12 bytes.
    reg [7:0] packet [0:11];

    // Registers for UART interface
    reg [7:0] uart_data_reg;
    reg       uart_send_reg;

    // Wires connecting to UART transmitter.
    wire uart_busy;

    // Instantiate UART transmitter.
    uart_tx #(
        .CLK_FREQ(12000000),
        .BAUD_RATE(115200)
    ) uart_transmitter (
        .clk(clk),
        .reset_n(reset_n),
        .data(uart_data_reg),
        .send(uart_send_reg),
        .tx(uart_tx),
        .busy(uart_busy)
    );

    // Aggregator FSM
    always @(posedge clk or negedge reset_n) begin
        if (!reset_n) begin
            agg_state      <= AGG_WAIT;
            packet_index   <= 0;
            uart_send_reg  <= 1'b0;
            // Optionally clear the packet memory.
        end else begin
            case (agg_state)
                AGG_WAIT: begin
                    // Wait until all three sensor modules have finished reading.
                    if (sensor0_done && sensor1_done && sensor2_done) begin
                        agg_state <= AGG_LOAD;
                    end
                end

                AGG_LOAD: begin
                    // Load sensor data into the packet.
                    // Sensor0: bytes 0-3
                    packet[0]  <= sensor0_data[31:24];
                    packet[1]  <= sensor0_data[23:16];
                    packet[2]  <= sensor0_data[15:8];
                    packet[3]  <= sensor0_data[7:0];
                    // Sensor1: bytes 4-7
                    packet[4]  <= sensor1_data[31:24];
                    packet[5]  <= sensor1_data[23:16];
                    packet[6]  <= sensor1_data[15:8];
                    packet[7]  <= sensor1_data[7:0];
                    // Sensor2: bytes 8-11
                    packet[8]  <= sensor2_data[31:24];
                    packet[9]  <= sensor2_data[23:16];
                    packet[10] <= sensor2_data[15:8];
                    packet[11] <= sensor2_data[7:0];

                    packet_index <= 0;
                    agg_state <= AGG_UART_LOAD;
                end

                AGG_UART_LOAD: begin
                    // Load the current packet byte into the UART data register.
                    uart_data_reg <= packet[packet_index];
                    agg_state <= AGG_UART_SEND;
                end

                AGG_UART_SEND: begin
                    // Trigger the UART transmitter.
                    if (!uart_busy && !uart_send_reg) begin
                        uart_send_reg <= 1'b1;
                        agg_state <= AGG_UART_WAIT;
                    end
                end

                AGG_UART_WAIT: begin
                    // Deassert uart_send_reg and wait for UART to finish the current byte.
                    uart_send_reg <= 1'b0;
                    if (!uart_busy) begin
                        if (packet_index == 11) begin
                            agg_state <= AGG_DONE;
                        end else begin
                            packet_index <= packet_index + 1;
                            agg_state <= AGG_UART_LOAD;
                        end
                    end
                end

                AGG_DONE: begin
                    // Optionally, reset sensor modules or simply return to waiting state
                    // for the next round of sensor measurements.
                    agg_state <= AGG_WAIT;
                end

                default: agg_state <= AGG_WAIT;
            endcase
        end
    end

endmodule