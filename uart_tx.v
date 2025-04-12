module uart_tx #(parameter CLK_FREQ = 12000000, parameter BAUD_RATE = 115200)
(
    input  wire clk,
    input  wire reset_n,
    input  wire [7:0] data,
    input  wire send,
    output reg  tx,
    output reg  busy
);
    localparam integer BIT_PERIOD = CLK_FREQ / BAUD_RATE;  // Approx. 104 cycles for 12MHz/115200.
    reg [7:0] shift_reg;
    reg [6:0] baud_counter;
    reg [3:0] bit_index;
    reg transmitting;

    always @(posedge clk or negedge reset_n) begin
        if (!reset_n) begin
            tx <= 1'b1;
            busy <= 1'b0;
            transmitting <= 1'b0;
            baud_counter <= 0;
            bit_index <= 0;
        end else begin
            if (send && !busy) begin
                busy <= 1'b1;
                transmitting <= 1'b1;
                shift_reg <= data;
                bit_index <= 0;
                baud_counter <= 0;
                tx <= 1'b0;  // Start bit (low)
            end else if (transmitting) begin
                if (baud_counter < BIT_PERIOD - 1) begin
                    baud_counter <= baud_counter + 1;
                end else begin
                    baud_counter <= 0;
                    if (bit_index < 8) begin
                        tx <= shift_reg[0];
                        shift_reg <= {1'b0, shift_reg[7:1]};
                        bit_index <= bit_index + 1;
                    end else if (bit_index == 8) begin
                        tx <= 1'b1;  // Stop bit
                        bit_index <= bit_index + 1;
                    end else begin
                        transmitting <= 1'b0;
                        busy <= 1'b0;
                        tx <= 1'b1;
                    end
                end
            end
        end
    end
endmodule