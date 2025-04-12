`default_nettype none

module bram_16bit (
    input  wire clk,
    input  wire we,
    input  wire addr,         // With only one location we use a single‐bit address.
    input  wire [15:0] din,
    output reg  [15:0] dout
);
    // Infer a block RAM using the "ram_style" attribute
    (* ram_style = "block" *) reg [15:0] mem [0:0];

    always @(posedge clk) begin
        if (we)
            mem[addr] <= din;
        dout <= mem[addr];
    end
endmodule
