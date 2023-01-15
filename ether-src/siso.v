`resetall
`timescale 1ns / 1ps
`default_nettype none

module siso #
(
    parameter DATA_WIDTH = 8,
    parameter DEPTH = 64,
    parameter REGS_WIDTH = (DATA_WIDTH*DEPTH)
)
(
    input  wire                   clk,
    input  wire                   rst_n,

    input  wire                   wr_en,
    input  wire [DATA_WIDTH-1:0]  din,
    output wire                   full,

    input  wire                   rd_en,
    output wire [DATA_WIDTH-1:0]  dout,
    output wire                   empty
);

parameter PTR_WIDTH = $clog2(DEPTH);

reg [DATA_WIDTH-1:0] regs[0:DEPTH-1];
reg [PTR_WIDTH-1:0] wp = 0;
reg [PTR_WIDTH-1:0] rp = 0;

wire [PTR_WIDTH-1:0] wp_next = wp + 1;
assign full = (wp_next == rp ? 1'b1 : 1'b0);
assign empty = (wp == rp ? 1'b1 : 1'b0);

integer i;
always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        wp <= 0;
        rp <= 0;
    end else begin
        if (wr_en) begin
          if (~full) begin
            regs[wp] <= din;
            wp <= wp + 1;
          end
        end
        if (rd_en) begin
          if (~empty) begin
            rp <= rp + 1;
          end
        end
    end
end

assign dout = regs[rp];

endmodule

`resetall
