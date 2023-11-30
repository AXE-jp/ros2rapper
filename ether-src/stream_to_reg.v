
`include "config.vh"

`resetall
`default_nettype none


module  stream_to_reg # (
	parameter DATA_BYTES = 16
)
(
	input wire	clk,
	input wire	rst_n,

	input wire [DATA_BYTES*8-1:0]		i_data,
	input wire				i_tvalid,
	output wire				i_tready,
	input wire [7:0]			mem_len,

	output wire [DATA_BYTES*8-1:0]		o_data,
	output wire [7:0]			o_data_len
);

(*mark_debug="true"*) reg [DATA_BYTES*8-1:0] data_reg[1:0];
(*mark_debug="true"*) reg [7:0] data_len_reg[1:0];
(*mark_debug="true"*) reg toggle_reg = 1'b0;

reg tready_reg = 1'b1;
assign i_tready = tready_reg;

assign o_data = data_reg[toggle_reg];
assign o_data_len = data_len_reg[toggle_reg];

integer ii;
integer sz = DATA_BYTES * 8;

always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        for(ii=0; ii < sz ; ii=ii+1) begin
            data_reg[0][ii] <= 8'b0;
            data_reg[1][ii] <= 8'b0;
        end
    end else begin
	if (i_tvalid) begin
            data_reg[~toggle_reg] <= i_data;
            data_len_reg[~toggle_reg] <= mem_len;
            toggle_reg <= ~toggle_reg;
        end
    end
end


endmodule
