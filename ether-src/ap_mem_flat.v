
`include "config.vh"

`resetall
`default_nettype none


module  ap_mem_flat # (
	parameter DATA_BYTES = 16
)
(
	input wire	clk,
	input wire	rst_n,

	input wire [7:0]			mem_data,
	input wire [$clog2(DATA_BYTES)-1:0]	mem_addr,
	input wire				mem_ce,
	input wire				mem_we,
	input wire [7:0]			mem_len,

	output wire [DATA_BYTES*8-1:0]		o_data,
	output wire [7:0]			o_data_len
);

reg [DATA_BYTES*8-1:0] data_reg[1:0];
reg [7:0] data_len_reg[1:0];
(*mark_debug="true"*) reg toggle_reg = 1'b0;

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
        if (mem_ce && mem_we) begin
            for(ii=0; ii < 8 ; ii=ii+1) begin
                data_reg[~toggle_reg][mem_addr*8+ii] <= mem_data[ii];
            end
            // the last byte comes.
            if (mem_addr == (mem_len - 1)) begin
                data_len_reg[~toggle_reg] <= mem_len;
                toggle_reg <= ~toggle_reg;
            end
        end
    end
end


endmodule
