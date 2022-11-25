`default_nettype none

module RAM_WRAP#(parameter AWIDTH=32, DWIDTH=32, OPT=0)(CLK, CEB_N, WEB_N, ADDR, DIN, DOUT);
    input wire CLK, CEB_N, WEB_N;
    input wire [AWIDTH-1:0] ADDR;
    input wire [DWIDTH-1:0] DIN;
    output wire [DWIDTH-1:0] DOUT;

//    initial DOUT = 0;

`ifdef TARGET_ASIC
generate
  if(DWIDTH==28)
    sram_28_512_sky130A RAM(
        .clk0  (CLK),
        .csb0  (CEB_N),
        .web0  (WEB_N),
        .addr0 (ADDR),
        .din0  (DIN),
        .dout0 (DOUT)
    );
  else if(DWIDTH==2)
//    RAMB2W8192RW RAM(
    RAMB2X4W2048RW RAM(
        .clk0  (CLK),
        .csb0  (CEB_N),
        .web0  (WEB_N),
        .addr0 (ADDR),
        .din0  (DIN),
        .dout0 (DOUT)
    );
  else if(DWIDTH==16)
    sram_16_256_sky130A RAM(
        .clk0  (CLK),
        .csb0  (CEB_N),
        .web0  (WEB_N),
        .addr0 (ADDR),
        .din0  (DIN),
        .dout0 (DOUT)
    );
endgenerate
`else
    reg [DWIDTH-1:0] DOUT_tmp;
    (* ram_style = "block" *) reg [DWIDTH-1:0] mem[0:2**AWIDTH-1];
    always @(posedge CLK) begin
        if (!CEB_N) begin
            if (!WEB_N) begin
                mem[ADDR] <= DIN;
            end else
                DOUT_tmp <= mem[ADDR];
        end
    end
    assign DOUT = DOUT_tmp;
`endif
endmodule

`default_nettype wire
