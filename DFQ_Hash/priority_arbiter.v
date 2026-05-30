//////////////////////////////////////////////////////////////////////////////////
// Company:         NNS@TSN
// Engineer:        Wenxue Wu
// Create Date:     2023/11/12
// Module Name:     priority_arbiter
// Project Name:    priority_arbiter
// Target Devices:  Zynq
// Tool Versions:   VIVADO2023.2
// Description:     
//////////////////////////////////////////////////////////////////////////////////

`timescale 1ns / 1ps

module priority_arbiter #(
    parameter       P_CHANNEL_NUM = 3
)(
    input                       i_clk         ,
    input                       i_rst         ,
    input                       i_req_release ,
    input   [P_CHANNEL_NUM-1:0] i_req_in      ,
    output  reg [P_CHANNEL_NUM-1:0] o_grant_out
);

/***************function**************/
/***************parameter*************/
/***************port******************/
/***************mechine***************/
/***************reg*******************/
  reg r_req_release_dly;

/***************wire******************/
/***************component*************/
/***************assign****************/
/***************always****************/
  always @(posedge i_clk)
  begin
    if (i_rst)
      r_req_release_dly <= 1'b0;
    else
      r_req_release_dly <= i_req_release;
  end

  always @(posedge i_clk)
  begin
    if (i_rst)
      o_grant_out <= {P_CHANNEL_NUM{1'b0}};
    else if (r_req_release_dly)
      o_grant_out <= i_req_in & ((~i_req_in) + 1'b1);
    else
      o_grant_out <= o_grant_out;
  end

endmodule
