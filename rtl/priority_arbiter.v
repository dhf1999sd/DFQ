`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company:         NNS@TSN
// Engineer:        Wenxue Wu
// Create Date:     2023/11/12
// Module Name:     priority_arbiter
// Project Name:    Dynamic per flow queues
// Target Devices:  Zynq
// Tool Versions:   VIVADO 2023.2
// Description:     Priority arbiter using lowest-set-bit grant strategy
//////////////////////////////////////////////////////////////////////////////////

module priority_arbiter #(
    parameter P_CHANEL_NUM = 3
  ) (
    input                         clk,
    input                         reset,
    input                         i_req_release,  // only one is valid
    input      [P_CHANEL_NUM-1:0] i_req_in,
    output reg [P_CHANEL_NUM-1:0] o_grant_out
  );

  reg ri_req_release;

  always @(posedge clk)
  begin
    if (reset)
      ri_req_release <= 0;
    else
      ri_req_release <= i_req_release;
  end

  always @(posedge clk)
  begin
    if (reset)
      o_grant_out <= 0;
    else if (ri_req_release)
      o_grant_out <= i_req_in & ((~i_req_in) + 1);
    else
      o_grant_out <= o_grant_out;
  end

endmodule

