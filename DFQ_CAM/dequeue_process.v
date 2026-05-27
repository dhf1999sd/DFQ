//////////////////////////////////////////////////////////////////////////////////
// Company:
// Engineer:
// Create Date:     2024/05/15 16:43:21
// Module Name:     dequeue_process
// Project Name:
// Target Devices:
// Tool Versions:
// Description:
//////////////////////////////////////////////////////////////////////////////////

`timescale 1ns / 1ps

module dequeue_process(
    input               i_clk              ,
    input               i_rst              ,
    output reg          o_read_busy        ,
    input               i_dequeue_req      ,
    input               i_write_busy       ,
    input      [19:0]   i_flow_head        ,
    output reg [19:0]   o_prio_queue_din   ,
    input      [19:0]   i_ptr_ram_dout     ,
    output reg          o_prio_queue_wr    ,
    output reg [9:0]    o_ptr_ram_rd_addr  ,
    output reg [19:0]   o_refresh_head     ,
    output reg          o_refresh_head_vld
);

/***************function**************/
/***************parameter*************/
  localparam S_IDLE         = 4'd0 ;
  localparam S_START        = 4'd1 ;
  localparam S_CHECK        = 4'd2 ;
  localparam S_NEXT         = 4'd3 ;
  localparam S_PUSH         = 4'd4 ;
  localparam S_WAIT         = 4'd5 ;
  localparam S_PUSH_LOOP    = 4'd6 ;
  localparam S_PUSH_DONE    = 4'd7 ;
  localparam S_REFRESH      = 4'd8 ;
  localparam S_REFRESH_DONE = 4'd9 ;
  localparam S_EXIT         = 4'd10;
  localparam S_READ         = 4'd11;
  localparam S_EXIT2        = 4'd12;

/***************port******************/
/***************mechine***************/
/***************reg*******************/
  reg [3:0]  r_dequeue_state;
  reg [19:0] r_curr_cell;

/***************wire******************/
/***************component*************/
/***************assign****************/
/***************always****************/
  always @(posedge i_clk)
    if (i_rst)
    begin
      r_dequeue_state   <= S_IDLE;
      o_prio_queue_wr   <= 1'b0;
      o_ptr_ram_rd_addr <= 10'd0;
      o_prio_queue_din  <= 20'd0;
      o_refresh_head_vld <= 1'b0;
      o_refresh_head    <= 20'd0;
      o_read_busy       <= 1'b0;
      r_curr_cell       <= 20'd0;
    end
    else
    begin
      o_prio_queue_wr <= 1'b0;
      case (r_dequeue_state)
        S_IDLE:
        begin
          if (i_dequeue_req & !i_write_busy)
          begin
            r_dequeue_state <= S_START;
            o_read_busy     <= 1'b1;
          end
        end
        S_START:
        begin
          // Output the current head first; it is the first cell of the packet.
          r_curr_cell       <= i_flow_head[19:0];
          o_prio_queue_din  <= i_flow_head[19:0];
          o_prio_queue_wr   <= 1'b1;
          o_ptr_ram_rd_addr <= i_flow_head[9:0];
          r_dequeue_state   <= S_CHECK;
        end
        S_CHECK:
        begin
          if ((r_curr_cell[15] && r_curr_cell[14]))
            r_dequeue_state <= S_EXIT2;
          else
            r_dequeue_state <= S_READ;
        end
        S_NEXT:
        begin
          r_dequeue_state   <= S_WAIT;
          o_refresh_head    <= i_ptr_ram_dout[19:0];
          o_refresh_head_vld <= 1'b1;
        end
        S_PUSH:
        begin
          r_dequeue_state   <= S_PUSH_LOOP;
          r_curr_cell       <= i_ptr_ram_dout[19:0];
          o_prio_queue_din  <= i_ptr_ram_dout[19:0];
          o_prio_queue_wr   <= 1'b1;
          o_ptr_ram_rd_addr <= i_ptr_ram_dout[9:0];
        end
        S_WAIT:
        begin
          r_dequeue_state    <= S_IDLE;
          o_read_busy        <= 1'b0;
          o_refresh_head_vld <= 1'b0;
          o_refresh_head     <= 20'd0;
        end
        S_PUSH_LOOP:
          r_dequeue_state <= S_PUSH_DONE;
        S_PUSH_DONE:
          r_dequeue_state <= S_REFRESH;
        S_REFRESH:
          r_dequeue_state <= S_REFRESH_DONE;
        S_REFRESH_DONE:
        begin
          if (r_curr_cell[15])
          begin
            o_refresh_head     <= i_ptr_ram_dout[19:0];
            o_refresh_head_vld <= 1'b1;
            r_dequeue_state    <= S_EXIT;
          end
          else
            r_dequeue_state <= S_PUSH;
        end
        S_EXIT:
        begin
          r_dequeue_state    <= S_IDLE;
          o_read_busy        <= 1'b0;
          o_refresh_head_vld <= 1'b0;
          o_refresh_head     <= 20'd0;
        end
        S_READ:
          r_dequeue_state <= S_PUSH;
        S_EXIT2:
          r_dequeue_state <= S_NEXT;
        default:
          r_dequeue_state <= S_IDLE;
      endcase
    end
endmodule
