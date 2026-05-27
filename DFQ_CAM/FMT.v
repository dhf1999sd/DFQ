//////////////////////////////////////////////////////////////////////////////////
// Company:
// Engineer:
// Create Date:     2024/05/07 22:05:26
// Module Name:     FMT
// Project Name:
// Target Devices:
// Tool Versions:
// Description:     Parameterized flow mapping table with associative lookup
//////////////////////////////////////////////////////////////////////////////////

`timescale 1ns / 1ps

module FMT #(
    parameter integer NUM_ENTRY    = 100,
    parameter integer READ_DEPTH_THRESHOLD = 0,
    parameter integer ENTRY_ADDR_W = (NUM_ENTRY <= 1) ? 1 : $clog2(NUM_ENTRY)
  )(
    input                               i_clk,
    input                               i_rst,
    input                               i_init_req,
    output reg                          o_init_ack,
    input                               i_search_req,
    output reg                          o_search_hit,
    output reg                          o_search_miss,
    output reg  [19:0]                  o_match_tail,
    output reg  [ENTRY_ADDR_W-1:0]      o_match_addr,
    input       [19:0]                  i_refresh_tail,
    input                               i_pkt_done,
    input                               i_tail_update_req,
    input                               i_alloc_head_req,
    input       [19:0]                  i_alloc_head_data,
    input                               i_alloc_tail_req,
    input       [19:0]                  i_alloc_tail_data,
    input       [31:0]                  i_flow_id,
    output reg                          o_dequeue_req,
    output reg  [19:0]                  o_flow_head,
    input       [19:0]                  i_refresh_head,
    input                               i_refresh_head_vld
  );

  /***************function**************/
  function [ENTRY_ADDR_W-1:0] next_ptr;
    input [ENTRY_ADDR_W-1:0] ptr;
    begin
      if (ptr == NUM_ENTRY - 1)
        next_ptr = {ENTRY_ADDR_W{1'b0}};
      else
        next_ptr = ptr + 1'b1;
    end
  endfunction

  /***************parameter*************/
  localparam integer ENTRY_DEPTH_W = 4;

  /***************port******************/
  /***************mechine***************/
  /***************reg*******************/
  reg                             r_init_req_dly;
  reg                             r_entry_valid      [0:NUM_ENTRY-1];
  reg      [31:0]                 r_entry_flow_id    [0:NUM_ENTRY-1];
  reg      [19:0]                 r_entry_head       [0:NUM_ENTRY-1];
  reg      [19:0]                 r_entry_tail       [0:NUM_ENTRY-1];
  reg      [ENTRY_DEPTH_W-1:0]          r_entry_depth      [0:NUM_ENTRY-1];
  reg      [ENTRY_ADDR_W-1:0]     r_free_entry_fifo        [0:NUM_ENTRY-1];
  reg      [ENTRY_ADDR_W-1:0]     r_free_entry_rd_ptr;
  reg      [ENTRY_ADDR_W-1:0]     r_free_entry_wr_ptr;
  reg      [ENTRY_ADDR_W:0]       r_free_entry_count;
  reg      [ENTRY_ADDR_W-1:0]     r_pending_alloc_addr;
  reg                             r_pending_alloc_vld;
  reg      [ENTRY_ADDR_W-1:0]     r_read_entry_addr;
  reg                             r_search_hit_dly;
  reg                             r_search_miss_dly;
  reg      [19:0]                 r_match_tail_dly;
  reg      [ENTRY_ADDR_W-1:0]     r_match_addr_dly;
  integer                         i;
  integer                         j;
  integer                         r;

  /***************wire******************/
  reg                             w_search_hit;
  reg      [ENTRY_ADDR_W-1:0]     w_search_addr;
  reg      [19:0]                 w_search_tail;
  reg                             w_read_select_vld;
  reg      [ENTRY_ADDR_W-1:0]     w_read_select_addr;
  reg      [19:0]                 w_read_select_head;

  /***************component*************/
  /***************assign****************/
  /***************always****************/
  always @(*)
  begin
    w_search_hit  = 1'b0;
    w_search_addr = {ENTRY_ADDR_W{1'b0}};
    w_search_tail = 20'd0;

    for (j = 0; j < NUM_ENTRY; j = j + 1)
    begin
      if ((i_flow_id == r_entry_flow_id[j]) && (r_entry_depth[j] != {ENTRY_DEPTH_W{1'b0}}))
      begin
        w_search_hit  = 1'b1;
        w_search_addr = j[ENTRY_ADDR_W-1:0];
        w_search_tail = r_entry_tail[j];
      end
    end
  end

  always @(*)
  begin
    w_read_select_vld = 1'b0;
    w_read_select_addr  = {ENTRY_ADDR_W{1'b0}};
    w_read_select_head  = 20'd0;

    for (r = 0; r < NUM_ENTRY; r = r + 1)
    begin
      if (r_entry_depth[r] > READ_DEPTH_THRESHOLD)
      begin
        w_read_select_vld = 1'b1;
        w_read_select_addr  = r[ENTRY_ADDR_W-1:0];
        w_read_select_head  = r_entry_head[r];
      end
    end
  end

  always @(posedge i_clk)
  begin
    if (i_rst)
    begin
      r_init_req_dly          <= 1'b0;
      o_init_ack            <= 1'b0;
      r_pending_alloc_addr  <= {ENTRY_ADDR_W{1'b0}};
      r_pending_alloc_vld <= 1'b0;
      r_free_entry_rd_ptr         <= {ENTRY_ADDR_W{1'b0}};
      r_free_entry_wr_ptr         <= {ENTRY_ADDR_W{1'b0}};
      r_free_entry_count          <= NUM_ENTRY[ENTRY_ADDR_W:0];
      for (i = 0; i < NUM_ENTRY; i = i + 1)
      begin
        r_entry_valid[i]   <= 1'b0;
        r_entry_flow_id[i] <= 32'd0;
        r_entry_head[i]    <= 20'd0;
        r_entry_tail[i]    <= 20'd0;
        r_entry_depth[i]   <= {ENTRY_DEPTH_W{1'b0}};
        r_free_entry_fifo[i]     <= i[ENTRY_ADDR_W-1:0];
      end
    end
    else
    begin
      r_init_req_dly <= i_init_req;
      o_init_ack   <= 1'b0;

      if (i_init_req && !r_init_req_dly)
      begin
        o_init_ack            <= 1'b1;
        r_pending_alloc_addr  <= {ENTRY_ADDR_W{1'b0}};
        r_pending_alloc_vld <= 1'b0;
        r_free_entry_rd_ptr         <= {ENTRY_ADDR_W{1'b0}};
        r_free_entry_wr_ptr         <= {ENTRY_ADDR_W{1'b0}};
        r_free_entry_count          <= NUM_ENTRY[ENTRY_ADDR_W:0];
        for (i = 0; i < NUM_ENTRY; i = i + 1)
        begin
          r_entry_valid[i]   <= 1'b0;
          r_entry_flow_id[i] <= 32'd0;
          r_entry_head[i]    <= 20'd0;
          r_entry_tail[i]    <= 20'd0;
          r_entry_depth[i]   <= {ENTRY_DEPTH_W{1'b0}};
          r_free_entry_fifo[i]     <= i[ENTRY_ADDR_W-1:0];
        end
      end
      else if (i_alloc_head_req)
      begin
        if ((r_free_entry_count != 0) && !r_pending_alloc_vld)
        begin
          r_pending_alloc_addr                  <= r_free_entry_fifo[r_free_entry_rd_ptr];
          r_pending_alloc_vld                 <= 1'b1;
          r_entry_valid[r_free_entry_fifo[r_free_entry_rd_ptr]]   <= 1'b1;
          r_entry_flow_id[r_free_entry_fifo[r_free_entry_rd_ptr]] <= i_flow_id;
          r_entry_head[r_free_entry_fifo[r_free_entry_rd_ptr]]    <= i_alloc_head_data;
        end
      end
      else if (i_alloc_tail_req)
      begin
        if (r_pending_alloc_vld)
        begin
          r_entry_tail[r_pending_alloc_addr] <= i_alloc_tail_data;
          if (i_pkt_done)
            r_entry_depth[r_pending_alloc_addr] <= r_entry_depth[r_pending_alloc_addr] + 1'b1;
          r_free_entry_rd_ptr         <= next_ptr(r_free_entry_rd_ptr);
          r_free_entry_count          <= r_free_entry_count - 1'b1;
          r_pending_alloc_vld <= 1'b0;
        end
      end
      else if (i_tail_update_req)
      begin
        r_entry_tail[o_match_addr] <= i_refresh_tail;
        if (i_pkt_done)
          r_entry_depth[o_match_addr] <= r_entry_depth[o_match_addr] + 1'b1;
      end
      else if (i_refresh_head_vld)
      begin
        r_entry_head[r_read_entry_addr]  <= i_refresh_head;
        r_entry_depth[r_read_entry_addr] <= r_entry_depth[r_read_entry_addr] - 1'b1;
        if (r_entry_depth[r_read_entry_addr] - 1'b1 == {ENTRY_DEPTH_W{1'b0}})
        begin
          r_free_entry_fifo[r_free_entry_wr_ptr]         <= r_read_entry_addr;
          r_free_entry_wr_ptr                    <= next_ptr(r_free_entry_wr_ptr);
          r_entry_valid[r_read_entry_addr]     <= 1'b0;
          r_entry_flow_id[r_read_entry_addr]   <= 32'd0;
          r_entry_head[r_read_entry_addr]      <= 20'd0;
          r_entry_tail[r_read_entry_addr]      <= 20'd0;
          r_entry_depth[r_read_entry_addr]     <= {ENTRY_DEPTH_W{1'b0}};
          r_free_entry_count                     <= r_free_entry_count + 1'b1;
        end
      end
    end
  end

  always @(posedge i_clk)
  begin
    if (i_rst)
    begin
      o_search_hit        <= 1'b0;
      o_search_miss     <= 1'b0;
      o_match_tail     <= 20'd0;
      o_match_addr     <= {ENTRY_ADDR_W{1'b0}};
      r_search_hit_dly    <= 1'b0;
      r_search_miss_dly <= 1'b0;
      r_match_tail_dly <= 20'd0;
      r_match_addr_dly <= {ENTRY_ADDR_W{1'b0}};
    end
    else
    begin
      o_search_hit    <= r_search_hit_dly;
      o_search_miss <= r_search_miss_dly;
      o_match_tail <= r_match_tail_dly;
      o_match_addr <= r_match_addr_dly;

      if (i_search_req)
      begin
        r_search_hit_dly    <= w_search_hit;
        r_search_miss_dly <= ~w_search_hit;
        r_match_tail_dly <= w_search_tail;
        r_match_addr_dly <= w_search_addr;
      end
      else if (i_tail_update_req || i_alloc_tail_req)
      begin
        r_search_hit_dly    <= 1'b0;
        r_search_miss_dly <= 1'b0;
      end
    end
  end

  always @(posedge i_clk)
  begin
    if (i_rst)
    begin
      o_dequeue_req      <= 1'b0;
      o_flow_head <= 20'd0;
      r_read_entry_addr <= {ENTRY_ADDR_W{1'b0}};
    end
    else
    begin
      if (i_refresh_head_vld == 1'b1)
      begin
        o_dequeue_req <= 1'b0;
      end
      else if (!o_dequeue_req && w_read_select_vld)
      begin
        o_dequeue_req      <= 1'b1;
        o_flow_head <= w_read_select_head;
        r_read_entry_addr <= w_read_select_addr;
      end
    end
  end

endmodule
