//////////////////////////////////////////////////////////////////////////////////
// Company:
// Engineer:
// Create Date:     2024/05/07 22:05:26
// Module Name:     FMT
// Project Name:
// Target Devices:
// Tool Versions:
// Description:     Flow mapping table with RAM-backed bucket and entry storage
//////////////////////////////////////////////////////////////////////////////////

`timescale 1ns / 1ps

module FMT #(
    parameter integer NUM_ENTRY    = 100,
    parameter integer READ_DEPTH_THRESHOLD = 4,
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

  function integer next_pow2;
    input integer value;
    integer pow2;
    begin
      pow2 = 1;
      while (pow2 < value)
        pow2 = pow2 << 1;
      next_pow2 = pow2;
    end
  endfunction

  /***************parameter*************/
  localparam integer ENTRY_DEPTH_W     = 11;
  localparam integer HASH_BUCKET_DEPTH = 8;
  localparam integer HASH_BUCKET_NUM   = next_pow2(NUM_ENTRY);
  localparam integer HASH_BUCKET_W     = (HASH_BUCKET_NUM <= 1) ? 1 : $clog2(HASH_BUCKET_NUM);
  localparam integer HASH_SLOT_W       = (HASH_BUCKET_DEPTH <= 1) ? 1 : $clog2(HASH_BUCKET_DEPTH);
  localparam integer BUCKET_SLOT_W     = ENTRY_ADDR_W + 1;

  localparam integer ENTRY_VALID_LSB       = 0;
  localparam integer ENTRY_DEPTH_LSB       = ENTRY_VALID_LSB + 1;
  localparam integer ENTRY_HEAD_LSB        = ENTRY_DEPTH_LSB + ENTRY_DEPTH_W;
  localparam integer ENTRY_TAIL_LSB        = ENTRY_HEAD_LSB + 20;
  localparam integer ENTRY_FLOW_LSB        = ENTRY_TAIL_LSB + 20;
  localparam integer ENTRY_BUCKET_IDX_LSB  = ENTRY_FLOW_LSB + 32;
  localparam integer ENTRY_BUCKET_SLOT_LSB = ENTRY_BUCKET_IDX_LSB + HASH_BUCKET_W;
  localparam integer ENTRY_WORD_W          = ENTRY_BUCKET_SLOT_LSB + HASH_SLOT_W;

  localparam [3:0] SEARCH_IDLE        = 4'd0;
  localparam [3:0] SEARCH_HASH_MUL0   = 4'd1;
  localparam [3:0] SEARCH_HASH_XOR1   = 4'd2;
  localparam [3:0] SEARCH_HASH_MUL1   = 4'd3;
  localparam [3:0] SEARCH_HASH_DONE   = 4'd4;
  localparam [3:0] SEARCH_WAIT_BUCKET = 4'd5;
  localparam [3:0] SEARCH_PRI_SLOT    = 4'd6;
  localparam [3:0] SEARCH_PRI_ENTRY   = 4'd7;
  localparam [3:0] SEARCH_SEC_SLOT    = 4'd8;
  localparam [3:0] SEARCH_SEC_ENTRY   = 4'd9;
  localparam [3:0] SEARCH_PRI_WAIT    = 4'd10;
  localparam [3:0] SEARCH_SEC_WAIT    = 4'd11;

  localparam [31:0] MIX32_MUL0 = 32'h7feb352d;
  localparam [31:0] MIX32_MUL1 = 32'h846ca68b;

  function [31:0] mix32;
    input [31:0] x;
    reg   [31:0] y;
    begin
      y = x;
      y = y ^ (y >> 16);
      y = y * MIX32_MUL0;
      y = y ^ (y >> 15);
      y = y * MIX32_MUL1;
      y = y ^ (y >> 16);
      mix32 = y;
    end
  endfunction

  function [HASH_BUCKET_W-1:0] primary_bucket_idx;
    input [31:0] flow_id;
    reg   [31:0] h;
    begin
      h = mix32(flow_id);
      primary_bucket_idx = h[HASH_BUCKET_W-1:0];
    end
  endfunction

  function [HASH_BUCKET_W-1:0] secondary_bucket_idx;
    input [31:0] flow_id;
    reg [HASH_BUCKET_W-1:0] pri;
    reg [HASH_BUCKET_W-1:0] sec;
    reg [31:0]              h;
    begin
      h = mix32(flow_id);
      pri = h[HASH_BUCKET_W-1:0];
      sec = h[31 -: HASH_BUCKET_W] ^ pri;
      if (sec == pri)
        secondary_bucket_idx = pri + {{(HASH_BUCKET_W-1){1'b0}}, 1'b1};
      else
        secondary_bucket_idx = sec;
    end
  endfunction

  function [ENTRY_WORD_W-1:0] pack_entry;
    input                         valid;
    input [31:0]                  flow_id;
    input [19:0]                  head;
    input [19:0]                  tail;
    input [ENTRY_DEPTH_W-1:0]     depth;
    input [HASH_BUCKET_W-1:0]     bucket_idx;
    input [HASH_SLOT_W-1:0]       bucket_slot;
    begin
      pack_entry = {ENTRY_WORD_W{1'b0}};
      pack_entry[ENTRY_VALID_LSB] = valid;
      pack_entry[ENTRY_DEPTH_LSB +: ENTRY_DEPTH_W] = depth;
      pack_entry[ENTRY_HEAD_LSB +: 20] = head;
      pack_entry[ENTRY_TAIL_LSB +: 20] = tail;
      pack_entry[ENTRY_FLOW_LSB +: 32] = flow_id;
      pack_entry[ENTRY_BUCKET_IDX_LSB +: HASH_BUCKET_W] = bucket_idx;
      pack_entry[ENTRY_BUCKET_SLOT_LSB +: HASH_SLOT_W] = bucket_slot;
    end
  endfunction

  /***************reg*******************/
  reg                             r_entry_wr_en;
  reg      [ENTRY_ADDR_W-1:0]     r_entry_wr_addr;
  reg      [ENTRY_WORD_W-1:0]     r_entry_wr_data;
  reg      [ENTRY_ADDR_W-1:0]     r_entry_rd_addr;
  wire     [ENTRY_WORD_W-1:0]     r_entry_rd_data;

  reg                             r_init_req_dly;
  reg                             r_init_busy;
  reg      [HASH_BUCKET_W-1:0]    r_init_bucket_addr;
  reg      [ENTRY_ADDR_W-1:0]     r_init_entry_addr;
  reg                             r_init_bucket_done;
  reg                             r_init_entry_done;

  reg      [ENTRY_ADDR_W-1:0]     r_free_entry_fifo  [0:NUM_ENTRY-1];
  reg      [ENTRY_ADDR_W-1:0]     r_free_entry_rd_ptr;
  reg      [ENTRY_ADDR_W-1:0]     r_free_entry_wr_ptr;
  reg      [ENTRY_ADDR_W:0]       r_free_entry_count;
  reg      [ENTRY_ADDR_W-1:0]     r_free_entry_front_addr;
  reg      [ENTRY_ADDR_W-1:0]     r_pending_alloc_addr;
  reg                             r_pending_alloc_vld;
  reg      [31:0]                 r_pending_flow_id;
  reg      [19:0]                 r_pending_head;
  reg      [HASH_BUCKET_W-1:0]    r_pending_bucket_idx;
  reg      [HASH_SLOT_W-1:0]      r_pending_bucket_slot;

  reg      [ENTRY_ADDR_W-1:0]     r_read_entry_addr;
  reg      [ENTRY_WORD_W-1:0]     r_read_entry_word;
  reg      [ENTRY_WORD_W-1:0]     r_match_entry_word;

  reg      [HASH_BUCKET_W-1:0]    r_primary_bucket_rd_addr;
  reg      [HASH_BUCKET_W-1:0]    r_secondary_bucket_rd_addr;
  reg      [HASH_BUCKET_DEPTH-1:0] r_bucket_wr_en;
  reg      [HASH_BUCKET_W-1:0]    r_bucket_wr_addr [0:HASH_BUCKET_DEPTH-1];
  reg      [BUCKET_SLOT_W-1:0]    r_bucket_wr_data [0:HASH_BUCKET_DEPTH-1];

  reg      [3:0]                  r_search_state;
  reg      [31:0]                 r_search_flow_id;
  reg      [31:0]                 r_hash_xor0;
  reg      [31:0]                 r_hash_mul0;
  reg      [31:0]                 r_hash_xor1;
  reg      [31:0]                 r_hash_mul1;
  reg      [HASH_BUCKET_W-1:0]    r_search_primary_bucket;
  reg      [HASH_BUCKET_W-1:0]    r_search_secondary_bucket;
  reg      [HASH_SLOT_W-1:0]      r_search_slot;
  reg      [ENTRY_ADDR_W-1:0]     r_search_candidate_addr;
  reg                             r_search_empty_avail;
  reg      [HASH_BUCKET_W-1:0]    r_search_empty_bucket_idx;
  reg      [HASH_SLOT_W-1:0]      r_search_empty_slot_idx;

  reg      [ENTRY_ADDR_W-1:0]     r_deq_scan_addr;
  reg      [ENTRY_ADDR_W-1:0]     r_deq_wait_addr;
  reg                             r_deq_wait_vld;
  reg      [ENTRY_ADDR_W-1:0]     r_deq_check_addr;
  reg                             r_deq_check_vld;
  reg      [ENTRY_ADDR_W:0]       r_deq_scan_count;
  reg                             r_deq_fallback_vld;
  reg      [ENTRY_ADDR_W-1:0]     r_deq_fallback_addr;
  reg      [ENTRY_WORD_W-1:0]     r_deq_fallback_word;

  integer                         i;
  integer                         s;

  /***************wire******************/
  wire     [BUCKET_SLOT_W-1:0]    w_primary_bucket_rd_data [0:HASH_BUCKET_DEPTH-1];
  wire     [BUCKET_SLOT_W-1:0]    w_secondary_bucket_rd_data [0:HASH_BUCKET_DEPTH-1];
  wire                            w_start_search;

  wire                            w_entry_valid;
  wire     [ENTRY_DEPTH_W-1:0]    w_entry_depth;
  wire     [19:0]                 w_entry_head;
  wire     [19:0]                 w_entry_tail;
  wire     [31:0]                 w_entry_flow_id;
  wire     [HASH_BUCKET_W-1:0]    w_entry_bucket_idx;
  wire     [HASH_SLOT_W-1:0]      w_entry_bucket_slot;
  wire                            w_entry_nonempty;
  wire                            w_entry_meets_threshold;

  wire                            w_match_valid;
  wire     [ENTRY_DEPTH_W-1:0]    w_match_depth;
  wire     [19:0]                 w_match_head;
  wire     [31:0]                 w_match_flow_id;
  wire     [HASH_BUCKET_W-1:0]    w_match_bucket_idx;
  wire     [HASH_SLOT_W-1:0]      w_match_bucket_slot;

  wire                            w_read_valid;
  wire     [ENTRY_DEPTH_W-1:0]    w_read_depth;
  wire     [19:0]                 w_read_tail;
  wire     [31:0]                 w_read_flow_id;
  wire     [HASH_BUCKET_W-1:0]    w_read_bucket_idx;
  wire     [HASH_SLOT_W-1:0]      w_read_bucket_slot;

  wire     [ENTRY_DEPTH_W-1:0]    w_match_depth_inc;
  wire     [ENTRY_DEPTH_W-1:0]    w_depth_max;
  wire     [ENTRY_DEPTH_W-1:0]    w_pending_depth;
  wire     [ENTRY_DEPTH_W-1:0]    w_read_depth_dec;
  wire     [ENTRY_ADDR_W-1:0]     w_free_front_addr;
  wire     [31:0]                 w_hash_final;
  wire     [HASH_BUCKET_W-1:0]    w_hash_primary_bucket;
  wire     [HASH_BUCKET_W-1:0]    w_hash_secondary_raw;
  wire     [HASH_BUCKET_W-1:0]    w_hash_secondary_bucket;

  assign w_start_search = i_search_req && !r_init_busy && (r_search_state == SEARCH_IDLE);

  assign w_entry_valid       = r_entry_rd_data[ENTRY_VALID_LSB];
  assign w_entry_depth       = r_entry_rd_data[ENTRY_DEPTH_LSB +: ENTRY_DEPTH_W];
  assign w_entry_head        = r_entry_rd_data[ENTRY_HEAD_LSB +: 20];
  assign w_entry_tail        = r_entry_rd_data[ENTRY_TAIL_LSB +: 20];
  assign w_entry_flow_id     = r_entry_rd_data[ENTRY_FLOW_LSB +: 32];
  assign w_entry_bucket_idx  = r_entry_rd_data[ENTRY_BUCKET_IDX_LSB +: HASH_BUCKET_W];
  assign w_entry_bucket_slot = r_entry_rd_data[ENTRY_BUCKET_SLOT_LSB +: HASH_SLOT_W];
  assign w_entry_nonempty    = w_entry_valid && (w_entry_depth != {ENTRY_DEPTH_W{1'b0}});
  assign w_entry_meets_threshold = w_entry_nonempty &&
         ((READ_DEPTH_THRESHOLD == 0) ||
          (w_entry_depth > READ_DEPTH_THRESHOLD));

  assign w_match_valid       = r_match_entry_word[ENTRY_VALID_LSB];
  assign w_match_depth       = r_match_entry_word[ENTRY_DEPTH_LSB +: ENTRY_DEPTH_W];
  assign w_match_head        = r_match_entry_word[ENTRY_HEAD_LSB +: 20];
  assign w_match_flow_id     = r_match_entry_word[ENTRY_FLOW_LSB +: 32];
  assign w_match_bucket_idx  = r_match_entry_word[ENTRY_BUCKET_IDX_LSB +: HASH_BUCKET_W];
  assign w_match_bucket_slot = r_match_entry_word[ENTRY_BUCKET_SLOT_LSB +: HASH_SLOT_W];

  assign w_read_valid       = r_read_entry_word[ENTRY_VALID_LSB];
  assign w_read_depth       = r_read_entry_word[ENTRY_DEPTH_LSB +: ENTRY_DEPTH_W];
  assign w_read_tail        = r_read_entry_word[ENTRY_TAIL_LSB +: 20];
  assign w_read_flow_id     = r_read_entry_word[ENTRY_FLOW_LSB +: 32];
  assign w_read_bucket_idx  = r_read_entry_word[ENTRY_BUCKET_IDX_LSB +: HASH_BUCKET_W];
  assign w_read_bucket_slot = r_read_entry_word[ENTRY_BUCKET_SLOT_LSB +: HASH_SLOT_W];

  assign w_depth_max       = {ENTRY_DEPTH_W{1'b1}};
  assign w_match_depth_inc = i_pkt_done ? ((w_match_depth == w_depth_max) ? w_match_depth : (w_match_depth + 1'b1)) : w_match_depth;
  assign w_pending_depth   = i_pkt_done ? {{(ENTRY_DEPTH_W-1){1'b0}}, 1'b1} : {ENTRY_DEPTH_W{1'b0}};
  assign w_read_depth_dec  = w_read_depth - 1'b1;
  assign w_free_front_addr = r_free_entry_fifo[r_free_entry_rd_ptr];
  assign w_hash_final      = r_hash_mul1 ^ (r_hash_mul1 >> 16);
  assign w_hash_primary_bucket = w_hash_final[HASH_BUCKET_W-1:0];
  assign w_hash_secondary_raw  = w_hash_final[31 -: HASH_BUCKET_W] ^ w_hash_primary_bucket;
  assign w_hash_secondary_bucket =
         (w_hash_secondary_raw == w_hash_primary_bucket) ?
         (w_hash_primary_bucket + {{(HASH_BUCKET_W-1){1'b0}}, 1'b1}) :
         w_hash_secondary_raw;

  /***************component*************/
  generate
    genvar slot_idx;
    for (slot_idx = 0; slot_idx < 8; slot_idx = slot_idx + 1)
    begin : BUCKET_SLOT_RAM
      fmt_bucket_slot_ram u_fmt_bucket_slot_ram_primary (
                            .clka  (i_clk                     ),
                            .wea   (r_bucket_wr_en[slot_idx]  ),
                            .addra (r_bucket_wr_addr[slot_idx]),
                            .dina  (r_bucket_wr_data[slot_idx]),
                            .clkb  (i_clk                     ),
                            .enb   (1'b1                      ),
                            .addrb (r_primary_bucket_rd_addr  ),
                            .doutb (w_primary_bucket_rd_data[slot_idx])
                          );

      fmt_bucket_slot_ram u_fmt_bucket_slot_ram_secondary (
                            .clka  (i_clk                     ),
                            .wea   (r_bucket_wr_en[slot_idx]  ),
                            .addra (r_bucket_wr_addr[slot_idx]),
                            .dina  (r_bucket_wr_data[slot_idx]),
                            .clkb  (i_clk                     ),
                            .enb   (1'b1                      ),
                            .addrb (r_secondary_bucket_rd_addr),
                            .doutb (w_secondary_bucket_rd_data[slot_idx])
                          );
    end
  endgenerate

  /***************entry ram*************/
  fmt_entry_ram  u_fmt_entry_ram (
      .clka  (i_clk),
      .wea   (r_entry_wr_en),
      .addra (r_entry_wr_addr),
      .dina  (r_entry_wr_data),
      .clkb  (i_clk),
      .addrb (r_entry_rd_addr),
      .doutb (r_entry_rd_data)
  );

  /***************always****************/
  always @(posedge i_clk)
  begin
    if (i_rst)
    begin
      r_entry_wr_en           <= 1'b0;
      r_entry_wr_addr         <= {ENTRY_ADDR_W{1'b0}};
      r_entry_wr_data         <= {ENTRY_WORD_W{1'b0}};
      r_entry_rd_addr         <= {ENTRY_ADDR_W{1'b0}};
      r_init_req_dly          <= 1'b0;
      r_init_busy             <= 1'b0;
      r_init_bucket_addr      <= {HASH_BUCKET_W{1'b0}};
      r_init_entry_addr       <= {ENTRY_ADDR_W{1'b0}};
      r_init_bucket_done      <= 1'b0;
      r_init_entry_done       <= 1'b0;
      o_init_ack              <= 1'b0;
      o_search_hit            <= 1'b0;
      o_search_miss           <= 1'b0;
      o_match_tail            <= 20'd0;
      o_match_addr            <= {ENTRY_ADDR_W{1'b0}};
      r_free_entry_rd_ptr     <= {ENTRY_ADDR_W{1'b0}};
      r_free_entry_wr_ptr     <= {ENTRY_ADDR_W{1'b0}};
      r_free_entry_count      <= NUM_ENTRY[ENTRY_ADDR_W:0];
      r_free_entry_front_addr <= {ENTRY_ADDR_W{1'b0}};
      r_pending_alloc_addr    <= {ENTRY_ADDR_W{1'b0}};
      r_pending_alloc_vld     <= 1'b0;
      r_pending_flow_id       <= 32'd0;
      r_pending_head          <= 20'd0;
      r_pending_bucket_idx    <= {HASH_BUCKET_W{1'b0}};
      r_pending_bucket_slot   <= {HASH_SLOT_W{1'b0}};
      r_read_entry_addr       <= {ENTRY_ADDR_W{1'b0}};
      r_read_entry_word       <= {ENTRY_WORD_W{1'b0}};
      r_match_entry_word      <= {ENTRY_WORD_W{1'b0}};
      r_primary_bucket_rd_addr <= {HASH_BUCKET_W{1'b0}};
      r_secondary_bucket_rd_addr <= {HASH_BUCKET_W{1'b0}};
      r_search_state          <= SEARCH_IDLE;
      r_search_flow_id        <= 32'd0;
      r_hash_xor0             <= 32'd0;
      r_hash_mul0             <= 32'd0;
      r_hash_xor1             <= 32'd0;
      r_hash_mul1             <= 32'd0;
      r_search_primary_bucket <= {HASH_BUCKET_W{1'b0}};
      r_search_secondary_bucket <= {HASH_BUCKET_W{1'b0}};
      r_search_slot           <= {HASH_SLOT_W{1'b0}};
      r_search_candidate_addr <= {ENTRY_ADDR_W{1'b0}};
      r_search_empty_avail    <= 1'b0;
      r_search_empty_bucket_idx <= {HASH_BUCKET_W{1'b0}};
      r_search_empty_slot_idx <= {HASH_SLOT_W{1'b0}};
      r_deq_scan_addr         <= {ENTRY_ADDR_W{1'b0}};
      r_deq_wait_addr         <= {ENTRY_ADDR_W{1'b0}};
      r_deq_wait_vld          <= 1'b0;
      r_deq_check_addr        <= {ENTRY_ADDR_W{1'b0}};
      r_deq_check_vld         <= 1'b0;
      r_deq_scan_count        <= {(ENTRY_ADDR_W+1){1'b0}};
      r_deq_fallback_vld      <= 1'b0;
      r_deq_fallback_addr     <= {ENTRY_ADDR_W{1'b0}};
      r_deq_fallback_word     <= {ENTRY_WORD_W{1'b0}};
      o_dequeue_req           <= 1'b0;
      o_flow_head             <= 20'd0;
      for (s = 0; s < HASH_BUCKET_DEPTH; s = s + 1)
      begin
        r_bucket_wr_en[s]   <= 1'b0;
        r_bucket_wr_addr[s] <= {HASH_BUCKET_W{1'b0}};
        r_bucket_wr_data[s] <= {BUCKET_SLOT_W{1'b0}};
      end
      for (i = 0; i < NUM_ENTRY; i = i + 1)
        r_free_entry_fifo[i] <= i[ENTRY_ADDR_W-1:0];
    end
    else
    begin
      r_init_req_dly          <= i_init_req;
      o_init_ack              <= 1'b0;
      r_entry_wr_en           <= 1'b0;
      r_free_entry_front_addr <= r_free_entry_fifo[r_free_entry_rd_ptr];
      o_search_hit            <= 1'b0;
      o_search_miss           <= 1'b0;

      for (s = 0; s < HASH_BUCKET_DEPTH; s = s + 1)
        r_bucket_wr_en[s] <= 1'b0;

      if (i_refresh_head_vld)
        o_dequeue_req <= 1'b0;

      if (i_init_req && !r_init_req_dly)
      begin
        r_init_busy             <= 1'b1;
        r_init_bucket_addr      <= {HASH_BUCKET_W{1'b0}};
        r_init_entry_addr       <= {ENTRY_ADDR_W{1'b0}};
        r_init_bucket_done      <= 1'b0;
        r_init_entry_done       <= 1'b0;
        r_free_entry_rd_ptr     <= {ENTRY_ADDR_W{1'b0}};
        r_free_entry_wr_ptr     <= {ENTRY_ADDR_W{1'b0}};
        r_free_entry_count      <= NUM_ENTRY[ENTRY_ADDR_W:0];
        r_free_entry_front_addr <= {ENTRY_ADDR_W{1'b0}};
        r_pending_alloc_addr    <= {ENTRY_ADDR_W{1'b0}};
        r_pending_alloc_vld     <= 1'b0;
        o_search_hit            <= 1'b0;
        o_search_miss           <= 1'b0;
        o_match_tail            <= 20'd0;
        o_match_addr            <= {ENTRY_ADDR_W{1'b0}};
        r_search_state          <= SEARCH_IDLE;
        r_hash_xor0             <= 32'd0;
        r_hash_mul0             <= 32'd0;
        r_hash_xor1             <= 32'd0;
        r_hash_mul1             <= 32'd0;
        r_search_empty_avail    <= 1'b0;
        r_deq_wait_vld          <= 1'b0;
        r_deq_check_vld         <= 1'b0;
        r_deq_scan_count        <= {(ENTRY_ADDR_W+1){1'b0}};
        r_deq_fallback_vld      <= 1'b0;
        o_dequeue_req           <= 1'b0;
        o_flow_head             <= 20'd0;
        for (i = 0; i < NUM_ENTRY; i = i + 1)
          r_free_entry_fifo[i] <= i[ENTRY_ADDR_W-1:0];
      end
      else if (r_init_busy)
      begin
        if (!r_init_bucket_done)
        begin
          for (s = 0; s < HASH_BUCKET_DEPTH; s = s + 1)
          begin
            r_bucket_wr_en[s]   <= 1'b1;
            r_bucket_wr_addr[s] <= r_init_bucket_addr;
            r_bucket_wr_data[s] <= {BUCKET_SLOT_W{1'b0}};
          end

          if (r_init_bucket_addr == HASH_BUCKET_NUM - 1)
            r_init_bucket_done <= 1'b1;
          else
            r_init_bucket_addr <= r_init_bucket_addr + 1'b1;
        end

        if (!r_init_entry_done)
        begin
          r_entry_wr_en   <= 1'b1;
          r_entry_wr_addr <= r_init_entry_addr;
          r_entry_wr_data <= {ENTRY_WORD_W{1'b0}};
          if (r_init_entry_addr == NUM_ENTRY - 1)
            r_init_entry_done <= 1'b1;
          else
            r_init_entry_addr <= r_init_entry_addr + 1'b1;
        end

        if (((r_init_bucket_done || (r_init_bucket_addr == HASH_BUCKET_NUM - 1)) &&
             (r_init_entry_done  || (r_init_entry_addr == NUM_ENTRY - 1))))
        begin
          r_init_busy <= 1'b0;
          o_init_ack  <= 1'b1;
        end
      end
      else
      begin
        if (i_alloc_head_req)
        begin
          r_deq_scan_count   <= {(ENTRY_ADDR_W+1){1'b0}};
          r_deq_fallback_vld <= 1'b0;
          if ((r_free_entry_count != 0) && !r_pending_alloc_vld && r_search_empty_avail)
          begin
            r_pending_alloc_addr   <= w_free_front_addr;
            r_pending_alloc_vld    <= 1'b1;
            r_pending_flow_id      <= i_flow_id;
            r_pending_head         <= i_alloc_head_data;
            r_pending_bucket_idx   <= r_search_empty_bucket_idx;
            r_pending_bucket_slot  <= r_search_empty_slot_idx;

            r_entry_wr_en   <= 1'b1;
            r_entry_wr_addr <= w_free_front_addr;
            r_entry_wr_data <= pack_entry(1'b1, i_flow_id, i_alloc_head_data, 20'd0,
                                          {ENTRY_DEPTH_W{1'b0}},
                                          r_search_empty_bucket_idx, r_search_empty_slot_idx);

            r_bucket_wr_en[r_search_empty_slot_idx]   <= 1'b1;
            r_bucket_wr_addr[r_search_empty_slot_idx] <= r_search_empty_bucket_idx;
            r_bucket_wr_data[r_search_empty_slot_idx] <= {1'b1, w_free_front_addr};
          end
        end
        else if (i_alloc_tail_req)
        begin
          r_deq_scan_count   <= {(ENTRY_ADDR_W+1){1'b0}};
          r_deq_fallback_vld <= 1'b0;
          if (r_pending_alloc_vld)
          begin
            r_entry_wr_en   <= 1'b1;
            r_entry_wr_addr <= r_pending_alloc_addr;
            r_entry_wr_data <= pack_entry(1'b1, r_pending_flow_id, r_pending_head,
                                          i_alloc_tail_data, w_pending_depth,
                                          r_pending_bucket_idx, r_pending_bucket_slot);
            if ((READ_DEPTH_THRESHOLD == 0) ||
                (w_pending_depth >= READ_DEPTH_THRESHOLD))
            begin
              r_deq_fallback_vld  <= 1'b1;
              r_deq_fallback_addr <= r_pending_alloc_addr;
              r_deq_fallback_word <= pack_entry(1'b1, r_pending_flow_id, r_pending_head,
                                                i_alloc_tail_data, w_pending_depth,
                                                r_pending_bucket_idx, r_pending_bucket_slot);
            end
            r_free_entry_rd_ptr <= next_ptr(r_free_entry_rd_ptr);
            r_free_entry_count  <= r_free_entry_count - 1'b1;
            r_pending_alloc_vld <= 1'b0;
          end
        end
        else if (i_tail_update_req)
        begin
          r_deq_scan_count   <= {(ENTRY_ADDR_W+1){1'b0}};
          r_deq_fallback_vld <= 1'b0;
          if (w_match_valid)
          begin
            r_entry_wr_en   <= 1'b1;
            r_entry_wr_addr <= o_match_addr;
            r_entry_wr_data <= pack_entry(1'b1, w_match_flow_id, w_match_head,
                                          i_refresh_tail, w_match_depth_inc,
                                          w_match_bucket_idx, w_match_bucket_slot);
            r_match_entry_word <= pack_entry(1'b1, w_match_flow_id, w_match_head,
                                             i_refresh_tail, w_match_depth_inc,
                                             w_match_bucket_idx, w_match_bucket_slot);
            if ((READ_DEPTH_THRESHOLD == 0) ||
                (w_match_depth_inc >= READ_DEPTH_THRESHOLD))
            begin
              r_deq_fallback_vld  <= 1'b1;
              r_deq_fallback_addr <= o_match_addr;
              r_deq_fallback_word <= pack_entry(1'b1, w_match_flow_id, w_match_head,
                                                i_refresh_tail, w_match_depth_inc,
                                                w_match_bucket_idx, w_match_bucket_slot);
            end
          end
        end
        else if (i_refresh_head_vld)
        begin
          r_deq_scan_count   <= {(ENTRY_ADDR_W+1){1'b0}};
          r_deq_fallback_vld <= 1'b0;
          if (w_read_valid)
          begin
            if (w_read_depth_dec == {ENTRY_DEPTH_W{1'b0}})
            begin
              r_entry_wr_en   <= 1'b1;
              r_entry_wr_addr <= r_read_entry_addr;
              r_entry_wr_data <= {ENTRY_WORD_W{1'b0}};
              r_free_entry_fifo[r_free_entry_wr_ptr] <= r_read_entry_addr;
              r_free_entry_wr_ptr                    <= next_ptr(r_free_entry_wr_ptr);
              r_free_entry_count                     <= r_free_entry_count + 1'b1;
              r_bucket_wr_en[w_read_bucket_slot]     <= 1'b1;
              r_bucket_wr_addr[w_read_bucket_slot]   <= w_read_bucket_idx;
              r_bucket_wr_data[w_read_bucket_slot]   <= {BUCKET_SLOT_W{1'b0}};
            end
            else
            begin
              r_entry_wr_en   <= 1'b1;
              r_entry_wr_addr <= r_read_entry_addr;
              r_entry_wr_data <= pack_entry(1'b1, w_read_flow_id, i_refresh_head,
                                            w_read_tail, w_read_depth_dec,
                                            w_read_bucket_idx, w_read_bucket_slot);
              r_read_entry_word <= pack_entry(1'b1, w_read_flow_id, i_refresh_head,
                                              w_read_tail, w_read_depth_dec,
                                              w_read_bucket_idx, w_read_bucket_slot);
              if ((READ_DEPTH_THRESHOLD == 0) ||
                  (w_read_depth_dec >= READ_DEPTH_THRESHOLD))
              begin
                r_deq_fallback_vld  <= 1'b1;
                r_deq_fallback_addr <= r_read_entry_addr;
                r_deq_fallback_word <= pack_entry(1'b1, w_read_flow_id, i_refresh_head,
                                                  w_read_tail, w_read_depth_dec,
                                                  w_read_bucket_idx, w_read_bucket_slot);
              end
            end
          end
        end

        if (w_start_search)
        begin
          r_search_flow_id           <= i_flow_id;
          r_hash_xor0                <= i_flow_id ^ (i_flow_id >> 16);
          r_hash_mul0                <= 32'd0;
          r_hash_xor1                <= 32'd0;
          r_hash_mul1                <= 32'd0;
          r_search_slot              <= {HASH_SLOT_W{1'b0}};
          r_search_empty_avail       <= 1'b0;
          r_search_empty_bucket_idx  <= {HASH_BUCKET_W{1'b0}};
          r_search_empty_slot_idx    <= {HASH_SLOT_W{1'b0}};
          r_search_state             <= SEARCH_HASH_MUL0;
          r_deq_wait_vld             <= 1'b0;
          r_deq_check_vld            <= 1'b0;
          o_search_hit               <= 1'b0;
          o_search_miss              <= 1'b0;
        end
        else
        begin
          case (r_search_state)
            SEARCH_HASH_MUL0:
            begin
              r_hash_mul0    <= r_hash_xor0 * MIX32_MUL0;
              r_search_state <= SEARCH_HASH_XOR1;
              r_deq_wait_vld  <= 1'b0;
              r_deq_check_vld <= 1'b0;
            end
            SEARCH_HASH_XOR1:
            begin
              r_hash_xor1    <= r_hash_mul0 ^ (r_hash_mul0 >> 15);
              r_search_state <= SEARCH_HASH_MUL1;
              r_deq_wait_vld  <= 1'b0;
              r_deq_check_vld <= 1'b0;
            end
            SEARCH_HASH_MUL1:
            begin
              r_hash_mul1    <= r_hash_xor1 * MIX32_MUL1;
              r_search_state <= SEARCH_HASH_DONE;
              r_deq_wait_vld  <= 1'b0;
              r_deq_check_vld <= 1'b0;
            end
            SEARCH_HASH_DONE:
            begin
              r_primary_bucket_rd_addr   <= w_hash_primary_bucket;
              r_secondary_bucket_rd_addr <= w_hash_secondary_bucket;
              r_search_primary_bucket    <= w_hash_primary_bucket;
              r_search_secondary_bucket  <= w_hash_secondary_bucket;
              r_search_slot              <= {HASH_SLOT_W{1'b0}};
              r_search_state             <= SEARCH_WAIT_BUCKET;
              r_deq_wait_vld             <= 1'b0;
              r_deq_check_vld            <= 1'b0;
            end
            SEARCH_WAIT_BUCKET:
            begin
              r_search_slot  <= {HASH_SLOT_W{1'b0}};
              r_search_state <= SEARCH_PRI_SLOT;
              r_deq_check_vld <= 1'b0;
            end
            SEARCH_PRI_SLOT:
            begin
              r_deq_check_vld <= 1'b0;
              if (w_primary_bucket_rd_data[r_search_slot][ENTRY_ADDR_W])
              begin
                r_search_candidate_addr <= w_primary_bucket_rd_data[r_search_slot][ENTRY_ADDR_W-1:0];
                r_entry_rd_addr         <= w_primary_bucket_rd_data[r_search_slot][ENTRY_ADDR_W-1:0];
                r_search_state          <= SEARCH_PRI_WAIT;
              end
              else
              begin
                if (!r_search_empty_avail)
                begin
                  r_search_empty_avail      <= 1'b1;
                  r_search_empty_bucket_idx <= r_search_primary_bucket;
                  r_search_empty_slot_idx   <= r_search_slot;
                end
                if (r_search_slot == HASH_BUCKET_DEPTH - 1)
                begin
                  r_search_slot  <= {HASH_SLOT_W{1'b0}};
                  r_search_state <= SEARCH_SEC_SLOT;
                end
                else
                  r_search_slot <= r_search_slot + 1'b1;
              end
            end
            SEARCH_PRI_WAIT:
            begin
              r_deq_wait_vld  <= 1'b0;
              r_deq_check_vld <= 1'b0;
              r_search_state  <= SEARCH_PRI_ENTRY;
            end
            SEARCH_PRI_ENTRY:
            begin
              r_deq_wait_vld  <= 1'b0;
              r_deq_check_vld <= 1'b0;
              if (w_entry_valid &&
                  (w_entry_flow_id == r_search_flow_id))
              begin
                o_search_hit       <= 1'b1;
                o_search_miss      <= 1'b0;
                o_match_tail       <= w_entry_tail;
                o_match_addr       <= r_search_candidate_addr;
                r_match_entry_word <= r_entry_rd_data;
                r_search_state     <= SEARCH_IDLE;
              end
              else if (r_search_slot == HASH_BUCKET_DEPTH - 1)
              begin
                r_search_slot  <= {HASH_SLOT_W{1'b0}};
                r_search_state <= SEARCH_SEC_SLOT;
              end
              else
              begin
                r_search_slot  <= r_search_slot + 1'b1;
                r_search_state <= SEARCH_PRI_SLOT;
              end
            end
            SEARCH_SEC_SLOT:
            begin
              r_deq_check_vld <= 1'b0;
              if (w_secondary_bucket_rd_data[r_search_slot][ENTRY_ADDR_W])
              begin
                r_search_candidate_addr <= w_secondary_bucket_rd_data[r_search_slot][ENTRY_ADDR_W-1:0];
                r_entry_rd_addr         <= w_secondary_bucket_rd_data[r_search_slot][ENTRY_ADDR_W-1:0];
                r_search_state          <= SEARCH_SEC_WAIT;
              end
              else
              begin
                if (!r_search_empty_avail)
                begin
                  r_search_empty_avail      <= 1'b1;
                  r_search_empty_bucket_idx <= r_search_secondary_bucket;
                  r_search_empty_slot_idx   <= r_search_slot;
                end
                if (r_search_slot == HASH_BUCKET_DEPTH - 1)
                begin
                  o_search_hit   <= 1'b0;
                  o_search_miss  <= 1'b1;
                  o_match_tail   <= 20'd0;
                  o_match_addr   <= {ENTRY_ADDR_W{1'b0}};
                  r_search_state <= SEARCH_IDLE;
                end
                else
                  r_search_slot <= r_search_slot + 1'b1;
              end
            end
            SEARCH_SEC_WAIT:
            begin
              r_deq_wait_vld  <= 1'b0;
              r_deq_check_vld <= 1'b0;
              r_search_state  <= SEARCH_SEC_ENTRY;
            end
            SEARCH_SEC_ENTRY:
            begin
              r_deq_wait_vld  <= 1'b0;
              r_deq_check_vld <= 1'b0;
              if (w_entry_valid &&
                  (w_entry_flow_id == r_search_flow_id))
              begin
                o_search_hit       <= 1'b1;
                o_search_miss      <= 1'b0;
                o_match_tail       <= w_entry_tail;
                o_match_addr       <= r_search_candidate_addr;
                r_match_entry_word <= r_entry_rd_data;
                r_search_state     <= SEARCH_IDLE;
              end
              else if (r_search_slot == HASH_BUCKET_DEPTH - 1)
              begin
                o_search_hit   <= 1'b0;
                o_search_miss  <= 1'b1;
                o_match_tail   <= 20'd0;
                o_match_addr   <= {ENTRY_ADDR_W{1'b0}};
                r_search_state <= SEARCH_IDLE;
              end
              else
              begin
                r_search_slot  <= r_search_slot + 1'b1;
                r_search_state <= SEARCH_SEC_SLOT;
              end
            end
            default:
              r_search_state <= SEARCH_IDLE;
          endcase
        end

        if (!w_start_search && (r_search_state == SEARCH_IDLE) &&
            !i_alloc_head_req && !i_alloc_tail_req && !i_tail_update_req && !i_refresh_head_vld)
        begin
          if (!o_dequeue_req && r_deq_fallback_vld)
          begin
            o_dequeue_req      <= 1'b1;
            o_flow_head        <= r_deq_fallback_word[ENTRY_HEAD_LSB +: 20];
            r_read_entry_addr  <= r_deq_fallback_addr;
            r_read_entry_word  <= r_deq_fallback_word;
            r_deq_fallback_vld <= 1'b0;
            r_deq_check_vld    <= 1'b0;
            r_deq_wait_vld     <= 1'b0;
            r_deq_scan_count   <= {(ENTRY_ADDR_W+1){1'b0}};
          end
          else if (!o_dequeue_req && r_deq_check_vld &&
              w_entry_meets_threshold)
          begin
            o_dequeue_req     <= 1'b1;
            o_flow_head       <= w_entry_head;
            r_read_entry_addr <= r_deq_check_addr;
            r_read_entry_word <= r_entry_rd_data;
            r_deq_check_vld   <= 1'b0;
            r_deq_wait_vld    <= 1'b0;
            r_deq_scan_count  <= {(ENTRY_ADDR_W+1){1'b0}};
            r_deq_fallback_vld <= 1'b0;
          end
          else if (!o_dequeue_req && r_deq_check_vld)
          begin
            r_deq_check_vld <= 1'b0;
            if (r_deq_scan_count == NUM_ENTRY - 1)
            begin
              r_deq_scan_count   <= {(ENTRY_ADDR_W+1){1'b0}};
              r_deq_fallback_vld <= 1'b0;
              r_entry_rd_addr   <= r_deq_scan_addr;
              r_deq_wait_addr   <= r_deq_scan_addr;
              r_deq_wait_vld    <= 1'b1;
              r_deq_scan_addr   <= next_ptr(r_deq_scan_addr);
            end
            else
            begin
              r_deq_scan_count <= r_deq_scan_count + 1'b1;
              r_entry_rd_addr <= r_deq_scan_addr;
              r_deq_wait_addr <= r_deq_scan_addr;
              r_deq_wait_vld  <= 1'b1;
              r_deq_scan_addr <= next_ptr(r_deq_scan_addr);
            end
          end
          else if (!o_dequeue_req && r_deq_wait_vld)
          begin
            r_deq_check_addr <= r_deq_wait_addr;
            r_deq_check_vld  <= 1'b1;
            r_deq_wait_vld   <= 1'b0;
          end
          else if (!o_dequeue_req)
          begin
            r_entry_rd_addr   <= r_deq_scan_addr;
            r_deq_wait_addr   <= r_deq_scan_addr;
            r_deq_wait_vld    <= 1'b1;
            r_deq_scan_addr   <= next_ptr(r_deq_scan_addr);
          end
        end
      end
    end
  end

endmodule
