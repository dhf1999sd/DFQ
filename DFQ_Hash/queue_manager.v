//////////////////////////////////////////////////////////////////////////////////
// Company:
// Engineer:
// Create Date:     2024/05/07 21:23:04
// Module Name:     queue_manager
// Project Name:
// Target Devices:
// Tool Versions:
// Description:
//////////////////////////////////////////////////////////////////////////////////

`timescale 1ns / 1ps

module queue_manager#(
    parameter       NUM_PRIORITY  = 1,
    parameter       FMT_NUM_ENTRY = 100
  )(
    input           clk             ,
    input           reset           ,
    input   [31:0]  flow_ID         ,
    input   [2:0]   PCP             ,
    input   [19:0]  metadata_in     ,
    input           metadata_in_wr  ,
    output          ptr_rdy         ,
    input           metadata_out_rd ,
    output  [19:0]  metadata_out    ,
    output          q_full
  );

  /***************function**************/
  /***************parameter*************/
  localparam       PRIORITY_WIDTH = (NUM_PRIORITY <= 1) ? 1 : $clog2(NUM_PRIORITY);
  localparam       FMT_ADDR_WIDTH = (FMT_NUM_ENTRY <= 1) ? 1 : $clog2(FMT_NUM_ENTRY);
  localparam       INGRESS_FLOW_FIFO_DEPTH = 64;
  localparam       INGRESS_FLOW_FIFO_ADDR_W = 6;

  /***************port******************/
  /***************mechine***************/
  function [PRIORITY_WIDTH-1:0] pcp_to_priority;
    input [2:0] pcp;
    integer scaled_priority;
    begin
      if (NUM_PRIORITY <= 1)
        pcp_to_priority = {PRIORITY_WIDTH{1'b0}};
      else if (NUM_PRIORITY == 3)
      begin
        if (pcp <= 3'd3)
          pcp_to_priority = {{(PRIORITY_WIDTH-1){1'b0}}, 1'b0};
        else if (pcp <= 3'd5)
          pcp_to_priority = {{(PRIORITY_WIDTH-1){1'b0}}, 1'b1};
        else
          pcp_to_priority = {{(PRIORITY_WIDTH-2){1'b0}}, 2'd2};
      end
      else if (NUM_PRIORITY >= 8)
        pcp_to_priority = pcp[PRIORITY_WIDTH-1:0];
      else
      begin
        scaled_priority = (pcp * NUM_PRIORITY) >> 3;
        if (scaled_priority >= NUM_PRIORITY)
          pcp_to_priority = NUM_PRIORITY - 1;
        else
          pcp_to_priority = scaled_priority[PRIORITY_WIDTH-1:0];
      end
    end
  endfunction

  /***************reg*******************/
  reg  [NUM_PRIORITY-1:0]    r_init_req;
  reg                        r_unused_cam_wr_search;
  reg  [31:0]                r_unused_search_flow_id;
  reg                        r_ptr_wr_ack;
  reg  [NUM_PRIORITY-1:0]    r_ptr_ram_wr_en;
  reg                        r_ptr_wr_req;
  reg  [3:0]                 r_flow_wr_state;
  reg  [19:0]                r_flow_tail [NUM_PRIORITY-1:0];
  reg  [19:0]                r_flow_head [NUM_PRIORITY-1:0];
  reg  [22:0]                r_ptr_din;
  reg  [31:0]                r_ptr_flow_id;
  reg  [19:0]                r_ptr_ram_din [NUM_PRIORITY-1:0];
  wire [19:0]                w_ptr_ram_dout [NUM_PRIORITY-1:0];
  reg  [10:0]                r_ptr_ram_addr [NUM_PRIORITY-1:0];
  wire [10:0]                w_ptr_ram_rd_addr [NUM_PRIORITY-1:0];
  wire [19:0]                w_prio_queue_din [NUM_PRIORITY-1:0];
  reg  [9:0]                 r_pkt_cell_count [NUM_PRIORITY-1:0];
  reg                        r_pkt_done [NUM_PRIORITY-1:0];
  wire [9:0]                 w_unused_ptr_fifo_dout;
  wire [9:0]                 w_unused_ptr_fifo_empty;
  wire [9:0]                 w_unused_ptr_fifo_full;
  wire [9:0]                 w_unused_ptr_fifo_count;
  wire [9:0]                 w_unused_ptr_fifo_space;
  wire [9:0]                 w_unused_ptr_fifo_wr_req;
  wire [9:0]                 w_unused_ptr_fifo_wr_tail;
  wire [9:0]                 w_unused_ptr_fifo_wr_head;
  wire [9:0]                 w_unused_ptr_fifo_wr_addr;
  reg                        r_ingress_fifo_rd_en;
  reg  [31:0]                r_ingress_flow_fifo [0:INGRESS_FLOW_FIFO_DEPTH-1];
  reg  [INGRESS_FLOW_FIFO_ADDR_W-1:0] r_ingress_flow_fifo_wr_ptr;
  reg  [INGRESS_FLOW_FIFO_ADDR_W-1:0] r_ingress_flow_fifo_rd_ptr;
  reg  [31:0]                r_ingress_flow_fifo_dout;
  wire [NUM_PRIORITY-1:0]    w_fmt_search_hit;
  wire [NUM_PRIORITY-1:0]    w_fmt_search_miss;
  reg  [9:0]                 r_unused_match_tail_buffer;
  reg  [1:0]                 r_ingress_wr_state;
  wire [6:0]                 w_prio_queue_count [NUM_PRIORITY-1:0];
  wire [22:0]                w_ingress_fifo_dout;
  wire                       w_ingress_fifo_empty;
  wire                       w_ingress_fifo_wr_en;
  reg  [8:0]                 r_unused_addr_cnt;
  reg  [NUM_PRIORITY-1:0]    r_fmt_tail_update_req;
  reg  [19:0]                r_fmt_refresh_tail [NUM_PRIORITY-1:0];
  reg  [NUM_PRIORITY-1:0]    r_fmt_alloc_head_req;
  reg  [NUM_PRIORITY-1:0]    r_fmt_alloc_tail_req;
  reg  [19:0]                r_fmt_alloc_tail_data [NUM_PRIORITY-1:0];
  reg  [19:0]                r_fmt_alloc_head_data [NUM_PRIORITY-1:0];
  wire [NUM_PRIORITY-1:0]    w_init_ack;
  wire [NUM_PRIORITY-1:0]    w_dequeue_req;
  wire [19:0]                w_flow_head [NUM_PRIORITY-1:0];
  wire [19:0]                w_prio_queue_dout [NUM_PRIORITY-1:0];
  reg  [NUM_PRIORITY-1:0]    r_write_busy;
  reg  [NUM_PRIORITY-1:0]    r_fmt_search_req;
  reg  [31:0]                r_fmt_flow_id_reg [NUM_PRIORITY-1:0];
  reg                        r_search_issued;
  reg                        r_search_miss_latched;
  wire [NUM_PRIORITY-1:0]    w_read_busy;
  wire [NUM_PRIORITY-1:0]    w_prio_queue_rd_en;
  reg  [NUM_PRIORITY-1:0]    r_unused_enb;
  reg  [3:0]                 r_unused_flow_queue_rd_state [NUM_PRIORITY-1:0];
  reg  [19:0]                r_unused_rd_head [NUM_PRIORITY-1:0];
  wire [19:0]                w_refresh_head [NUM_PRIORITY-1:0];
  wire [NUM_PRIORITY-1:0]    w_refresh_head_vld;
  reg  [PRIORITY_WIDTH-1:0]  r_wr_prio_idx;
  reg                        r_ptr_wr_nack;
  integer                    i;
  integer                    out_idx;
  reg  [19:0]                r_out_metadata;
  reg                        r_out_metadata_wr;
  reg                        r_out_found;

  /***************wire******************/
  wire [19:0]                w_fmt_match_tail [NUM_PRIORITY-1:0];
  wire [NUM_PRIORITY-1:0]    w_prio_queue_wr;
  wire [PRIORITY_WIDTH-1:0]  w_in_prio_idx;
  wire [NUM_PRIORITY-1:0]    w_fmt_in_wr;
  wire [19:0]                w_fmt_in_data [NUM_PRIORITY-1:0];
  wire [31:0]                w_fmt_flow_id [NUM_PRIORITY-1:0];
  wire [FMT_ADDR_WIDTH-1:0]  w_fmt_match_addr [NUM_PRIORITY-1:0];
  wire                       w_output_queue_empty;
  wire                       w_output_queue_full;
  wire [NUM_PRIORITY-1:0]    w_prio_queue_full;
  wire [NUM_PRIORITY-1:0]    w_prio_queue_empty;
  wire [NUM_PRIORITY-1:0]    w_prio_queue_valid;
  wire [NUM_PRIORITY-1:0]    w_arb_req_in;
  wire [NUM_PRIORITY-1:0]    w_arb_grant_out;
  wire [NUM_PRIORITY-1:0]    w_prio_queue_rd_en_int;
  wire                       w_fmt_init_done;

  /***************component*************/
  /***************assign****************/
  assign w_in_prio_idx    = pcp_to_priority(PCP);
  assign w_prio_queue_valid  = ~w_prio_queue_empty;
  assign w_arb_req_in        = w_prio_queue_valid;
  assign w_prio_queue_rd_en_int = w_arb_grant_out;
  assign w_prio_queue_rd_en  = w_output_queue_full ? {NUM_PRIORITY{1'b0}} : w_prio_queue_rd_en_int;
  assign ptr_rdy         = !w_output_queue_empty;
  assign w_fmt_init_done = (r_init_req == {NUM_PRIORITY{1'b0}});
  assign w_ingress_fifo_wr_en = metadata_in_wr && !q_full;

  /***************always****************/
  always @(posedge clk)
  begin
    if (reset)
    begin
      r_init_req <= {NUM_PRIORITY{1'b1}};
    end
    else
    begin
      for (i = 0; i < NUM_PRIORITY; i = i + 1)
      begin
        if (w_init_ack[i])
          r_init_req[i] <= 1'b0;
      end
    end
  end

  /***************component*************/
  fifo_d64_in_queue_port u_ptr_wr_fifo (
                           .clk        (clk                        ),
                           .rst        (reset                      ),
                           .din        ({PCP[2:0], metadata_in[19:0]}),
                           .wr_en      (w_ingress_fifo_wr_en       ),
                           .rd_en      (r_ingress_fifo_rd_en                       ),
                           .dout       (w_ingress_fifo_dout                     ),
                           .full       (q_full                     ),
                           .empty      (w_ingress_fifo_empty                    ),
                           .data_count (                           )
                          );

  always @(posedge clk)
  begin
    if (reset)
    begin
      r_ingress_flow_fifo_wr_ptr <= {INGRESS_FLOW_FIFO_ADDR_W{1'b0}};
      r_ingress_flow_fifo_rd_ptr <= {INGRESS_FLOW_FIFO_ADDR_W{1'b0}};
      r_ingress_flow_fifo_dout   <= 32'd0;
    end
    else
    begin
      if (w_ingress_fifo_wr_en)
      begin
        r_ingress_flow_fifo[r_ingress_flow_fifo_wr_ptr] <= flow_ID;
        r_ingress_flow_fifo_wr_ptr <= r_ingress_flow_fifo_wr_ptr + 1'b1;
      end

      if (r_ingress_fifo_rd_en)
      begin
        r_ingress_flow_fifo_dout <= r_ingress_flow_fifo[r_ingress_flow_fifo_rd_ptr];
        r_ingress_flow_fifo_rd_ptr <= r_ingress_flow_fifo_rd_ptr + 1'b1;
      end
    end
  end

  generate
    genvar in_map_idx;
    for (in_map_idx = 0; in_map_idx < NUM_PRIORITY; in_map_idx = in_map_idx + 1)
    begin : FMT_INPUT_MAP
      localparam [PRIORITY_WIDTH-1:0] P_IDX = in_map_idx;
      assign w_fmt_in_wr[in_map_idx]   = metadata_in_wr && (w_in_prio_idx == P_IDX);
      assign w_fmt_in_data[in_map_idx] =  metadata_in;
      assign w_fmt_flow_id[in_map_idx] =  flow_ID;
    end
  endgenerate
  generate  // Cyclic treatment of different PCPs
    genvar p;
    for (p = 0; p < NUM_PRIORITY; p = p + 1)
    begin : FMT_Different_PCP
      FMT #(
            .NUM_ENTRY (FMT_NUM_ENTRY)
          ) u_CAM_FMT (
            .i_clk                 (clk                             ),
            .i_rst                 (reset                           ),
            .i_init_req            (r_init_req[p]                   ),
            .o_init_ack             (w_init_ack[p]                     ),
            .i_search_req           (r_fmt_search_req[p]              ),
            .o_search_hit           (w_fmt_search_hit[p]               ),
            .o_search_miss          (w_fmt_search_miss[p]              ),
            .o_match_tail           (w_fmt_match_tail[p]               ),
            .o_match_addr           (w_fmt_match_addr[p]               ),
            .i_refresh_tail         (r_fmt_refresh_tail[p]             ),
            .i_pkt_done             (r_pkt_done[p]                     ),
            .i_tail_update_req      (r_fmt_tail_update_req[p]          ),
            .i_alloc_head_req       (r_fmt_alloc_head_req[p]           ),
            .i_alloc_head_data      (r_fmt_alloc_head_data[p]          ),
            .i_alloc_tail_req       (r_fmt_alloc_tail_req[p]           ),
            .i_alloc_tail_data      (r_fmt_alloc_tail_data[p]          ),
            .i_flow_id             (r_fmt_flow_id_reg[p]              ),
            .o_dequeue_req          (w_dequeue_req[p]                  ),
            .o_flow_head            (w_flow_head[p]                    ),
            .i_refresh_head         (w_refresh_head[p]                 ),
            .i_refresh_head_vld     (w_refresh_head_vld[p]             )
          );
    end
  endgenerate

  always @(posedge clk)
    if (reset)
    begin
      r_ptr_din     <= 0;
      r_ptr_flow_id <= 32'd0;
      r_ptr_wr_req      <= 0;
      r_ingress_fifo_rd_en        <= 0;
      r_ingress_wr_state <= 0;
      r_wr_prio_idx <= {PRIORITY_WIDTH{1'b0}};
    end
    else
    begin
      case (r_ingress_wr_state)
        0:
        begin
          if (w_fmt_init_done && !w_ingress_fifo_empty)
          begin
            r_ingress_fifo_rd_en        <= 1;
            r_ingress_wr_state <= 1;
          end
        end
        1:
        begin
          r_ingress_fifo_rd_en        <= 0;
          r_ingress_wr_state <= 2;
        end
        2:
        begin
          r_ptr_din     <= w_ingress_fifo_dout;
          r_ptr_flow_id <= r_ingress_flow_fifo_dout;
          r_ptr_wr_req      <= 1;
          r_ingress_wr_state <= 3;
          r_wr_prio_idx <= pcp_to_priority(w_ingress_fifo_dout[22:20]);
        end
        3:
        begin
          if (r_ptr_wr_nack)
          begin
            r_ptr_wr_req      <= 0;
            r_ingress_wr_state <= 2;
          end
          else if (r_ptr_wr_ack)
          begin
            r_ptr_wr_req      <= 0;
            r_ingress_wr_state <= 0;
          end
        end
      endcase
    end

  always @(posedge clk)
    if (reset)
    begin
      r_ptr_wr_ack     <= 0;
      r_flow_wr_state <= 0;
      r_ptr_wr_nack    <= 0;
      r_write_busy     <= 0;
      r_fmt_search_req <= {NUM_PRIORITY{1'b0}};
      r_search_issued  <= 1'b0;
      r_search_miss_latched <= 1'b0;
      for (i = 0; i < NUM_PRIORITY; i = i + 1)
      begin
        r_fmt_flow_id_reg[i] <= 32'd0;
        r_fmt_tail_update_req[i] <= 1'b0;
        r_fmt_refresh_tail[i]  <= 20'd0;
        r_fmt_alloc_tail_data[i]       <= 20'd0;
        r_fmt_alloc_head_req[i]   <= 1'b0;
        r_fmt_alloc_head_data[i]       <= 20'd0;
        r_fmt_alloc_tail_req[i]   <= 1'b0;
        r_ptr_ram_addr[i]      <= 11'd0;
        r_ptr_ram_wr_en[i]        <= 1'b0;
        r_ptr_ram_din[i]       <= 20'd0;
        r_pkt_cell_count[i]        <= 10'd0;
        r_flow_tail[i]              <= 20'd0;
        r_flow_head[i]              <= 20'd0;
        r_pkt_done[i]        <= 1'b0;
      end
    end
    else
    begin
      r_ptr_wr_ack  <= 0;
      r_ptr_wr_nack <= 1'b0;
      for (i = 0; i < NUM_PRIORITY; i = i + 1)
      begin
        r_fmt_search_req[i]        <= 1'b0;
        r_ptr_ram_wr_en[i]        <= 1'b0;
        r_fmt_tail_update_req[i] <= 1'b0;
        r_fmt_alloc_head_req[i]   <= 1'b0;
        r_fmt_alloc_tail_req[i]   <= 1'b0;
        r_fmt_alloc_tail_data[i]       <= 20'd0;
        r_pkt_done[i]        <= 1'b0;
      end

      case (r_flow_wr_state)
        0:
        begin
          if (r_ptr_wr_req)
          begin
            if (w_read_busy[r_wr_prio_idx])
            begin
              r_ptr_wr_nack          <= 1'b1;
              r_search_issued        <= 1'b0;
              r_search_miss_latched  <= 1'b0;
            end
            else if (r_ptr_din[14])
            begin
              if (!r_search_issued)
              begin
                r_fmt_flow_id_reg[r_wr_prio_idx] <= r_ptr_flow_id;
                r_fmt_search_req[r_wr_prio_idx]  <= 1'b1;
                r_search_issued                  <= 1'b1;
                r_search_miss_latched            <= 1'b0;
              end
              else if (w_fmt_search_hit[r_wr_prio_idx])
              begin
                r_flow_tail[r_wr_prio_idx]       <= w_fmt_match_tail[r_wr_prio_idx];
                r_write_busy[r_wr_prio_idx]      <= 1'b1;
                r_search_issued                  <= 1'b0;
                r_search_miss_latched            <= 1'b0;
                r_flow_wr_state                  <= 3;
              end
              else if (w_fmt_search_miss[r_wr_prio_idx])
              begin
                r_flow_tail[r_wr_prio_idx]         <= r_ptr_din[19:0];
                r_fmt_alloc_head_req[r_wr_prio_idx] <= 1'b1;
                r_fmt_alloc_head_data[r_wr_prio_idx]  <= r_ptr_din[19:0];
                r_write_busy[r_wr_prio_idx]   <= 1'b1;
                r_search_issued               <= 1'b0;
                r_search_miss_latched         <= 1'b1;
                r_flow_wr_state               <= 3;
              end
            end
            else
            begin
              r_search_issued <= 1'b0;
              r_flow_wr_state <= 3;
            end
          end
        end
        3:
        begin
          r_flow_wr_state <= 4;
        end
        4:
        begin
          r_flow_wr_state <= 1;
        end
        1:
        begin
          r_ptr_wr_ack     <= 1;
          r_flow_wr_state <= 2;
          if (r_pkt_cell_count[r_wr_prio_idx][9:0] != 10'd0)
          begin
            r_ptr_ram_wr_en[r_wr_prio_idx]        <= 1'b1;
            r_ptr_ram_addr[r_wr_prio_idx][10:0] <= r_flow_tail[r_wr_prio_idx][10:0];
            r_ptr_ram_din[r_wr_prio_idx][19:0] <= r_ptr_din[19:0];
            r_flow_tail[r_wr_prio_idx]              <= r_ptr_din[19:0];
          end
          else
          begin
            if (r_search_miss_latched == 1'b1)
            begin
              r_ptr_ram_wr_en[r_wr_prio_idx]        <= 1'b1;
              r_ptr_ram_addr[r_wr_prio_idx][10:0] <= r_ptr_din[10:0];
              r_ptr_ram_din[r_wr_prio_idx][19:0] <= r_ptr_din[19:0];
              r_flow_tail[r_wr_prio_idx]              <= r_ptr_din[19:0];
              r_flow_head[r_wr_prio_idx]              <= r_ptr_din[19:0];
            end
            else
            begin
              r_ptr_ram_wr_en[r_wr_prio_idx]        <= 1'b1;
              r_ptr_ram_addr[r_wr_prio_idx][10:0] <= r_flow_tail[r_wr_prio_idx][10:0];
              r_ptr_ram_din[r_wr_prio_idx][19:0] <= r_ptr_din[19:0];
              r_flow_tail[r_wr_prio_idx]              <= r_ptr_din[19:0];
            end
          end

          r_pkt_cell_count[r_wr_prio_idx] <= r_pkt_cell_count[r_wr_prio_idx] + 1'b1;
          if (r_ptr_din[15])
          begin
            r_pkt_done[r_wr_prio_idx] <= 1'b1;
            r_pkt_cell_count[r_wr_prio_idx] <= 10'd0;
            r_write_busy[r_wr_prio_idx] <= 1'b0;
            if (r_search_miss_latched)
            begin
              r_fmt_alloc_tail_data[r_wr_prio_idx]     <= r_ptr_din[19:0];
              r_fmt_alloc_tail_req[r_wr_prio_idx] <= 1'b1;
            end
            else
            begin
              r_fmt_tail_update_req[r_wr_prio_idx] <= 1'b1;
              r_fmt_refresh_tail[r_wr_prio_idx]  <= r_ptr_din[19:0];
            end
          end
        end
        2:
        begin
          r_flow_wr_state               <= 0;
          r_ptr_ram_addr[r_wr_prio_idx]    <= r_flow_tail[r_wr_prio_idx][10:0];
          r_ptr_ram_din[r_wr_prio_idx]     <= r_flow_tail[r_wr_prio_idx][19:0];
          r_ptr_ram_wr_en[r_wr_prio_idx]      <= 1'b1;
        end
      endcase
    end

  generate
    genvar j;
    for (j = 0; j < NUM_PRIORITY; j = j + 1)
    begin : dequeue_state_machine
      dequeue_process u_dequeue_process (
                        .i_clk                 (clk                      ),
                        .i_rst                 (reset                    ),
                        .o_read_busy           (w_read_busy[j]             ),
                        .i_dequeue_req         (w_dequeue_req[j]            ),
                        .i_write_busy          (r_write_busy[j]             ),
                        .i_prio_queue_full     (w_prio_queue_full[j]        ),
                        .i_flow_head           (w_flow_head[j]              ),
                        .o_prio_queue_din      (w_prio_queue_din[j]         ),
                        .i_ptr_ram_dout        (w_ptr_ram_dout[j]           ),
                        .o_prio_queue_wr       (w_prio_queue_wr[j]          ),
                        .o_ptr_ram_rd_addr     (w_ptr_ram_rd_addr[j]        ),
                        .o_refresh_head        (w_refresh_head[j]           ),
                        .o_refresh_head_vld    (w_refresh_head_vld[j]       )
                      );
    end
  endgenerate

  generate
    genvar q;
    for (q = 0; q < NUM_PRIORITY; q = q + 1)
    begin : FMT_Different_RAM
      sram_FMT u_flow_ram (
                 .clka   (clk                  ),  // input wire clka
                 .wea    (r_ptr_ram_wr_en[q]        ),  // input wire [0 : 0] wea
                 .addra  (r_ptr_ram_addr[q][10:0]),
                 .dina   (r_ptr_ram_din[q]       ),  // input wire [19 : 0] dina
                 .clkb   (clk                  ),  // input wire clkb
                 .addrb  (w_ptr_ram_rd_addr[q][10:0]),
                 .doutb  (w_ptr_ram_dout[q]      )  // output wire [19 : 0] doutb
               );
    end  // block: in_arb_queues
  endgenerate

  generate
    genvar y;
    for (y = 0; y < NUM_PRIORITY; y = y + 1)
    begin : PRIORITY_QUEUE
      fifo_ft_w16_d64 u_PRIORITY_queue (
                        .clk        (clk              ),
                        .rst        (reset            ),
                        .din        (w_prio_queue_din[y] ),
                        .wr_en      (w_prio_queue_wr[y]  ),
                        .rd_en      (w_prio_queue_rd_en[y] ),
                        .dout       (w_prio_queue_dout[y]),
                        .full       (w_prio_queue_full[y]),
                        .empty      (w_prio_queue_empty[y]),
                        .data_count (w_prio_queue_count[y] )
                      );
    end  // block: PRIORITY_QUEUE
  endgenerate

  priority_arbiter #(
                     .P_CHANNEL_NUM(NUM_PRIORITY)
                   ) u_priority_arbiter (
                     .i_clk       (clk             ),
                     .i_rst       (reset           ),
                     .i_req_release(|w_prio_queue_valid),
                     .i_req_in    (w_arb_req_in      ),
                     .o_grant_out (w_arb_grant_out   )
                   );

  always @(posedge clk)
  begin
    if (reset == 1'b1)
    begin
      r_out_metadata    <= 20'd0;
      r_out_metadata_wr <= 'b0;
    end
    else
    begin
      r_out_metadata    <= 20'd0;
      r_out_metadata_wr <= 1'b0;
      r_out_found    = 1'b0;
      for (out_idx = 0; out_idx < NUM_PRIORITY; out_idx = out_idx + 1)
      begin
        if (!r_out_found && w_prio_queue_rd_en[out_idx] && !w_prio_queue_empty[out_idx])
        begin
          r_out_found    = 1'b1;
          r_out_metadata    <= w_prio_queue_dout[out_idx][19:0];
          r_out_metadata_wr <= 1'b1;
        end
      end
    end
  end

  fifo_output_w20 u_output (
                    .clk        (clk                 ),  // input wire clk
                    .rst        (reset               ),  // input wire rst
                    .din        (r_out_metadata[19:0]     ),  // input wire [19 : 0] din
                    .wr_en      (r_out_metadata_wr        ),  // input wire wr_en
                    .rd_en      (metadata_out_rd     ),  // input wire rd_en
                    .dout       (metadata_out[19:0]  ),  // output wire [19 : 0] dout
                   .full       (w_output_queue_full ),  // output wire full
                    .empty      (w_output_queue_empty  ),  // output wire empty
                    .data_count (                    )  // output wire [5 : 0] data_count
                  );

endmodule
