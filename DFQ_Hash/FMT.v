//////////////////////////////////////////////////////////////////////////////////
// Company:         LZU
// Engineer:        WenxuWu
// Module Name:     FMT
// Description:     Hash-bucket flow table implemented with inferred block RAM.
//////////////////////////////////////////////////////////////////////////////////

`timescale 1ns / 1ps

module FMT #(
    parameter integer NUM_ENTRY   = 200,
    parameter integer ENTRY_WIDTH = 77,
    parameter integer NUM_BUCKET  = 4,
    parameter integer ADDR_WIDTH  = ((NUM_ENTRY + NUM_BUCKET - 1) <= 1) ? 1 : $clog2(NUM_ENTRY + NUM_BUCKET - 1)
  )(
    input               clk,
    input               reset,
    input               init_req,
    output              init_ack,
    input               hash_wr_search,
    output              hash_search_busy,
    output              hash_matched,
    output              hash_mismatched,
    output [19:0]       hash_match_tail,
    output [ADDR_WIDTH-1:0] hash_match_addr,
    input  [19:0]       hash_refresh_tail,
    input               depth_flag,
    input               hash_wr_search_ack,
    input               hash_wr_head_req,
    input  [19:0]       hash_wr_head,
    input               hash_wr_tail_req,
    input  [19:0]       hash_wr_tail,
    input  [31:0]       flow_ID,
    output              ptr_read,
    output [19:0]       hash_read_head,
    output              read_mode_flag,
    input  [19:0]       hash_refresh_head,
    input               hash_refresh_head_flag
  );

  FMT_table_manager #(
    .NUM_ENTRY(NUM_ENTRY),
    .ENTRY_WIDTH(ENTRY_WIDTH),
    .NUM_BUCKET(NUM_BUCKET),
    .ADDR_WIDTH(ADDR_WIDTH)
  ) u_table_manager (
    .clk(clk),
    .reset(reset),
    .init_req(init_req),
    .init_ack(init_ack),
    .hash_wr_search(hash_wr_search),
    .hash_search_busy(hash_search_busy),
    .hash_matched(hash_matched),
    .hash_mismatched(hash_mismatched),
    .hash_match_tail(hash_match_tail),
    .hash_match_addr(hash_match_addr),
    .hash_refresh_tail(hash_refresh_tail),
    .depth_flag(depth_flag),
    .hash_wr_search_ack(hash_wr_search_ack),
    .hash_wr_head_req(hash_wr_head_req),
    .hash_wr_head(hash_wr_head),
    .hash_wr_tail_req(hash_wr_tail_req),
    .hash_wr_tail(hash_wr_tail),
    .flow_ID(flow_ID),
    .ptr_read(ptr_read),
    .hash_read_head(hash_read_head),
    .read_mode_flag(read_mode_flag),
    .hash_refresh_head(hash_refresh_head),
    .hash_refresh_head_flag(hash_refresh_head_flag)
  );

endmodule

module FMT_table_manager #(
    parameter integer NUM_ENTRY   = 200,
    parameter integer ENTRY_WIDTH = 77,
    parameter integer NUM_BUCKET  = 4,
    parameter integer ADDR_WIDTH  = ((NUM_ENTRY + NUM_BUCKET - 1) <= 1) ? 1 : $clog2(NUM_ENTRY + NUM_BUCKET - 1)
  )(
    input                       clk,
    input                       reset,
    input                       init_req,
    output reg                  init_ack,
    input                       hash_wr_search,
    output                      hash_search_busy,
    output reg                  hash_matched,
    output reg                  hash_mismatched,
    output reg [19:0]           hash_match_tail,
    output reg [ADDR_WIDTH-1:0] hash_match_addr,
    input      [19:0]           hash_refresh_tail,
    input                       depth_flag,
    input                       hash_wr_search_ack,
    input                       hash_wr_head_req,
    input      [19:0]           hash_wr_head,
    input                       hash_wr_tail_req,
    input      [19:0]           hash_wr_tail,
    input      [31:0]           flow_ID,
    output reg                  ptr_read,
    output reg [19:0]           hash_read_head,
    output reg                  read_mode_flag,
    input      [19:0]           hash_refresh_head,
    input                       hash_refresh_head_flag
  );

  localparam integer EFFECTIVE_NUM_BUCKET = (NUM_BUCKET <= 1) ? 1 : NUM_BUCKET;
  localparam integer BUCKET_ID_WIDTH      = (EFFECTIVE_NUM_BUCKET <= 1) ? 1 : $clog2(EFFECTIVE_NUM_BUCKET);
  localparam integer BUCKET_DEPTH         = (NUM_ENTRY + EFFECTIVE_NUM_BUCKET - 1) / EFFECTIVE_NUM_BUCKET;
  localparam integer TABLE_SIZE           = BUCKET_DEPTH * EFFECTIVE_NUM_BUCKET;
  localparam integer TABLE_ADDR_WIDTH     = (TABLE_SIZE <= 1) ? 1 : $clog2(TABLE_SIZE);

  localparam [2:0] SEARCH_IDLE  = 3'd0;
  localparam [2:0] SEARCH_READ  = 3'd1;
  localparam [2:0] SEARCH_WAIT  = 3'd2;
  localparam [2:0] SEARCH_CHECK = 3'd3;
  localparam [1:0] SCAN_READ    = 2'd0;
  localparam [1:0] SCAN_WAIT    = 2'd1;
  localparam [1:0] SCAN_CHECK   = 2'd2;

  function [ENTRY_WIDTH-1:0] make_entry;
    input valid;
    input [3:0] depth;
    input [31:0] flow_id;
    input [19:0] head_ptr;
    input [19:0] tail_ptr;
    begin
      make_entry = {valid, depth, flow_id, head_ptr, tail_ptr};
    end
  endfunction

  function [BUCKET_ID_WIDTH-1:0] flow_hash_bucket;
    input [31:0] flow_id;
    reg [1:0] legacy_hash;
    reg [31:0] folded_hash;
    begin
      legacy_hash = flow_id[1:0] ^ flow_id[3:2] ^ flow_id[5:4] ^ flow_id[7:6] ^
                    flow_id[9:8] ^ flow_id[11:10] ^ flow_id[13:12] ^ flow_id[15:14] ^
                    flow_id[17:16] ^ flow_id[19:18] ^ flow_id[21:20] ^ flow_id[23:22] ^
                    flow_id[25:24] ^ flow_id[27:26] ^ flow_id[29:28] ^ flow_id[31:30];
      folded_hash = flow_id ^ {flow_id[15:0], flow_id[31:16]} ^
                    {flow_id[7:0], flow_id[31:8]} ^ {flow_id[3:0], flow_id[31:4]};
      if (EFFECTIVE_NUM_BUCKET <= 1)
        flow_hash_bucket = {BUCKET_ID_WIDTH{1'b0}};
      else if (EFFECTIVE_NUM_BUCKET == 4)
        flow_hash_bucket = legacy_hash;
      else
        flow_hash_bucket = folded_hash % EFFECTIVE_NUM_BUCKET;
    end
  endfunction

  function [TABLE_ADDR_WIDTH-1:0] bucket_slot_addr_int;
    input [BUCKET_ID_WIDTH-1:0] bucket_id;
    input [TABLE_ADDR_WIDTH-1:0] slot_id;
    begin
      bucket_slot_addr_int = (bucket_id * BUCKET_DEPTH) + slot_id;
    end
  endfunction

  function [ADDR_WIDTH-1:0] fit_addr;
    input [TABLE_ADDR_WIDTH-1:0] addr;
    begin
      fit_addr = addr[ADDR_WIDTH-1:0];
    end
  endfunction

  function [TABLE_ADDR_WIDTH-1:0] next_table_addr;
    input [TABLE_ADDR_WIDTH-1:0] addr;
    begin
      next_table_addr = (addr == TABLE_SIZE - 1) ? {TABLE_ADDR_WIDTH{1'b0}} : (addr + 1'b1);
    end
  endfunction

  /***************reg*******************/
  (* ram_style = "block" *) reg [ENTRY_WIDTH-1:0] hash_entry [0:TABLE_SIZE-1];

  reg hash_wr_en;
  reg [TABLE_ADDR_WIDTH-1:0] hash_wr_addr;
  reg [ENTRY_WIDTH-1:0] hash_wr_data;
  reg [TABLE_ADDR_WIDTH-1:0] hash_rd_addr;
  reg [ENTRY_WIDTH-1:0] hash_rd_data;
  reg init_active;
  reg init_wait_release;
  reg [TABLE_ADDR_WIDTH-1:0] init_idx;
  reg [2:0] search_state;
  reg [31:0] search_flow_id_q;
  reg [BUCKET_ID_WIDTH-1:0] search_bucket_id_q;
  reg [TABLE_ADDR_WIDTH-1:0] search_slot_idx;
  reg [TABLE_ADDR_WIDTH-1:0] search_free_slot;
  reg search_free_valid;
  reg [TABLE_ADDR_WIDTH-1:0] search_addr_q;
  reg [TABLE_ADDR_WIDTH-1:0] matched_entry_addr;
  reg [ENTRY_WIDTH-1:0] matched_entry;
  reg matched_entry_valid;
  reg [TABLE_ADDR_WIDTH-1:0] allocated_entry_addr;
  reg [ENTRY_WIDTH-1:0] allocated_entry;
  reg allocated_entry_valid;
  reg [1:0] scan_state;
  reg [TABLE_ADDR_WIDTH-1:0] scan_addr;
  reg [TABLE_ADDR_WIDTH-1:0] scan_addr_q;
  reg [TABLE_ADDR_WIDTH-1:0] dequeue_entry_addr;
  reg [ENTRY_WIDTH-1:0] dequeue_entry;
  reg dequeue_entry_valid;

  wire search_last_slot = (search_slot_idx == (BUCKET_DEPTH - 1));
  wire [TABLE_ADDR_WIDTH-1:0] search_addr_calc = bucket_slot_addr_int(search_bucket_id_q, search_slot_idx);

  assign hash_search_busy = init_req | init_active | (search_state != SEARCH_IDLE) |
                            hash_matched | hash_mismatched | ptr_read | dequeue_entry_valid;

  always @*
  begin
    hash_wr_en = 1'b0;
    hash_wr_addr = {TABLE_ADDR_WIDTH{1'b0}};
    hash_wr_data = {ENTRY_WIDTH{1'b0}};

    if (init_active)
    begin
      hash_wr_en = 1'b1;
      hash_wr_addr = init_idx;
    end
    else if (!init_req)
    begin
      if (hash_wr_head_req && search_free_valid)
      begin
        hash_wr_en = 1'b1;
        hash_wr_addr = search_free_slot;
        hash_wr_data = make_entry(1'b1, 4'd0, search_flow_id_q, hash_wr_head, hash_wr_head);
      end
      else if (hash_wr_tail_req && allocated_entry_valid)
      begin
        hash_wr_en = 1'b1;
        hash_wr_addr = allocated_entry_addr;
        hash_wr_data = make_entry(1'b1, depth_flag ? 4'd1 : 4'd0,
                                  allocated_entry[71:40], allocated_entry[39:20], hash_wr_tail);
      end
      else if (hash_wr_search_ack && matched_entry_valid)
      begin
        hash_wr_en = 1'b1;
        hash_wr_addr = matched_entry_addr;
        hash_wr_data = make_entry(1'b1,
                                  depth_flag ? (matched_entry[75:72] + 1'b1) : matched_entry[75:72],
                                  matched_entry[71:40], matched_entry[39:20], hash_refresh_tail);
      end
      else if (hash_refresh_head_flag && dequeue_entry_valid)
      begin
        hash_wr_en = 1'b1;
        hash_wr_addr = dequeue_entry_addr;
        if (dequeue_entry[75:72] == 4'd1)
          hash_wr_data = {ENTRY_WIDTH{1'b0}};
        else
          hash_wr_data = make_entry(1'b1, dequeue_entry[75:72] - 1'b1,
                                    dequeue_entry[71:40], hash_refresh_head, dequeue_entry[19:0]);
      end
    end
  end

  always @(posedge clk)
  begin
    if (hash_wr_en)
      hash_entry[hash_wr_addr] <= hash_wr_data;
    hash_rd_data <= hash_entry[hash_rd_addr];
  end

  always @(posedge clk)
  begin
    if (reset)
    begin
      init_ack <= 1'b0;
      init_active <= 1'b0;
      init_wait_release <= 1'b0;
      init_idx <= {TABLE_ADDR_WIDTH{1'b0}};
      hash_matched <= 1'b0;
      hash_mismatched <= 1'b0;
      hash_match_tail <= 20'd0;
      hash_match_addr <= {ADDR_WIDTH{1'b0}};
      ptr_read <= 1'b0;
      hash_read_head <= 20'd0;
      read_mode_flag <= 1'b0;
      search_state <= SEARCH_IDLE;
      search_flow_id_q <= 32'd0;
      search_bucket_id_q <= {BUCKET_ID_WIDTH{1'b0}};
      search_slot_idx <= {TABLE_ADDR_WIDTH{1'b0}};
      search_free_slot <= {TABLE_ADDR_WIDTH{1'b0}};
      search_free_valid <= 1'b0;
      search_addr_q <= {TABLE_ADDR_WIDTH{1'b0}};
      hash_rd_addr <= {TABLE_ADDR_WIDTH{1'b0}};
      matched_entry_addr <= {TABLE_ADDR_WIDTH{1'b0}};
      matched_entry <= {ENTRY_WIDTH{1'b0}};
      matched_entry_valid <= 1'b0;
      allocated_entry_addr <= {TABLE_ADDR_WIDTH{1'b0}};
      allocated_entry <= {ENTRY_WIDTH{1'b0}};
      allocated_entry_valid <= 1'b0;
      scan_state <= SCAN_READ;
      scan_addr <= {TABLE_ADDR_WIDTH{1'b0}};
      scan_addr_q <= {TABLE_ADDR_WIDTH{1'b0}};
      dequeue_entry_addr <= {TABLE_ADDR_WIDTH{1'b0}};
      dequeue_entry <= {ENTRY_WIDTH{1'b0}};
      dequeue_entry_valid <= 1'b0;
    end
    else
    begin
      init_ack <= 1'b0;

      if (hash_wr_search_ack | hash_wr_tail_req)
      begin
        hash_matched <= 1'b0;
        hash_mismatched <= 1'b0;
        matched_entry_valid <= 1'b0;
      end

      if (!init_req)
        init_wait_release <= 1'b0;

      if (init_req && !init_active && !init_wait_release)
      begin
        init_active <= 1'b1;
        init_idx <= {TABLE_ADDR_WIDTH{1'b0}};
      end

      if (init_active)
      begin
        if (init_idx == (TABLE_SIZE - 1))
        begin
          init_active <= 1'b0;
          init_wait_release <= 1'b1;
          init_ack <= 1'b1;
        end
        else
          init_idx <= init_idx + 1'b1;
      end
      else if (!init_req)
      begin
        if (hash_wr_head_req && search_free_valid)
        begin
          allocated_entry_addr <= search_free_slot;
          allocated_entry <= hash_wr_data;
          allocated_entry_valid <= 1'b1;
          search_free_valid <= 1'b0;
        end
        else if (hash_wr_tail_req && allocated_entry_valid)
        begin
          allocated_entry <= hash_wr_data;
          allocated_entry_valid <= 1'b0;
        end
        else if (hash_wr_search_ack && matched_entry_valid)
        begin
          matched_entry <= hash_wr_data;
        end
        else if (hash_refresh_head_flag && dequeue_entry_valid)
        begin
          ptr_read <= 1'b0;
          dequeue_entry_valid <= 1'b0;
          scan_addr <= next_table_addr(dequeue_entry_addr);
          scan_state <= SCAN_READ;
        end

        case (search_state)
          SEARCH_IDLE:
          begin
            if (hash_wr_search && !hash_search_busy)
            begin
              hash_matched <= 1'b0;
              hash_mismatched <= 1'b0;
              hash_match_tail <= 20'd0;
              hash_match_addr <= {ADDR_WIDTH{1'b0}};
              matched_entry_valid <= 1'b0;
              search_flow_id_q <= flow_ID;
              search_bucket_id_q <= flow_hash_bucket(flow_ID);
              search_slot_idx <= {TABLE_ADDR_WIDTH{1'b0}};
              search_free_slot <= {TABLE_ADDR_WIDTH{1'b0}};
              search_free_valid <= 1'b0;
              search_state <= SEARCH_READ;
              scan_state <= SCAN_READ;
            end
          end

          SEARCH_READ:
          begin
            search_addr_q <= search_addr_calc;
            hash_rd_addr <= search_addr_calc;
            search_state <= SEARCH_WAIT;
          end

          SEARCH_WAIT:
          begin
            search_state <= SEARCH_CHECK;
          end

          SEARCH_CHECK:
          begin
            if ((hash_rd_data[76] == 1'b1) && (hash_rd_data[71:40] == search_flow_id_q))
            begin
              hash_matched <= 1'b1;
              hash_mismatched <= 1'b0;
              hash_match_tail <= hash_rd_data[19:0];
              hash_match_addr <= fit_addr(search_addr_q);
              matched_entry_addr <= search_addr_q;
              matched_entry <= hash_rd_data;
              matched_entry_valid <= 1'b1;
              search_state <= SEARCH_IDLE;
            end
            else
            begin
              if ((hash_rd_data[76] == 1'b0) && !search_free_valid)
              begin
                search_free_slot <= search_addr_q;
                search_free_valid <= 1'b1;
              end
              if (search_last_slot)
              begin
                hash_matched <= 1'b0;
                hash_mismatched <= 1'b1;
                search_state <= SEARCH_IDLE;
              end
              else
              begin
                search_slot_idx <= search_slot_idx + 1'b1;
                search_state <= SEARCH_READ;
              end
            end
          end

          default:
          begin
            search_state <= SEARCH_IDLE;
          end
        endcase

        if (!hash_wr_search && (search_state == SEARCH_IDLE) &&
            !hash_matched && !hash_mismatched && !ptr_read)
        begin
          case (scan_state)
            SCAN_READ:
            begin
              scan_addr_q <= scan_addr;
              hash_rd_addr <= scan_addr;
              scan_state <= SCAN_WAIT;
            end
            SCAN_WAIT:
            begin
              scan_state <= SCAN_CHECK;
            end
            SCAN_CHECK:
            begin
              if ((hash_rd_data[76] == 1'b1) && (hash_rd_data[75:72] > 4'd0))
              begin
                ptr_read <= 1'b1;
                hash_read_head <= hash_rd_data[39:20];
                read_mode_flag <= hash_rd_data[76];
                dequeue_entry_addr <= scan_addr_q;
                dequeue_entry <= hash_rd_data;
                dequeue_entry_valid <= 1'b1;
              end
              else
              begin
                scan_addr <= next_table_addr(scan_addr_q);
                scan_state <= SCAN_READ;
              end
            end

            default:
            begin
              scan_state <= SCAN_READ;
            end
          endcase
        end
      end
    end
  end

endmodule
