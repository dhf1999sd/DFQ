`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company:         NNS@TSN
// Engineer:        Wenxue Wu
// Create Date:     2024/05/07
// Module Name:     FMT
// Project Name:    Dynamic per flow queues
// Target Devices:  Zynq
// Tool Versions:   VIVADO 2023.2
// Description:     CAM table manager supporting initialization, write, search, and refresh
//////////////////////////////////////////////////////////////////////////////////

module FMT #(
    parameter NUM_ENTRY = 32,   // Number of CAM entries
    parameter ENTRY_WIDTH = 77  // Entry width including flow ID, head pointer, tail pointer, and depth flag
) (
    input               clk,                    // Clock
    input               reset,                  // Asynchronous reset
    input               init_req,               // Initialization request
    output reg          init_ack,               // Initialization acknowledge
    input               cam_wr_search,          // Search request
    output reg          cam_matched,            // Search matched
    output reg          cam_mismatched,         // Search mismatched
    output reg  [19:0]  cam_match_tail,         // Tail pointer of matched entry
    output reg  [3:0]   cam_match_addr,         // Address of matched entry
    input       [19:0]  cam_refresh_tail,       // Refresh tail pointer
    input               depth_flag,             // Depth flag
    input               cam_wr_search_ack,      // Search acknowledge
    input               cam_wr_head_req,        // Write head request
    input       [19:0]  cam_wr_head,            // Write head pointer
    input               cam_wr_tail_req,        // Write tail request
    input       [19:0]  cam_wr_tail,            // Write tail pointer
    input       [31:0]  flow_ID,                // Flow ID
    output reg          ptr_read,               // Pointer read enable
    output reg  [19:0]  cam_read_head,          // CAM read head pointer
    output reg          read_mode_flag,         // Read mode flag
    input       [19:0]  cam_refresh_head,       // Refresh head pointer
    input               cam_refresh_head_flag   // Refresh head flag
  );

  // Internal registers and variable declarations
  reg  [3:0]  cam_addr_fifo_din;
  reg         cam_addr_fifo_wr;
  reg         cam_addr_fifo_rd;
  wire [3:0]  cam_addr_fifo_dout;
  // CAM entry width is configured as 77 bits
  reg  [ENTRY_WIDTH-1:0] cam [0:NUM_ENTRY-1];
  reg  [ENTRY_WIDTH-1:0] cam_scheduler [0:NUM_ENTRY-1];
  reg  [3:0]  state;
  reg  [5:0]  aging_addr;
  reg  [3:0]  cam_read_addr;
  integer     i, j, m, k, r;

  // Search result registers
  reg         cam_matched_reg;
  reg         cam_mismatched_reg;
  reg  [19:0]  cam_match_tail_reg;
  reg  [3:0]  cam_match_addr_reg;

  // State machine
  always @(posedge clk)
  begin
    if (reset)
    begin
      state               <= 0;
      cam_addr_fifo_din   <= 0;
      cam_addr_fifo_wr    <= 0;
      cam_addr_fifo_rd    <= 0;
      init_ack            <= 0;
      aging_addr          <= 0;
      i                   <= 0;
      m                   <= 0;
    end
    else
    begin
      case (state)
        0:
        begin
          cam_addr_fifo_wr <= 0;
          if (init_req)
          begin
            i     <= 0;
            state <= 4;
          end
          //
          else if (cam_wr_head_req)
          begin
            cam[cam_addr_fifo_dout][71:40] <= flow_ID;
            cam[cam_addr_fifo_dout][39:20] <= cam_wr_head;
            state <= 1;

          end
          else if (cam_wr_tail_req)
          begin
            cam[cam_addr_fifo_dout][19:0]   <= cam_wr_tail;
            state                           <= 1;
            cam_addr_fifo_rd                <= 1;
            cam[cam_addr_fifo_dout][75:72]  <= depth_flag ? cam[cam_addr_fifo_dout][75:72] + 1 : cam[cam_addr_fifo_dout][75:72];
            cam[cam_addr_fifo_dout][ENTRY_WIDTH-1]     <= 1;
          end
          else if (cam_wr_search_ack)
          begin
            cam[cam_match_addr][19:0]      <= cam_refresh_tail;
            cam[cam_match_addr][75:72]     <= depth_flag ? cam[cam_match_addr][75:72] + 1 : cam[cam_match_addr][75:72];
          end
          else if (cam_refresh_head_flag)
          begin
            cam[cam_read_addr][39:20]      <= cam_refresh_head;
            cam[cam_read_addr][75:72]      <= cam[cam_read_addr][75:72] - 1;
            cam[cam_read_addr][76]         <= 0;
            if(cam[cam_read_addr][75:72] - 1==0)
            begin
              cam_addr_fifo_din <= cam_read_addr;
              cam_addr_fifo_wr <= 1;
              cam[cam_read_addr] <= 0;
            end
          end
        end
        1:
        begin
          cam_addr_fifo_wr <= 0;
          cam_addr_fifo_rd <= 0;
          state            <= 0;
        end
        2:
        begin
          cam_addr_fifo_din <= i[3:0];
          cam_addr_fifo_wr  <= 1;
          cam[i]            <= 'h0;
          if (i < NUM_ENTRY - 1)
            i <= i + 1;
          else
          begin
            init_ack <= 1;
            state    <= 3;
          end
        end
        3:
        begin
          cam_addr_fifo_wr <= 0;
          init_ack         <= 0;
          state            <= 0;
        end
        4:
          state <= 5;
        5:
          state <= 6;
        6:
          state <= 7;
        7:
          state <= 8;
        8:
          state <= 9;
        9:
          state <= 10;
        10:
          state <= 11;
        11:
          state <= 12;
        12:
          state <= 2;
        default:
          state <= 0;
      endcase
    end
  end

  // cam_scheduler synchronization
  always @(*)
  begin
    for (k = 0; k < NUM_ENTRY; k = k + 1)
      cam_scheduler[k] = cam[k];
  end

  // Search logic
  always @(posedge clk)
  begin
    if (reset)
    begin
      j                <= 0;
      cam_matched      <= 0;
      cam_mismatched   <= 0;
      cam_match_tail   <= 0;
      cam_match_addr   <= 0;
      cam_matched_reg  <= 0;
      cam_mismatched_reg <= 0;
      cam_match_tail_reg <= 0;
      cam_match_addr_reg <= 0;
    end
    else
    begin
      cam_matched      <= cam_matched_reg;
      cam_mismatched   <= cam_mismatched_reg;
      cam_match_tail   <= cam_match_tail_reg;
      cam_match_addr   <= cam_match_addr_reg;

      if (cam_wr_search)
      begin
        cam_matched_reg    <= 0;
        cam_mismatched_reg <= 1'b1;
        for (j = 0; j < NUM_ENTRY; j = j + 1)
        begin
          if (flow_ID == cam[j][71:40] && cam[j][75:72] != 4'd0)
          begin
            cam_matched_reg    <= 1'b1;
            cam_mismatched_reg <= 1'b0;
            cam_match_tail_reg <= cam[j][19:0];
            cam_match_addr_reg <= j;
          end
        end
      end
      else if (cam_wr_search_ack)
      begin
        cam_matched_reg    <= 0;
        cam_mismatched_reg <= 1'b0;

      end
    end
  end

  // Pointer read logic
  always @(posedge clk)
  begin
    if (reset)
    begin
      r              <= 0;
      ptr_read       <= 0;
      cam_read_head  <= 0;
      cam_read_addr  <= 0;
      read_mode_flag <= 0;
    end
    else
    begin
      for (r = 0; r < NUM_ENTRY; r = r + 1)
      begin
        if (cam_scheduler[r][75:72] > 3 && !ptr_read)
        begin
          ptr_read       <= 1;
          cam_read_head  <= cam_scheduler[r][39:20];
          read_mode_flag <= cam_scheduler[r][76];
          cam_read_addr  <= r;
        end
        else if (cam_refresh_head_flag == 1)
        begin
          ptr_read <= 0;
        end
      end
    end
  end

  // FIFO instance
  FMT_fifo_ft u_fifo_ft_w4_d16 (
                .clk       (clk),
                .rst       (reset),
                .din       (cam_addr_fifo_din),
                .wr_en     (cam_addr_fifo_wr),
                .rd_en     (cam_addr_fifo_rd),
                .dout      (cam_addr_fifo_dout),
                .full      (),
                .empty     (),
                .data_count()
              );

endmodule
