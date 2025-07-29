`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company:
// Engineer:
//
// Create Date: 2024/05/07 22:05:26
// Design Name:
// Module Name: FMT
// Project Name:
// Target Devices:
// Tool Versions:
// Description: CAM表管理模块，支持初始化�?�写入�?�查找�?�刷新等操作
//
// Dependencies:
//
// Revision:
// Revision 0.01 - File Created
// Additional Comments:
//
//////////////////////////////////////////////////////////////////////////////////

module FMT #(
    parameter NUM_ENTRY = 32, // CAM表项数目
    parameter ENTRY_WIDTH = 77 // CAM表项宽度，包含流ID、头指针、尾指针、深度标志等信息
  )(
    input              clk,                   // 时钟
    input              reset,                 // 异步复位
    input              init_req,              // 初始化请
    output reg         init_ack,              // 初始化完成应�??
    input              cam_wr_search,         // 查找请求
    output reg         cam_matched,           // 查找命中
    output reg         cam_mismatched,        // 查找未命�??
    output reg  [19:0]  cam_match_tail,        // 命中项尾指针
    output reg  [3:0]  cam_match_addr,        // 命中项地�??
    input       [19:0]  cam_refresh_tail,      // 刷新尾指�??
    input              depth_flag,            // 深度标志
    input              cam_wr_search_ack,     // 查找应答
    input              cam_wr_head_req,       // 写头请求
    input       [19:0]  cam_wr_head,           // 写头指针
    input              cam_wr_tail_req,       // 写尾请求
    input       [19:0]  cam_wr_tail,           // 写尾指针
    input      [31:0]  flow_ID,               // 流ID
    output reg         ptr_read,              // 指针读取使能
    output reg  [19:0]  cam_read_head,         // CAM读取头指�??
    output reg         read_mode_flag,        // 读取模式标志
    input       [19:0]  cam_refresh_head,      // 刷新头指�??
    input              cam_refresh_head_flag  // 刷新头标�??
  );

  // 内部寄存器和变量声明
  reg  [3:0]  cam_addr_fifo_din;
  reg         cam_addr_fifo_wr;
  reg         cam_addr_fifo_rd;
  wire [3:0]  cam_addr_fifo_dout;
  // CAM表项宽度调整为77
  reg  [ENTRY_WIDTH-1:0] cam [0:NUM_ENTRY-1];
  reg  [ENTRY_WIDTH-1:0] cam_scheduler [0:NUM_ENTRY-1];
  reg  [3:0]  state;
  reg  [5:0]  aging_addr;
  reg  [3:0]  cam_read_addr;
  integer     i, j, m, k, r;

  // 查找结果寄存�??
  reg         cam_matched_reg;
  reg         cam_mismatched_reg;
  reg  [19:0]  cam_match_tail_reg;
  reg  [3:0]  cam_match_addr_reg;

  // 状�?�机
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

  // cam_scheduler同步
  always @(*)
  begin
    for (k = 0; k < NUM_ENTRY; k = k + 1)
      cam_scheduler[k] = cam[k];
  end

  // 查找逻辑
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

  // 指针读取逻辑
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

  // FIFO实例
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
