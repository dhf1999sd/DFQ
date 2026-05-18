//////////////////////////////////////////////////////////////////////////////////
// Company:         LZU
// Engineer:        WenxuWu
// Create Date:     2026/05/07
// Module Name:     FMT
// Project Name:    DFQ_CAM_v6
// Target Devices:  ZYNQ-7000
// Tool Versions:   VIVADO2023.2
// Description:     CAM table management wrapper.
//                  Table update/search & dequeue scheduling are split into submodules.
//////////////////////////////////////////////////////////////////////////////////

`timescale 1ns / 1ps

module FMT #(
    parameter           NUM_ENTRY   = 16,
    parameter           ENTRY_WIDTH = 77
  )(
    input               clk,
    input               reset,
    input               init_req,
    output              init_ack,
    input               cam_wr_search,
    output              cam_matched,
    output              cam_mismatched,
    output [19:0]       cam_match_tail,
    output [3:0]        cam_match_addr,
    input  [19:0]       cam_refresh_tail,
    input               depth_flag,
    input               cam_wr_search_ack,
    input               cam_wr_head_req,
    input  [19:0]       cam_wr_head,
    input               cam_wr_tail_req,
    input  [19:0]       cam_wr_tail,
    input  [31:0]       flow_ID,
    output              ptr_read,
    output [19:0]       cam_read_head,
    output              read_mode_flag,
    input  [19:0]       cam_refresh_head,
    input               cam_refresh_head_flag
  );

  /***************function**************/

  /***************parameter*************/

  /***************port******************/

  /***************mechine***************/

  /***************reg*******************/

  /***************wire******************/
  wire [3:0]                         cam_read_addr;
  wire [NUM_ENTRY*ENTRY_WIDTH-1:0]   cam_snapshot;

  /***************component*************/
  FMT_table_manager #(
                      .NUM_ENTRY(NUM_ENTRY),
                      .ENTRY_WIDTH(ENTRY_WIDTH)
                    ) u_table_manager (
                      .clk                  (clk),
                      .reset                (reset),
                      .init_req             (init_req),
                      .init_ack             (init_ack),
                      .cam_wr_search        (cam_wr_search),
                      .cam_matched          (cam_matched),
                      .cam_mismatched       (cam_mismatched),
                      .cam_match_tail       (cam_match_tail),
                      .cam_match_addr       (cam_match_addr),
                      .cam_refresh_tail     (cam_refresh_tail),
                      .depth_flag           (depth_flag),
                      .cam_wr_search_ack    (cam_wr_search_ack),
                      .cam_wr_head_req      (cam_wr_head_req),
                      .cam_wr_head          (cam_wr_head),
                      .cam_wr_tail_req      (cam_wr_tail_req),
                      .cam_wr_tail          (cam_wr_tail),
                      .flow_ID              (flow_ID),
                      .cam_read_addr        (cam_read_addr),
                      .cam_refresh_head     (cam_refresh_head),
                      .cam_refresh_head_flag(cam_refresh_head_flag),
                      .cam_snapshot         (cam_snapshot)
                    );

  FMT_dequeue_scheduler #(
                          .NUM_ENTRY(NUM_ENTRY),
                          .ENTRY_WIDTH(ENTRY_WIDTH)
                        ) u_dequeue_scheduler (
                          .clk                  (clk),
                          .reset                (reset),
                          .cam_snapshot         (cam_snapshot),
                          .cam_refresh_head_flag(cam_refresh_head_flag),
                          .ptr_read             (ptr_read),
                          .cam_read_head        (cam_read_head),
                          .cam_read_addr        (cam_read_addr),
                          .read_mode_flag       (read_mode_flag)
                        );

  /***************assign****************/

  /***************always****************/

endmodule


//////////////////////////////////////////////////////////////////////////////////
// Company:         LZU
// Engineer:        WenxuWu
// Create Date:     2024/05/07
// Module Name:     FMT_table_manager
// Project Name:    DFQ_CAM_v5
// Target Devices:  ZYNQ-7000
// Tool Versions:   VIVADO2023.2
// Description:     CAM table maintenance: init, write, search, refresh, delete
//////////////////////////////////////////////////////////////////////////////////

module FMT_table_manager #(
    parameter           NUM_ENTRY   = 16,
    parameter           ENTRY_WIDTH = 77
  )(
    input                           clk,
    input                           reset,
    input                           init_req,
    output reg                      init_ack,
    input                           cam_wr_search,
    output reg                      cam_matched,
    output reg                      cam_mismatched,
    output reg [19:0]               cam_match_tail,
    output reg [3:0]                cam_match_addr,
    input      [19:0]               cam_refresh_tail,
    input                           depth_flag,
    input                           cam_wr_search_ack,
    input                           cam_wr_head_req,
    input      [19:0]               cam_wr_head,
    input                           cam_wr_tail_req,
    input      [19:0]               cam_wr_tail,
    input      [31:0]               flow_ID,
    input      [3:0]                cam_read_addr,
    input      [19:0]               cam_refresh_head,
    input                           cam_refresh_head_flag,
    output     [NUM_ENTRY*ENTRY_WIDTH-1:0] cam_snapshot
  );

  /***************function**************/

  /***************parameter*************/

  /***************port******************/

  /***************mechine***************/

  /***************reg*******************/
  reg  [3:0]              free_addr_fifo_din;
  reg                     free_addr_fifo_wr;
  reg                     free_addr_fifo_rd;
  reg  [ENTRY_WIDTH-1:0]  cam_entry [0:NUM_ENTRY-1];
  reg  [3:0]              fsm_state;
  reg  [5:0]              aging_idx;
  reg  [3:0]              match_entry_idx;
  reg                     match_entry_found;
  reg                     cam_matched_d;
  reg                     cam_mismatched_d;
  reg  [19:0]             cam_match_tail_d;
  reg  [3:0]              cam_match_addr_d;

  integer                 init_idx;
  integer                 search_idx;
  integer                 temp_idx;
  integer                 scan_idx;

  /***************wire******************/
  wire [3:0]              free_addr_fifo_dout;
  wire                    free_addr_fifo_empty;
  wire                    free_addr_fifo_full;

  /***************component*************/
  genvar                  c;
  generate
    for (c = 0; c < NUM_ENTRY; c = c + 1)
    begin : CAM_SNAPSHOT_PACK
      assign cam_snapshot[c*ENTRY_WIDTH +: ENTRY_WIDTH] = cam_entry[c];
    end
  endgenerate

  FMT_fifo_ft u_fifo_ft_w4_d16 (
                .clk       (clk),
                .rst       (reset),
                .din       (free_addr_fifo_din),
                .wr_en     (free_addr_fifo_wr),
                .rd_en     (free_addr_fifo_rd),
                .dout      (free_addr_fifo_dout),
                .full      (free_addr_fifo_full),
                .empty     (free_addr_fifo_empty),
                .data_count()
              );

  /***************assign****************/

  /***************always****************/
  always @(posedge clk)
  begin
    if (reset)
    begin
      fsm_state          <= 0;
      free_addr_fifo_din <= 0;
      free_addr_fifo_wr  <= 0;
      free_addr_fifo_rd  <= 0;
      init_ack          <= 0;
      aging_idx         <= 0;
      init_idx          <= 0;
      temp_idx          <= 0;
      match_entry_idx   <= 0;
      match_entry_found <= 0;
    end
    else
    begin
      case (fsm_state)
        0:
        begin
          free_addr_fifo_wr <= 0;
          free_addr_fifo_rd <= 0;

          if (init_req)
          begin
            init_idx  <= 0;
            fsm_state <= 4;
          end
          else if (cam_wr_head_req)
          begin
            if (!free_addr_fifo_empty)
            begin
              cam_entry[free_addr_fifo_dout][71:40] <= flow_ID;
              cam_entry[free_addr_fifo_dout][39:20] <= cam_wr_head;
              cam_entry[free_addr_fifo_dout][19:0]  <= cam_wr_head;
              cam_entry[free_addr_fifo_dout][75:72] <= 4'd0;
              cam_entry[free_addr_fifo_dout][76]    <= 1'b1;
              free_addr_fifo_rd                     <= 1'b1;
              fsm_state                              <= 1;
            end
          end
          else if (cam_wr_tail_req)
          begin
            match_entry_idx   = 4'd0;
            match_entry_found = 1'b0;

            for (scan_idx = 0; scan_idx < NUM_ENTRY; scan_idx = scan_idx + 1)
            begin
              if (!match_entry_found && (flow_ID == cam_entry[scan_idx][71:40]) && (cam_entry[scan_idx][76] == 1'b1))
              begin
                match_entry_found = 1'b1;
                match_entry_idx   = scan_idx[3:0];
              end
            end

            if (match_entry_found)
            begin
              cam_entry[match_entry_idx][19:0]  <= cam_wr_tail;
              cam_entry[match_entry_idx][75:72] <= depth_flag ? cam_entry[match_entry_idx][75:72] + 1'b1 : cam_entry[match_entry_idx][75:72];
              cam_entry[match_entry_idx][76]    <= 1'b1;
            end
            fsm_state <= 1;
          end
          else if (cam_wr_search_ack)
          begin
            match_entry_idx   = 4'd0;
            match_entry_found = 1'b0;

            for (scan_idx = 0; scan_idx < NUM_ENTRY; scan_idx = scan_idx + 1)
            begin
              if (!match_entry_found && (flow_ID == cam_entry[scan_idx][71:40]) && (cam_entry[scan_idx][76] == 1'b1))
              begin
                match_entry_found = 1'b1;
                match_entry_idx   = scan_idx[3:0];
              end
            end

            if (match_entry_found)
            begin
              cam_entry[match_entry_idx][19:0]  <= cam_refresh_tail;
              cam_entry[match_entry_idx][75:72] <= depth_flag ? cam_entry[match_entry_idx][75:72] + 1'b1 : cam_entry[match_entry_idx][75:72];
              cam_entry[match_entry_idx][76]    <= 1'b1;
            end
          end
          else if (cam_refresh_head_flag)
          begin
            cam_entry[cam_read_addr][39:20] <= cam_refresh_head;
            cam_entry[cam_read_addr][75:72] <= cam_entry[cam_read_addr][75:72] - 1'b1;

            if (cam_entry[cam_read_addr][75:72] - 1'b1 == 0)
            begin
              free_addr_fifo_din   <= cam_read_addr;
              free_addr_fifo_wr    <= 1'b1;
              cam_entry[cam_read_addr] <= {ENTRY_WIDTH{1'b0}};
            end
          end
        end

        1:
        begin
          free_addr_fifo_wr <= 0;
          free_addr_fifo_rd <= 0;
          fsm_state         <= 0;
        end

        2:
        begin
          free_addr_fifo_din <= init_idx[3:0];
          free_addr_fifo_wr  <= 1'b1;
          cam_entry[init_idx] <= {ENTRY_WIDTH{1'b0}};

          if (init_idx < NUM_ENTRY - 1)
            init_idx <= init_idx + 1;
          else
          begin
            init_ack <= 1'b1;
            fsm_state <= 3;
          end
        end

        3:
        begin
          free_addr_fifo_wr <= 0;
          init_ack         <= 0;
          fsm_state        <= 0;
        end

        4:
          fsm_state <= 5;
        5:
          fsm_state <= 6;
        6:
          fsm_state <= 7;
        7:
          fsm_state <= 8;
        8:
          fsm_state <= 9;
        9:
          fsm_state <= 10;
        10:
          fsm_state <= 11;
        11:
          fsm_state <= 12;
        12:
          fsm_state <= 2;

        default:
          fsm_state <= 0;
      endcase
    end
  end

  always @(posedge clk)
  begin
    if (reset)
    begin
      search_idx         <= 0;
      cam_matched        <= 0;
      cam_mismatched     <= 0;
      cam_match_tail     <= 0;
      cam_match_addr     <= 0;
      cam_matched_d      <= 0;
      cam_mismatched_d   <= 0;
      cam_match_tail_d   <= 0;
      cam_match_addr_d   <= 0;
    end
    else
    begin
      cam_matched      <= cam_matched_d;
      cam_mismatched   <= cam_mismatched_d;
      cam_match_tail   <= cam_match_tail_d;
      cam_match_addr   <= cam_match_addr_d;

      if (cam_wr_search)
      begin
        cam_matched_d    <= 0;
        cam_mismatched_d <= 1'b1;     
        for (search_idx = 0; search_idx < NUM_ENTRY; search_idx = search_idx + 1)
        begin
          if ((flow_ID == cam_entry[search_idx][71:40]) && (cam_entry[search_idx][76] == 1'b1))
          begin
            cam_matched_d    <= 1'b1;
            cam_mismatched_d <= 1'b0;
            cam_match_tail_d <= cam_entry[search_idx][19:0];
            cam_match_addr_d <= search_idx[3:0];
          end
        end
      end
      else if (cam_wr_search_ack|cam_wr_tail_req)
      begin
        cam_matched_d    <= 0;
        cam_mismatched_d <= 0;
      end
    end
  end

endmodule


//////////////////////////////////////////////////////////////////////////////////
// Company:         LZU
// Engineer:        WenxuWu
// Create Date:     2024/05/07
// Module Name:     FMT_dequeue_scheduler
// Project Name:    DFQ_CAM_v5
// Target Devices:  ZYNQ-7000
// Tool Versions:   VIVADO2023.2
// Description:     Dequeue scheduling for CAM: scan valid entries & issue read
//////////////////////////////////////////////////////////////////////////////////

module FMT_dequeue_scheduler #(
    parameter           NUM_ENTRY   = 16,
    parameter           ENTRY_WIDTH = 77
  )(
    input                         clk,
    input                         reset,
    input  [NUM_ENTRY*ENTRY_WIDTH-1:0] cam_snapshot,
    input                         cam_refresh_head_flag,
    output reg                    ptr_read,
    output reg [19:0]             cam_read_head,
    output reg [3:0]              cam_read_addr,
    output reg                    read_mode_flag
  );

  /***************function**************/

  /***************parameter*************/

  /***************port******************/

  /***************mechine***************/

  /***************reg*******************/
  reg [ENTRY_WIDTH-1:0] cam_snapshot_entry [0:NUM_ENTRY-1];
  integer              snapshot_idx;
  integer              scan_idx;

  /***************wire******************/

  /***************component*************/

  /***************assign****************/
  always @(*)
  begin
    for (snapshot_idx = 0; snapshot_idx < NUM_ENTRY; snapshot_idx = snapshot_idx + 1)
      cam_snapshot_entry[snapshot_idx] = cam_snapshot[snapshot_idx*ENTRY_WIDTH +: ENTRY_WIDTH];
  end

  /***************always****************/
  always @(posedge clk)
  begin
    if (reset)
    begin
      scan_idx       <= 0;
      ptr_read       <= 0;
      cam_read_head  <= 0;
      cam_read_addr  <= 0;
      read_mode_flag <= 0;
    end
    else
    begin
      for (scan_idx = 0; scan_idx < NUM_ENTRY; scan_idx = scan_idx + 1)
      begin
        if ((cam_snapshot_entry[scan_idx][75:72] > 4'd1) && !ptr_read)
        begin
          ptr_read       <= 1'b1;
          cam_read_head  <= cam_snapshot_entry[scan_idx][39:20];
          read_mode_flag <= cam_snapshot_entry[scan_idx][76];
          cam_read_addr  <= scan_idx[3:0];
        end
        else if (cam_refresh_head_flag)
        begin
          ptr_read <= 1'b0;
        end
      end
    end
  end

endmodule
