`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company:         LZU
// Engineer:        WenxuWu
// Create Date:     2024/05/07
// Module Name:     queue_manager
// Project Name:    queue_manager
// Target Devices:  ZYNQ-7000
// Tool Versions:   VIVADO2023.2
// Description:     Multi-priority queue manager with CAM-based flow table,
//                  supporting PCP-based traffic classification and arbitration
//////////////////////////////////////////////////////////////////////////////////
module queue_manager #(
    parameter NUM_PRIORITY = 1
  ) (
    input         clk,
    input         reset,
    input  [31:0] flow_ID,
    input  [ 2:0] PCP,
    input  [19:0] metadata_in,
    input         metadata_in_wr,
    output        ptr_rdy,
    input         metadata_out_rd,
    output [19:0] metadata_out,
    output        q_full
  );

  /***************function**************/

  /***************parameter*************/

  /***************port******************/

  /***************mechine***************/

  /***************reg*******************/

  /***************wire******************/

  /***************component*************/

  /***************assign****************/

  /***************always****************/

  localparam PRIORITY_WIDTH = (NUM_PRIORITY <= 1) ? 1 : $clog2(NUM_PRIORITY);

  function [PRIORITY_WIDTH-1:0] pcp_to_priority;
    input [2:0] pcp;
    integer scaled_priority;
    begin
      if (NUM_PRIORITY <= 1)
      begin
        pcp_to_priority = {PRIORITY_WIDTH{1'b0}};
      end
      else if (NUM_PRIORITY == 3)
      begin
        if (pcp <= 3'd3)
          pcp_to_priority = 0;
        else if (pcp <= 3'd5)
          pcp_to_priority = 1;
        else
          pcp_to_priority = 2;
      end
      else if (NUM_PRIORITY >= 8)
      begin
        pcp_to_priority = pcp;
      end
      else
      begin
        scaled_priority = (pcp * NUM_PRIORITY) >> 3;
        if (scaled_priority >= NUM_PRIORITY)
          pcp_to_priority = NUM_PRIORITY - 1;
        else
          pcp_to_priority = scaled_priority;
      end
    end
  endfunction

  reg  [NUM_PRIORITY-1:0] fmt_init_req;


  wire [             19:0] fmt_match_tail        [NUM_PRIORITY-1:0];
  reg                     ptr_write_ack;
  wire [NUM_PRIORITY-1:0] pcp_queue_wr;
  reg  [NUM_PRIORITY-1:0] flow_ram_wr;
  reg                     ptr_write_req;
  reg  [             3:0] ptr_write_state;
  reg  [            19:0] flow_tail_ptr          [NUM_PRIORITY-1:0];
  reg  [            19:0] flow_head_ptr          [NUM_PRIORITY-1:0];
  reg  [            22:0] ptr_write_word;
  reg  [            19:0] flow_ram_din           [NUM_PRIORITY-1:0];
  wire [            19:0] flow_ram_dout          [NUM_PRIORITY-1:0];
  reg  [             9:0] flow_ram_addr          [NUM_PRIORITY-1:0];
  wire [             9:0] ptr_ram_rd_addr       [NUM_PRIORITY-1:0];
  wire [            19:0] pcp_queue_din         [NUM_PRIORITY-1:0];
  reg  [             9:0] cell_count             [NUM_PRIORITY-1:0];
  reg                     depth_inc_flag         [NUM_PRIORITY-1:0];
  reg                     in_fifo_rd_en;
  wire [NUM_PRIORITY-1:0] fmt_matched;
  wire [NUM_PRIORITY-1:0] fmt_mismatched;
  reg  [             1:0] in_fifo_state;
  wire [             5:0] pcp_queue_cnt         [NUM_PRIORITY-1:0];
  wire [            22:0] q_dout;
  wire                    q_empty;
  reg  [NUM_PRIORITY-1:0] fmt_search_ack;
  reg  [             19:0] fmt_refresh_tail      [NUM_PRIORITY-1:0];
  reg  [NUM_PRIORITY-1:0] fmt_head_wr_req;
  reg  [NUM_PRIORITY-1:0] fmt_tail_wr_req;
  reg  [             19:0] fmt_tail_value        [NUM_PRIORITY-1:0];
  reg  [             19:0] fmt_head_value        [NUM_PRIORITY-1:0];
  wire [NUM_PRIORITY-1:0] init_ack;
  wire [NUM_PRIORITY-1:0] fmt_ptr_read;
  wire [             19:0] fmt_read_head         [NUM_PRIORITY-1:0];
  wire [            19:0] pcp_queue_dout        [NUM_PRIORITY-1:0];
  reg  [NUM_PRIORITY-1:0] fmt_write_busy;
  wire [NUM_PRIORITY-1:0] read_flag;
  wire [NUM_PRIORITY-1:0] pcp_queue_ack;
  reg  [             3:0] dequeue_state          [NUM_PRIORITY-1:0];
  reg  [            19:0] dequeue_head           [NUM_PRIORITY-1:0];
  wire [             19:0] fmt_refresh_head      [NUM_PRIORITY-1:0];
  wire [NUM_PRIORITY-1:0] fmt_refresh_head_flag;
  wire [NUM_PRIORITY-1:0] fmt_read_mode;
  reg  [PRIORITY_WIDTH-1:0] write_priority;
  reg                     ptr_write_nack;
  reg  [NUM_PRIORITY-1:0] FMT_in_wr_pipe;
  reg  [            19:0] FMT_in_data_pipe      [NUM_PRIORITY-1:0];
  reg  [            31:0] FMT_flow_ID_pipe      [NUM_PRIORITY-1:0];
  wire [PRIORITY_WIDTH-1:0] metadata_priority;
  wire [PRIORITY_WIDTH-1:0] q_priority;
  wire [NUM_PRIORITY-1:0] pcp_queue_valid;
  wire [NUM_PRIORITY-1:0] arb_req_in;
  wire [NUM_PRIORITY-1:0] arb_grant_out;

  integer init_req_idx;
  integer pipe_idx;
  integer ptr_idx;
  integer out_idx;

  assign metadata_priority = pcp_to_priority(PCP);
  assign q_priority        = pcp_to_priority(q_dout[22:20]);


  always @(posedge clk)
  begin
    if (reset)
    begin
      fmt_init_req <= {NUM_PRIORITY{1'b1}};
    end
    else
    begin
      for (init_req_idx = 0; init_req_idx < NUM_PRIORITY; init_req_idx = init_req_idx + 1)
      begin
        if (init_ack[init_req_idx])
          fmt_init_req[init_req_idx] <= 1'b0;
      end
    end

  end

  always @(posedge clk)
  begin
    if (reset)
    begin
      FMT_in_wr_pipe      <= {NUM_PRIORITY{1'b0}};
      for (pipe_idx = 0; pipe_idx < NUM_PRIORITY; pipe_idx = pipe_idx + 1)
      begin
        FMT_in_data_pipe[pipe_idx] <= 20'd0;
        FMT_flow_ID_pipe[pipe_idx] <= 32'd0;
      end
    end
    else
    begin
      FMT_in_wr_pipe <= {NUM_PRIORITY{1'b0}};
      if (metadata_in_wr)
      begin
        FMT_in_wr_pipe[metadata_priority]   <= 1'b1;
        FMT_in_data_pipe[metadata_priority] <= metadata_in;
        FMT_flow_ID_pipe[metadata_priority] <= flow_ID;
      end
    end
  end


  fifo_d64_in_queue_port u_ptr_wr_fifo (
                           .clk(clk),
                           .rst(reset),
                           .din({PCP[2:0], metadata_in[19:0]}),
                           .wr_en(metadata_in_wr),
                           .rd_en(in_fifo_rd_en),
                           .dout(q_dout),
                           .full(q_full),
                           .empty(q_empty),
                           .data_count()
                         );


  wire [NUM_PRIORITY-1:0] FMT_in_wr;
  wire [19:0] FMT_in_data[NUM_PRIORITY-1:0];
  wire [31:0] FMT_flow_ID[NUM_PRIORITY-1:0];
  wire [6:0] cam_match_addr[NUM_PRIORITY-1:0];
  wire output_queue_empty;




  generate  //Cyclic treatment of different PCPs
    genvar p;
    for (p = 0; p < NUM_PRIORITY; p = p + 1)
    begin : FMT
      assign FMT_in_wr[p]   = FMT_in_wr_pipe[p];
      assign FMT_in_data[p] = FMT_in_data_pipe[p];
      assign FMT_flow_ID[p] = FMT_flow_ID_pipe[p];

      FMT u_CAM_FMT (
            .clk(clk),
            .reset(reset),
            .init_req(fmt_init_req[p]),
            .init_ack(init_ack[p]),
            .depth_flag(depth_inc_flag[p]),
            .cam_wr_search(FMT_in_wr[p] & FMT_in_data[p][14]),  //
            .cam_matched(fmt_matched[p]),
            .cam_mismatched(fmt_mismatched[p]),
            .cam_match_tail(fmt_match_tail[p]),
            .cam_match_addr(cam_match_addr[p]),
            .cam_refresh_tail(fmt_refresh_tail[p]),
            .cam_wr_search_ack(fmt_search_ack[p]),
            .cam_wr_head_req(fmt_head_wr_req[p]),
            .cam_wr_tail_req(fmt_tail_wr_req[p]),
            .cam_wr_tail(fmt_tail_value[p]),
            .cam_wr_head(fmt_head_value[p]),
            .flow_ID(FMT_flow_ID[p]),
            .ptr_read(fmt_ptr_read[p]),
            .cam_read_head(fmt_read_head[p]),
            .cam_refresh_head(fmt_refresh_head[p]),
            .read_mode_flag(fmt_read_mode[p]),
            .cam_refresh_head_flag(fmt_refresh_head_flag[p])
          );

    end  // block: in_arb_queues
  endgenerate


  always @(posedge clk)
    if (reset)
    begin
      ptr_write_word <= 0;
      ptr_write_req  <= 0;
      in_fifo_rd_en  <= 0;
      in_fifo_state  <= 0;
      write_priority <= 0;

    end
    else
    begin
      case (in_fifo_state)
        0:
        begin
          if (!q_empty)
          begin
            in_fifo_rd_en <= 1;
            in_fifo_state <= 1;
          end
        end
        1:
        begin
          in_fifo_rd_en <= 0;
          in_fifo_state <= 2;
        end
        2:
        begin
          ptr_write_word <= q_dout;
          ptr_write_req  <= 1;
          in_fifo_state  <= 3;
          write_priority <= q_priority;
        end
        3:
        begin
          if (ptr_write_nack)
          begin
            ptr_write_req <= 0;
            in_fifo_state <= 2;
          end
          else if (ptr_write_ack)
          begin
            ptr_write_req <= 0;
            in_fifo_state <= 0;
          end
        end
      endcase
    end

  always @(posedge clk)
    if (reset)
    begin
      ptr_write_ack     <= 0;
      ptr_write_state   <= 0;
      ptr_write_nack    <= 0;
      fmt_search_ack   <= {NUM_PRIORITY{1'b0}};
      fmt_head_wr_req  <= {NUM_PRIORITY{1'b0}};
      fmt_tail_wr_req  <= {NUM_PRIORITY{1'b0}};
      fmt_write_busy   <= {NUM_PRIORITY{1'b0}};
      for (ptr_idx = 0; ptr_idx < NUM_PRIORITY; ptr_idx = ptr_idx + 1)
      begin
        fmt_tail_value[ptr_idx]   <= 20'd0;
        fmt_head_value[ptr_idx]   <= 20'd0;
        flow_ram_addr[ptr_idx]    <= 10'd0;
        flow_ram_wr[ptr_idx]      <= 1'b0;
        flow_ram_din[ptr_idx]     <= 20'd0;
        cell_count[ptr_idx]       <= 10'd0;
        flow_tail_ptr[ptr_idx]    <= 20'd0;
        flow_head_ptr[ptr_idx]    <= 20'd0;
        depth_inc_flag[ptr_idx]   <= 1'b0;
        fmt_refresh_tail[ptr_idx] <= 20'd0;
      end
    end
    else
    begin
      ptr_write_ack <= 0;
      flow_ram_wr   <= {NUM_PRIORITY{1'b0}};
      fmt_search_ack <= {NUM_PRIORITY{1'b0}};

      case (ptr_write_state)
        0:
        begin
          fmt_tail_wr_req <= {NUM_PRIORITY{1'b0}};
          for (ptr_idx = 0; ptr_idx < NUM_PRIORITY; ptr_idx = ptr_idx + 1)
            fmt_tail_value[ptr_idx] <= 20'd0;

          if (ptr_write_req)
          begin
            ptr_write_state <= read_flag[write_priority] ? 0 : 3;
            ptr_write_nack  <= read_flag[write_priority] ? 1'b1 : 1'b0;


            if (ptr_write_word[14])
            begin
              if ((|fmt_matched) && !read_flag[write_priority])
              begin
                flow_tail_ptr[write_priority] <= fmt_match_tail[write_priority];
                fmt_write_busy[write_priority] <= 1'b1;
              end
              else if ((|fmt_mismatched) && !read_flag[write_priority])
              begin
                flow_tail_ptr[write_priority]  <= ptr_write_word[19:0];
                fmt_head_wr_req[write_priority] <= 1'b1;
                fmt_head_value[write_priority]  <= ptr_write_word[19:0];
                fmt_write_busy[write_priority]  <= 1'b1;
              end
            end
          end
        end
        3:
        begin
          ptr_write_state <= 4;
        end
        4:
        begin
          ptr_write_state <= 1;
          fmt_head_wr_req <= {NUM_PRIORITY{1'b0}};
        end
        1:
        begin
          ptr_write_ack <= 1;
          ptr_write_state <= 2;
          if (cell_count[write_priority][9:0])
          begin
            flow_ram_wr[write_priority]           <= 1'b1;
            flow_ram_addr[write_priority][9:0]    <= flow_tail_ptr[write_priority][9:0];
            flow_ram_din[write_priority][19:0]    <= ptr_write_word[19:0];
            flow_tail_ptr[write_priority]         <= ptr_write_word[19:0];
          end
          else
          begin
            if (fmt_mismatched[write_priority] == 1'b1)
            begin
              flow_ram_wr[write_priority]        <= 1'b1;
              flow_ram_addr[write_priority][9:0] <= ptr_write_word[9:0];
              flow_ram_din[write_priority][19:0] <= ptr_write_word[19:0];
              flow_tail_ptr[write_priority]      <= ptr_write_word[19:0];
              flow_head_ptr[write_priority]      <= ptr_write_word[19:0];
            end
            else
            begin
              flow_ram_wr[write_priority]        <= 1'b1;
              flow_ram_addr[write_priority][9:0] <= flow_tail_ptr[write_priority][9:0];
              flow_ram_din[write_priority][19:0] <= ptr_write_word[19:0];
              flow_tail_ptr[write_priority]      <= ptr_write_word[19:0];
            end
          end

          cell_count[write_priority] <= cell_count[write_priority] + 1'b1;

          if (ptr_write_word[15])
          begin
            depth_inc_flag[write_priority] <= 1'b1;
            fmt_write_busy[write_priority] <= 1'b0;
            cell_count[write_priority]     <= 10'd0;
            if (fmt_mismatched[write_priority])
            begin
              fmt_tail_value[write_priority]  <= ptr_write_word[19:0];
              fmt_tail_wr_req[write_priority] <= 1'b1;
            end
            else
            begin
              fmt_search_ack[write_priority]   <= 1'b1;
              fmt_refresh_tail[write_priority] <= ptr_write_word[19:0];
            end
          end
        end

        2:
        begin
          ptr_write_state <= 0;
          flow_ram_addr[write_priority] <= flow_tail_ptr[write_priority][9:0];
          flow_ram_din[write_priority]  <= flow_tail_ptr[write_priority][19:0];
          flow_ram_wr[write_priority]   <= 1'b1;
        end
      endcase

    end


  generate  //
    genvar j;
    for (j = 0; j < NUM_PRIORITY; j = j + 1)
    begin : dequeue_state_machine

      dequeue_process u_dequeue_process (
                        .clk(clk),
                        .reset(reset),
                        .read_flag(read_flag[j]),
                        .ptr_read(fmt_ptr_read[j]),
                        .write_flag(fmt_write_busy[j]),
                        .cam_read_head(fmt_read_head[j]),
                        .pcp_queue_din(pcp_queue_din[j]),
                        .read_mode_flag(fmt_read_mode[j]),
                        .ptr_ram_dout(flow_ram_dout[j]),
                        .pcp_queue_wr(pcp_queue_wr[j]),
                        .ptr_ram_rd_addr(ptr_ram_rd_addr[j]),
                        .cam_refresh_head(fmt_refresh_head[j]),
                        .cam_refresh_head_flag(fmt_refresh_head_flag[j])
                      );
    end  //
  endgenerate













  wire [NUM_PRIORITY-1:0] pcp_queue_full;
  wire [NUM_PRIORITY-1:0] pcp_queue_empty;

  generate  //
    genvar q;
    for (q = 0; q < NUM_PRIORITY; q = q + 1)
    begin : FMT_Different_RAM
      sram_FMT u_flow_ram (
                 .clka (clk),                   // input wire clka
                 .wea  (flow_ram_wr[q]),         // input wire [0 : 0] wea
                 .addra(flow_ram_addr[q][8:0]),  // input wire [7 : 0] addra
                 .dina (flow_ram_din[q]),        // input wire [19 : 0] dina
                 .clkb (clk),                   // input wire clkb
                 .addrb(ptr_ram_rd_addr[q]),    // input wire [7 : 0] addrb
                 .doutb(flow_ram_dout[q])        // output wire [19 : 0] doutb
               );
    end  // block: in_arb_queues
  endgenerate



  generate  //
    genvar y;
    for (y = 0; y < NUM_PRIORITY; y = y + 1)
    begin : PRIORITY_QUEUE
      fifo_ft_w16_d64 u_PRIORITY_queue (
                        .clk(clk),
                        .rst(reset),
                        .din(pcp_queue_din[y]),
                        .wr_en(pcp_queue_wr[y]),
                        .rd_en(pcp_queue_ack[y]),
                        .dout(pcp_queue_dout[y]),
                        .full(pcp_queue_full[y]),
                        .empty(pcp_queue_empty[y]),
                        .data_count(pcp_queue_cnt[y])
                      );

    end  // block: PRIORITY_QUEUE
  endgenerate

  generate
    genvar arb_idx;
    for (arb_idx = 0; arb_idx < NUM_PRIORITY; arb_idx = arb_idx + 1)
    begin : PRIORITY_ARB_MAP
      assign pcp_queue_valid[arb_idx] = (pcp_queue_cnt[arb_idx] != 0);
      assign arb_req_in[NUM_PRIORITY-1-arb_idx] = pcp_queue_valid[arb_idx];
      assign pcp_queue_ack[arb_idx] = arb_grant_out[NUM_PRIORITY-1-arb_idx];
    end
  endgenerate



  priority_arbiter #(
                     .P_CHANEL_NUM(NUM_PRIORITY)
                   ) u_priority_arbiter (
                     .clk(clk),
                     .reset(reset),
                     .i_req_release(|pcp_queue_valid),
                     .i_req_in(arb_req_in),
                     .o_grant_out(arb_grant_out)
                   );


  reg [19:0] out_meta_word;
  reg        out_meta_wr;


  always @(posedge clk)
  begin
    if (reset == 1'b1)
    begin
      out_meta_word <= 20'd0;
      out_meta_wr <= 'b0;
    end
    else
    begin
      out_meta_word <= 20'd0;
      out_meta_wr   <= 1'b0;
      for (out_idx = 0; out_idx < NUM_PRIORITY; out_idx = out_idx + 1)
      begin
        if (pcp_queue_ack[out_idx] == 1'b1 && !pcp_queue_empty[out_idx])
        begin
          out_meta_word <= pcp_queue_dout[out_idx][19:0];
          out_meta_wr   <= 1'b1;
        end
      end
    end
  end




  fifo_output_w20 u_output (
                    .clk       (clk),                 // input wire clk
                    .rst       (reset),               // input wire rst
                    .din       (out_meta_word[19:0]),     // input wire [19 : 0] din
                    .wr_en     (out_meta_wr),        // input wire wr_en
                    .rd_en     (metadata_out_rd),     // input wire rd_en
                    .dout      (metadata_out[19:0]),  // output wire [19 : 0] dout
                    .full      (),                    // output wire full
                    .empty     (output_queue_empty),  // output wire empty
                    .data_count()                     // output wire [5 : 0] data_count
                  );


  assign ptr_rdy = !output_queue_empty;






endmodule
