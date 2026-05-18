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
    parameter NUM_PRIORITY = 3
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
  reg  [             2:0] write_pcp;
  reg                     ptr_write_nack;


  always @(posedge clk)
  begin
    if (reset)
    begin
      fmt_init_req[2:0] <= 3'b111;
    end
    else
    begin

      if (init_ack[0])
      begin
        fmt_init_req[0] <= 0;
      end
      if (init_ack[1])
      begin
        fmt_init_req[1] <= 0;
      end
      if (init_ack[2])
      begin
        fmt_init_req[2] <= 0;
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
  wire [3:0] cam_match_addr[NUM_PRIORITY-1:0];
  wire output_queue_empty;




  assign FMT_in_wr[0]   = (PCP <= 3) ? metadata_in_wr : 0;
  assign FMT_in_wr[1]   = (PCP == 4 | PCP == 5) ? metadata_in_wr : 0;
  assign FMT_in_wr[2]   = (PCP == 6 | PCP == 7) ? metadata_in_wr : 0;
  assign FMT_in_data[0] = (PCP <= 3) ? metadata_in : 0;
  assign FMT_in_data[1] = (PCP == 4 | PCP == 5) ? metadata_in : 0;
  assign FMT_in_data[2] = (PCP == 6 | PCP == 7) ? metadata_in : 0;
  assign FMT_flow_ID[0] = (PCP <= 3) ? flow_ID : 0;
  assign FMT_flow_ID[1] = (PCP == 4 | PCP == 5) ? flow_ID : 0;
  assign FMT_flow_ID[2] = (PCP == 6 | PCP == 7) ? flow_ID : 0;





  generate  //Cyclic treatment of different PCPs
    genvar p;
    for (p = 0; p < NUM_PRIORITY; p = p + 1)
    begin : FMT_Different_PCP
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
      write_pcp      <= 0;

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
          write_pcp      <= q_dout[22:20];
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
      fmt_search_ack[0] <= 0;
      fmt_search_ack[1] <= 0;
      fmt_search_ack[2] <= 0;
      fmt_tail_value[0] <= 0;
      fmt_tail_value[1] <= 0;
      fmt_tail_value[2] <= 0;
      fmt_head_wr_req[0] <= 0;
      fmt_head_wr_req[1] <= 0;
      fmt_head_wr_req[2] <= 0;
      fmt_head_value[0] <= 0;
      fmt_head_value[1] <= 0;
      fmt_head_value[2] <= 0;
      fmt_tail_wr_req[0] <= 0;
      fmt_tail_wr_req[1] <= 0;
      fmt_tail_wr_req[2] <= 0;
      flow_ram_addr[0] <= 'h0;
      flow_ram_wr[0] <= 0;
      flow_ram_din[0] <= 20'h0;
      flow_ram_addr[1] <= 'h0;
      flow_ram_wr[1] <= 0;
      flow_ram_din[1] <= 20'h0;
      flow_ram_addr[2] <= 'h0;
      flow_ram_wr[2] <= 0;
      flow_ram_din[2] <= 20'h0;
      cell_count[0] <= 0;
      cell_count[1] <= 0;
      cell_count[2] <= 0;
      fmt_write_busy <= 0;
      flow_tail_ptr[0] <= 0;
      flow_tail_ptr[1] <= 0;
      flow_tail_ptr[2] <= 0;
      flow_head_ptr[0] <= 0;
      flow_head_ptr[1] <= 0;
      flow_head_ptr[2] <= 0;
      depth_inc_flag[0] <= 0;
      depth_inc_flag[1] <= 0;
      depth_inc_flag[2] <= 0;
    end
    else
    begin
      ptr_write_ack <= 0;
      flow_ram_wr[0] <= 0;
      flow_ram_wr[1] <= 0;
      flow_ram_wr[2] <= 0;
      fmt_search_ack[0] <= 0;
      fmt_search_ack[1] <= 0;
      fmt_search_ack[2] <= 0;

      case (ptr_write_state)
        0:
        begin
          fmt_tail_wr_req[0] <= 0;
          fmt_tail_wr_req[1] <= 0;
          fmt_tail_wr_req[2] <= 0;
          fmt_tail_value[0] <= 0;
          fmt_tail_value[1] <= 0;
          fmt_tail_value[2] <= 0;

          if (ptr_write_req)
          begin
            if (write_pcp == 0 | 1 | 2 | 3)
            begin
              ptr_write_state <= read_flag[0] ? 0 : 3;
              ptr_write_nack <= read_flag[0] ? 1 : 0;
            end
            else if (write_pcp == 4 | 5)
            begin
              ptr_write_state <= read_flag[1] ? 0 : 3;
              ptr_write_nack <= read_flag[1] ? 1 : 0;
            end
            else if (write_pcp == 6 | 7)
            begin
              ptr_write_state <= read_flag[2] ? 0 : 3;
              ptr_write_nack <= read_flag[2] ? 1 : 0;
            end


            if (ptr_write_word[14])
            begin
              if (fmt_matched[2:0] != 0 & !ptr_write_nack)
              begin
                case (write_pcp)
                  'd0, 'd1, 'd2, 'd3:
                  begin
                    flow_tail_ptr[0] <= fmt_match_tail[0];
                    fmt_write_busy[0] <= 1;
                  end
                  'd4, 'd5:
                  begin
                    flow_tail_ptr[1] <= fmt_match_tail[1];
                    fmt_write_busy[1] <= 1;
                  end
                  'd6, 'd7:
                  begin
                    flow_tail_ptr[2] <= fmt_match_tail[2];
                    fmt_write_busy[2] <= 1;
                  end
                endcase
              end
              else if (fmt_mismatched[2:0] != 0 & !ptr_write_nack)
              begin
                case (write_pcp)
                  'd0, 'd1, 'd2, 'd3:
                  begin
                    flow_tail_ptr[0] <= ptr_write_word[19:0];
                    fmt_head_wr_req[0] <= 1;
                    fmt_head_value[0] <= ptr_write_word[19:0];
                    fmt_write_busy[0] <= 1;
                  end
                  'd4, 'd5:
                  begin
                    flow_tail_ptr[1] <= ptr_write_word[19:0];
                    fmt_head_wr_req[1] <= 1;
                    fmt_head_value[1] <= ptr_write_word[19:0];
                    fmt_write_busy[1] <= 1;
                  end
                  'd6, 'd7:
                  begin
                    flow_tail_ptr[2] <= ptr_write_word[19:0];
                    fmt_head_wr_req[2] <= 1;
                    fmt_head_value[2] <= ptr_write_word[19:0];
                    fmt_write_busy[2] <= 1;
                  end
                endcase
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
          fmt_head_wr_req[0] <= 0;
          fmt_head_wr_req[1] <= 0;
          fmt_head_wr_req[2] <= 0;
        end
        1:
        begin
          ptr_write_ack <= 1;
          ptr_write_state <= 2;
          case (write_pcp)
            0, 1, 2, 3:
            begin
              if (cell_count[0][9:0])
              begin
                flow_ram_wr[0] <= 1;
                flow_ram_addr[0][9:0] <= flow_tail_ptr[0][9:0];
                flow_ram_din[0][19:0] <= ptr_write_word[19:0];
                flow_tail_ptr[0] <= ptr_write_word;
              end
              else
              begin
                if (fmt_mismatched[0] == 1)
                begin
                  flow_ram_wr[0] <= 1;
                  flow_ram_addr[0][9:0] <= ptr_write_word[9:0];
                  flow_ram_din[0][19:0] <= ptr_write_word[19:0];
                  flow_tail_ptr[0] <= ptr_write_word;
                  flow_head_ptr[0] <= ptr_write_word;
                end
                else
                begin
                  flow_ram_wr[0] <= 1;
                  flow_ram_addr[0][9:0] <= flow_tail_ptr[0][9:0];
                  flow_ram_din[0][19:0] <= ptr_write_word[19:0];
                  flow_tail_ptr[0] <= ptr_write_word;
                end
              end
              cell_count[0] <= cell_count[0] + 1;
              if (ptr_write_word[15])
              begin
                depth_inc_flag[0] <= 1;
                cell_count[0] <= 0;
                fmt_write_busy[0] <= 0;
                if (fmt_mismatched[0])
                begin
                  fmt_tail_value[0] <= ptr_write_word;
                  fmt_tail_wr_req[0] <= 1;
                end
                else
                begin
                  fmt_search_ack[0] <= 1;
                  fmt_refresh_tail[0]  <= ptr_write_word;
                end
              end
            end
            4, 5:
            begin
              if (cell_count[1][9:0])
              begin
                flow_ram_wr[1] <= 1;
                flow_ram_addr[1][9:0] <= flow_tail_ptr[1][9:0];
                flow_ram_din[1][19:0] <= ptr_write_word[19:0];
                flow_tail_ptr[1] <= ptr_write_word;
              end
              else
              begin
                if (fmt_mismatched[1] == 1)
                begin
                  flow_ram_wr[1] <= 1;
                  flow_ram_addr[1][9:0] <= ptr_write_word[9:0];
                  flow_ram_din[1][19:0] <= ptr_write_word[19:0];
                  flow_tail_ptr[1] <= ptr_write_word;
                  flow_head_ptr[1] <= ptr_write_word;
                end
                else
                begin
                  flow_ram_wr[1] <= 1;
                  flow_ram_addr[1][9:0] <= flow_tail_ptr[1][9:0];
                  flow_ram_din[1][19:0] <= ptr_write_word[19:0];
                  flow_tail_ptr[1] <= ptr_write_word;
                end
              end
              cell_count[1] <= cell_count[1] + 1;
              if (ptr_write_word[15])
              begin
                depth_inc_flag[1] <= 1;
                cell_count[1] <= 0;
                fmt_write_busy[1] <= 0;
                if (fmt_mismatched[1])
                begin
                  fmt_tail_value[1] <= ptr_write_word;
                  fmt_tail_wr_req[1] <= 1;
                end
                else
                begin
                  fmt_search_ack[1] <= 1;
                  fmt_refresh_tail[1]  <= ptr_write_word;
                end
              end
            end


            6, 7:
            begin
              if (cell_count[2][9:0])
              begin
                flow_ram_wr[2] <= 1;
                flow_ram_addr[2][9:0] <= flow_tail_ptr[2][9:0];
                flow_ram_din[2][19:0] <= ptr_write_word[19:0];
                flow_tail_ptr[2] <= ptr_write_word;
              end
              else
              begin
                if (fmt_mismatched[2] == 1)
                begin
                  flow_ram_wr[2] <= 1;
                  flow_ram_addr[2][9:0] <= ptr_write_word[9:0];
                  flow_ram_din[2][19:0] <= ptr_write_word[19:0];
                  flow_tail_ptr[2] <= ptr_write_word;
                  flow_head_ptr[2] <= ptr_write_word;
                end
                else
                begin
                  flow_ram_wr[2] <= 1;
                  flow_ram_addr[2][9:0] <= flow_tail_ptr[2][9:0];
                  flow_ram_din[2][19:0] <= ptr_write_word[19:0];
                  flow_tail_ptr[2] <= ptr_write_word;
                end
              end
              cell_count[2] <= cell_count[2] + 1;
              if (ptr_write_word[15])
              begin
                depth_inc_flag[2] <= 1;
                fmt_write_busy[2] <= 0;
                cell_count[2] <= 0;
                if (fmt_mismatched[2])
                begin
                  fmt_tail_value[2] <= ptr_write_word;
                  fmt_tail_wr_req[2] <= 1;
                end
                else
                begin
                  fmt_search_ack[2] <= 1;
                  fmt_refresh_tail[2]  <= ptr_write_word;
                end
              end
            end
          endcase
        end

        2:
        begin
          ptr_write_state <= 0;
          case (write_pcp)
            0, 1, 2, 3:
            begin
              flow_ram_addr[0] <= flow_tail_ptr[0][9:0];
              flow_ram_din[0]  <= flow_tail_ptr[0][19:0];
              flow_ram_wr[0]   <= 1;
              // cam_wr_tail_req[0] <= 0;

            end
            4, 5:
            begin
              flow_ram_addr[1] <= flow_tail_ptr[1][9:0];
              flow_ram_din[1]  <= flow_tail_ptr[1][19:0];
              flow_ram_wr[1]   <= 1;
              // cam_wr_tail_req[1] <= 0;
            end
            6, 7:
            begin
              flow_ram_addr[2] <= flow_tail_ptr[2][9:0];
              flow_ram_din[2]  <= flow_tail_ptr[2][19:0];
              flow_ram_wr[2]   <= 1;
              // cam_wr_tail_req[2] <= 0;
            end
          endcase
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
  wire q0_flag;
  wire q1_flag;
  wire q2_flag;

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

  assign q0_flag = (pcp_queue_cnt[0] == 0) ? 0 : 1;
  assign q1_flag = (pcp_queue_cnt[1] == 0) ? 0 : 1;
  assign q2_flag = (pcp_queue_cnt[2] == 0) ? 0 : 1;



  priority_arbiter u_priority_arbiter (
                     .clk(clk),
                     .reset(reset),
                     .i_req_release(q0_flag | q1_flag | q2_flag),
                     .i_req_in({q0_flag, q1_flag, q2_flag}),
                     .o_grant_out({pcp_queue_ack[0], pcp_queue_ack[1], pcp_queue_ack[2]})
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
      if (pcp_queue_ack[0] == 1'b1 & !pcp_queue_empty[0])
      begin
        out_meta_word <= pcp_queue_dout[0][19:0];
        out_meta_wr <= 1'b1;
      end
      else if (pcp_queue_ack[1] == 1'b1 & !pcp_queue_empty[1])
      begin
        out_meta_word <= pcp_queue_dout[1][19:0];
        out_meta_wr <= 1'b1;
      end
      else if (pcp_queue_ack[2] == 1'b1 & !pcp_queue_empty[2])
      begin
        out_meta_word <= pcp_queue_dout[2][19:0];
        out_meta_wr <= 1'b1;
      end
      else
      begin
        out_meta_word <= 'b0;
        out_meta_wr <= 1'b0;
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
