`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company:
// Engineer:
//
// Create Date: 2024/05/07 21:23:04
// Design Name:
// Module Name: queue_manager
// Project Name:
// Target Devices:
// Tool Versions:
// Description:
//
// Dependencies:
//
// Revision:
// Revision 0.01 - File Created
// Additional Comments:
//
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

  reg  [NUM_PRIORITY-1:0] init_req;


  wire [             19:0] cam_match_tail        [NUM_PRIORITY-1:0];
  reg                     ptr_wr_ack;
  wire [NUM_PRIORITY-1:0] pcp_queue_wr;
  reg  [NUM_PRIORITY-1:0] ptr_ram_wr;
  reg                     ptr_wr;
  reg  [             3:0] flow_wr_mstate;
  reg  [            19:0] tail                  [NUM_PRIORITY-1:0];
  reg  [            19:0] head                  [NUM_PRIORITY-1:0];
  reg  [            22:0] ptr_din;
  reg  [            19:0] ptr_ram_din           [NUM_PRIORITY-1:0];
  wire [            19:0] ptr_ram_dout          [NUM_PRIORITY-1:0];
  reg  [             9:0] ptr_ram_addr          [NUM_PRIORITY-1:0];
  wire [             9:0] ptr_ram_rd_addr       [NUM_PRIORITY-1:0];
  wire [            19:0] pcp_queue_din         [NUM_PRIORITY-1:0];
  reg  [             9:0] depth_cell            [NUM_PRIORITY-1:0];
  reg                     depth_flag            [NUM_PRIORITY-1:0];









  reg                     q_rd;
  wire [NUM_PRIORITY-1:0] cam_matched;
  wire [NUM_PRIORITY-1:0] cam_mismatched;

  reg  [             1:0] qc_wr_state;
  wire [             5:0] pcp_queue_cnt         [NUM_PRIORITY-1:0];
  wire [            22:0] q_dout;
  wire                    q_empty;

  reg  [NUM_PRIORITY-1:0] cam_wr_search_ack;
  reg  [             19:0] cam_refresh_tail      [NUM_PRIORITY-1:0];
  reg  [NUM_PRIORITY-1:0] cam_wr_head_req;
  reg  [NUM_PRIORITY-1:0] cam_wr_tail_req;
  reg  [             19:0] cam_wr_tail           [NUM_PRIORITY-1:0];
  reg  [             19:0] cam_wr_head           [NUM_PRIORITY-1:0];

  wire [NUM_PRIORITY-1:0] init_ack;
  wire [NUM_PRIORITY-1:0] ptr_read;

  wire [             19:0] cam_read_head         [NUM_PRIORITY-1:0];


  wire [            19:0] pcp_queue_dout        [NUM_PRIORITY-1:0];
  reg  [NUM_PRIORITY-1:0] write_flag;
  wire [NUM_PRIORITY-1:0] read_flag;
  wire [NUM_PRIORITY-1:0] pcp_queue_ack;

  reg  [             3:0] flow_queue_rd_state   [NUM_PRIORITY-1:0];
  reg  [            19:0] rd_head               [NUM_PRIORITY-1:0];

  wire [             19:0] cam_refresh_head      [NUM_PRIORITY-1:0];
  wire [NUM_PRIORITY-1:0] cam_refresh_head_flag;
  wire [NUM_PRIORITY-1:0] read_mode_flag;
  reg  [             2:0] wr_PCP;
  reg                     ptr_wr_nack;


  always @(posedge clk)
  begin
    if (reset)
    begin
      init_req[2:0] <= 3'b111;
    end
    else
    begin

      if (init_ack[0])
      begin
        init_req[0] <= 0;
      end
      if (init_ack[1])
      begin
        init_req[1] <= 0;
      end
      if (init_ack[2])
      begin
        init_req[2] <= 0;
      end

    end

  end


  fifo_d64_in_queue_port u_ptr_wr_fifo (
                           .clk(clk),
                           .rst(reset),
                           .din({PCP[2:0], metadata_in[19:0]}),
                           .wr_en(metadata_in_wr),
                           .rd_en(q_rd),
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
            .init_req(init_req[p]),
            .init_ack(init_ack[p]),
            .depth_flag(depth_flag[p]),
            .cam_wr_search(FMT_in_wr[p] & FMT_in_data[p][14]),  //
            .cam_matched(cam_matched[p]),
            .cam_mismatched(cam_mismatched[p]),
            .cam_match_tail(cam_match_tail[p]),
            .cam_match_addr(cam_match_addr[p]),
            .cam_refresh_tail(cam_refresh_tail[p]),
            .cam_wr_search_ack(cam_wr_search_ack[p]),
            .cam_wr_head_req(cam_wr_head_req[p]),
            .cam_wr_tail_req(cam_wr_tail_req[p]),
            .cam_wr_tail(cam_wr_tail[p]),
            .cam_wr_head(cam_wr_head[p]),
            .flow_ID(FMT_flow_ID[p]),
            .ptr_read(ptr_read[p]),
            .cam_read_head(cam_read_head[p]),
            .cam_refresh_head(cam_refresh_head[p]),
            .read_mode_flag(read_mode_flag[p]),
            .cam_refresh_head_flag(cam_refresh_head_flag[p])
          );

    end  // block: in_arb_queues
  endgenerate


  always @(posedge clk)
    if (reset)
    begin
      ptr_din <= 0;
      ptr_wr <= 0;
      q_rd <= 0;
      qc_wr_state <= 0;
      wr_PCP <= 0;

    end
    else
    begin
      case (qc_wr_state)
        0:
        begin
          if (!q_empty)
          begin
            q_rd <= 1;
            qc_wr_state <= 1;
          end
        end
        1:
        begin
          q_rd <= 0;
          qc_wr_state <= 2;
        end
        2:
        begin
          ptr_din <= q_dout;
          ptr_wr <= 1;
          qc_wr_state <= 3;
          wr_PCP <= q_dout[22:20];
        end
        3:
        begin
          if (ptr_wr_nack)
          begin
            ptr_wr <= 0;
            qc_wr_state <= 2;
          end
          else if (ptr_wr_ack)
          begin
            ptr_wr <= 0;
            qc_wr_state <= 0;
          end
        end
      endcase
    end

  always @(posedge clk)
    if (reset)
    begin
      ptr_wr_ack <= 0;
      flow_wr_mstate <= 0;
      ptr_wr_nack <= 0;
      cam_wr_search_ack[0] <= 0;
      cam_wr_search_ack[1] <= 0;
      cam_wr_search_ack[2] <= 0;
      cam_wr_tail[0] <= 0;
      cam_wr_tail[1] <= 0;
      cam_wr_tail[2] <= 0;
      cam_wr_head_req[0] <= 0;
      cam_wr_head_req[1] <= 0;
      cam_wr_head_req[2] <= 0;
      cam_wr_head[0] <= 0;
      cam_wr_head[1] <= 0;
      cam_wr_head[2] <= 0;
      cam_wr_tail_req[0] <= 0;
      cam_wr_tail_req[1] <= 0;
      cam_wr_tail_req[2] <= 0;
      ptr_ram_addr[0] <= 'h0;
      ptr_ram_wr[0] <= 0;
      ptr_ram_din[0] <= 20'h0;
      ptr_ram_addr[1] <= 'h0;
      ptr_ram_wr[1] <= 0;
      ptr_ram_din[1] <= 20'h0;
      ptr_ram_addr[2] <= 'h0;
      ptr_ram_wr[2] <= 0;
      ptr_ram_din[2] <= 20'h0;
      depth_cell[0] <= 0;
      depth_cell[1] <= 0;
      depth_cell[2] <= 0;
      write_flag <= 0;
      tail[0] <= 0;
      tail[1] <= 0;
      tail[2] <= 0;
      head[0] <= 0;
      head[1] <= 0;
      head[2] <= 0;
      depth_flag[0] <= 0;
      depth_flag[1] <= 0;
      depth_flag[2] <= 0;
    end
    else
    begin
      ptr_wr_ack <= 0;
      ptr_ram_wr[0] <= 0;
      ptr_ram_wr[1] <= 0;
      ptr_ram_wr[2] <= 0;

      cam_wr_search_ack[0] <= 0;
      cam_wr_search_ack[1] <= 0;
      cam_wr_search_ack[2] <= 0;

      case (flow_wr_mstate)
        0:
        begin
          cam_wr_tail_req[0] <= 0;
          cam_wr_tail_req[1] <= 0;
          cam_wr_tail_req[2] <= 0;
          cam_wr_tail[0] <= 0;
          cam_wr_tail[1] <= 0;
          cam_wr_tail[2] <= 0;
    
          if (ptr_wr)
          begin
            if (wr_PCP == 0 | 1 | 2 | 3)
            begin
              flow_wr_mstate <= read_flag[0] ? 0 : 3;
              ptr_wr_nack <= read_flag[0] ? 1 : 0;
            end
            else if (wr_PCP == 4 | 5)
            begin
              flow_wr_mstate <= read_flag[1] ? 0 : 3;
              ptr_wr_nack <= read_flag[1] ? 1 : 0;
            end
            else if (wr_PCP == 6 | 7)
            begin
              flow_wr_mstate <= read_flag[2] ? 0 : 3;
              ptr_wr_nack <= read_flag[2] ? 1 : 0;
            end


            if (ptr_din[14])
            begin
              if (cam_matched[2:0] != 0 & !ptr_wr_nack)
              begin
                case (wr_PCP)
                  'd0, 'd1, 'd2, 'd3:
                  begin
                    tail[0] <= cam_match_tail[0];
                    write_flag[0] <= 1;
                  end
                  'd4, 'd5:
                  begin
                    tail[1] <= cam_match_tail[1];
                    write_flag[1] <= 1;
                  end
                  'd6, 'd7:
                  begin
                    tail[2] <= cam_match_tail[2];
                    write_flag[2] <= 1;
                  end
                endcase
              end
              else if (cam_mismatched[2:0] != 0 & !ptr_wr_nack)
              begin
                case (wr_PCP)
                  'd0, 'd1, 'd2, 'd3:
                  begin
                    tail[0] <= ptr_din[19:0];
                    cam_wr_head_req[0] <= 1;
                    cam_wr_head[0] <= ptr_din[19:0];
                    write_flag[0] <= 1;
                  end
                  'd4, 'd5:
                  begin
                    tail[1] <= ptr_din[19:0];
                    cam_wr_head_req[1] <= 1;
                    cam_wr_head[1] <= ptr_din[19:0];
                    write_flag[1] <= 1;
                  end
                  'd6, 'd7:
                  begin
                    tail[2] <= ptr_din[19:0];
                    cam_wr_head_req[2] <= 1;
                    cam_wr_head[2] <= ptr_din[19:0];
                    write_flag[2] <= 1;
                  end
                endcase
              end
            end
          end
        end
        3:
        begin
          flow_wr_mstate <= 4;
        end
        4:
        begin
          flow_wr_mstate <= 1;
          cam_wr_head_req[0] <= 0;
          cam_wr_head_req[1] <= 0;
          cam_wr_head_req[2] <= 0;
        end
        1:
        begin
          ptr_wr_ack <= 1;
          flow_wr_mstate <= 2;
          case (wr_PCP)
            0, 1, 2, 3:
            begin
              if (depth_cell[0][9:0])
              begin
                ptr_ram_wr[0] <= 1;
                ptr_ram_addr[0][9:0] <= tail[0][9:0];
                ptr_ram_din[0][19:0] <= ptr_din[19:0];
                tail[0] <= ptr_din;
              end
              else
              begin
                if (cam_mismatched[0] == 1)
                begin
                  ptr_ram_wr[0] <= 1;
                  ptr_ram_addr[0][9:0] <= ptr_din[9:0];
                  ptr_ram_din[0][19:0] <= ptr_din[19:0];
                  tail[0] <= ptr_din;
                  head[0] <= ptr_din;
                end
                else
                begin
                  ptr_ram_wr[0] <= 1;
                  ptr_ram_addr[0][9:0] <= tail[0][9:0];
                  ptr_ram_din[0][19:0] <= ptr_din[19:0];
                  tail[0] <= ptr_din;
                end
              end
              depth_cell[0] <= depth_cell[0] + 1;
              if (ptr_din[15])
              begin
                depth_flag[0] <= 1;
                depth_cell[0] <= 0;
                write_flag[0] <= 0;
                if (cam_mismatched[0])
                begin
                  cam_wr_tail[0] <= ptr_din;
                  cam_wr_tail_req[0] <= 1;
                end
                else
                begin
                  cam_wr_search_ack[0] <= 1;
                  cam_refresh_tail[0]  <= ptr_din;
                end
              end
            end
            4, 5:
            begin
              if (depth_cell[1][9:0])
              begin
                ptr_ram_wr[1] <= 1;
                ptr_ram_addr[1][9:0] <= tail[1][9:0];
                ptr_ram_din[1][19:0] <= ptr_din[19:0];
                tail[1] <= ptr_din;
              end
              else
              begin
                if (cam_mismatched[1] == 1)
                begin
                  ptr_ram_wr[1] <= 1;
                  ptr_ram_addr[1][9:0] <= ptr_din[9:0];
                  ptr_ram_din[1][19:0] <= ptr_din[19:0];
                  tail[1] <= ptr_din;
                  head[1] <= ptr_din;
                end
                else
                begin
                  ptr_ram_wr[1] <= 1;
                  ptr_ram_addr[1][9:0] <= tail[1][9:0];
                  ptr_ram_din[1][19:0] <= ptr_din[19:0];
                  tail[1] <= ptr_din;
                end
              end
              depth_cell[1] <= depth_cell[1] + 1;
              if (ptr_din[15])
              begin
                depth_flag[1] <= 1;
                depth_cell[1] <= 0;
                write_flag[1] <= 0;
                if (cam_mismatched[1])
                begin
                  cam_wr_tail[1] <= ptr_din;
                  cam_wr_tail_req[1] <= 1;
                end
                else
                begin
                  cam_wr_search_ack[1] <= 1;
                  cam_refresh_tail[1]  <= ptr_din;
                end
              end
            end


            6, 7:
            begin
              if (depth_cell[2][9:0])
              begin
                ptr_ram_wr[2] <= 1;
                ptr_ram_addr[2][9:0] <= tail[2][9:0];
                ptr_ram_din[2][19:0] <= ptr_din[19:0];
                tail[2] <= ptr_din;
              end
              else
              begin
                if (cam_mismatched[2] == 1)
                begin
                  ptr_ram_wr[2] <= 1;
                  ptr_ram_addr[2][9:0] <= ptr_din[9:0];
                  ptr_ram_din[2][19:0] <= ptr_din[19:0];
                  tail[2] <= ptr_din;
                  head[2] <= ptr_din;
                end
                else
                begin
                  ptr_ram_wr[2] <= 1;
                  ptr_ram_addr[2][9:0] <= tail[2][9:0];
                  ptr_ram_din[2][19:0] <= ptr_din[19:0];
                  tail[2] <= ptr_din;
                end
              end
              depth_cell[2] <= depth_cell[2] + 1;
              if (ptr_din[15])
              begin
                depth_flag[2] <= 1;
                write_flag[2] <= 0;
                depth_cell[2] <= 0;
                if (cam_mismatched[2])
                begin
                  cam_wr_tail[2] <= ptr_din;
                  cam_wr_tail_req[2] <= 1;
                end
                else
                begin
                  cam_wr_search_ack[2] <= 1;
                  cam_refresh_tail[2]  <= ptr_din;
                end
              end
            end
          endcase
        end

        2:
        begin
          flow_wr_mstate <= 0;
          case (wr_PCP)
            0, 1, 2, 3:
            begin
              ptr_ram_addr[0] <= tail[0][9:0];
              ptr_ram_din[0]  <= tail[0][19:0];
              ptr_ram_wr[0]   <= 1;
              // cam_wr_tail_req[0] <= 0;

            end
            4, 5:
            begin
              ptr_ram_addr[1] <= tail[1][9:0];
              ptr_ram_din[1]  <= tail[1][19:0];
              ptr_ram_wr[1]   <= 1;
              // cam_wr_tail_req[1] <= 0;
            end
            6, 7:
            begin
              ptr_ram_addr[2] <= tail[2][9:0];
              ptr_ram_din[2]  <= tail[2][19:0];
              ptr_ram_wr[2]   <= 1;
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
                        .ptr_read(ptr_read[j]),
                        .write_flag(write_flag[j]),
                        .cam_read_head(cam_read_head[j]),
                        .pcp_queue_din(pcp_queue_din[j]),
                        .read_mode_flag(read_mode_flag[j]),
                        .ptr_ram_dout(ptr_ram_dout[j]),
                        .pcp_queue_wr(pcp_queue_wr[j]),
                        .ptr_ram_rd_addr(ptr_ram_rd_addr[j]),
                        .cam_refresh_head(cam_refresh_head[j]),
                        .cam_refresh_head_flag(cam_refresh_head_flag[j])
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
                 .wea  (ptr_ram_wr[q]),         // input wire [0 : 0] wea
                 .addra(ptr_ram_addr[q][8:0]),  // input wire [7 : 0] addra
                 .dina (ptr_ram_din[q]),        // input wire [19 : 0] dina
                 .clkb (clk),                   // input wire clkb
                 .addrb(ptr_ram_rd_addr[q]),    // input wire [7 : 0] addrb
                 .doutb(ptr_ram_dout[q])        // output wire [19 : 0] doutb
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


  reg [19:0] out_mb_md;
  reg        out_mb_md_wr;


  always @(posedge clk)
  begin
    if (reset == 1'b1)
    begin
      out_mb_md <= 20'd0;
      out_mb_md_wr <= 'b0;
    end
    else
    begin
      if (pcp_queue_ack[0] == 1'b1 & !pcp_queue_empty[0])
      begin
        out_mb_md <= pcp_queue_dout[0][19:0];
        out_mb_md_wr <= 1'b1;
      end
      else if (pcp_queue_ack[1] == 1'b1 & !pcp_queue_empty[1])
      begin
        out_mb_md <= pcp_queue_dout[1][19:0];
        out_mb_md_wr <= 1'b1;
      end
      else if (pcp_queue_ack[2] == 1'b1 & !pcp_queue_empty[2])
      begin
        out_mb_md <= pcp_queue_dout[2][19:0];
        out_mb_md_wr <= 1'b1;
      end
      else
      begin
        out_mb_md <= 'b0;
        out_mb_md_wr <= 1'b0;
      end
    end
  end




  fifo_output_w20 u_output (
                    .clk       (clk),                 // input wire clk
                    .rst       (reset),               // input wire rst
                    .din       (out_mb_md[19:0]),     // input wire [19 : 0] din
                    .wr_en     (out_mb_md_wr),        // input wire wr_en
                    .rd_en     (metadata_out_rd),     // input wire rd_en
                    .dout      (metadata_out[19:0]),  // output wire [19 : 0] dout
                    .full      (),                    // output wire full
                    .empty     (output_queue_empty),  // output wire empty
                    .data_count()                     // output wire [5 : 0] data_count
                  );


  assign ptr_rdy = !output_queue_empty;






endmodule
