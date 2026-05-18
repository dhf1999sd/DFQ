//////////////////////////////////////////////////////////////////////////////////
// Company:         LZU
// Engineer:        WenxuWu
// Create Date:     2026/05/15
// Module Name:     dequeue_process
// Project Name:    DFQ_CAM_v6
// Target Devices:  ZYNQ-7000
// Tool Versions:   VIVADO2023.2
// Description:     Dequeue process module for DFQ CAM system with FSM control
//                  Handles reading from pointer RAM and managing queue operations
//////////////////////////////////////////////////////////////////////////////////

`timescale 1ns / 1ps

module dequeue_process #(
    parameter           DATA_WIDTH = 20,
    parameter           ADDR_WIDTH = 10
)(
    // Clock and Reset
    input               clk,
    input               reset,

    // Control Signals
    input               ptr_read,
    input               write_flag,
    input               read_mode_flag,
    output reg          read_flag,

    // CAM Interface
    input  [DATA_WIDTH-1:0]   cam_read_head,
    output reg [DATA_WIDTH-1:0] cam_refresh_head,
    output reg                 cam_refresh_head_flag,

    // Pointer RAM Interface
    input  [DATA_WIDTH-1:0]   ptr_ram_dout,
    output reg [ADDR_WIDTH-1:0] ptr_ram_rd_addr,

    // PCP Queue Interface
    output reg [DATA_WIDTH-1:0] pcp_queue_din,
    output reg                 pcp_queue_wr
);

/***************function**************/

/***************parameter*************/

// State Machine Definition
localparam [3:0] ST_IDLE         = 4'd0;
localparam [3:0] ST_START        = 4'd1;
localparam [3:0] ST_CHECK        = 4'd2;
localparam [3:0] ST_READ         = 4'd3;
localparam [3:0] ST_PUSH         = 4'd4;
localparam [3:0] ST_PUSH_LOOP    = 4'd5;
localparam [3:0] ST_PUSH_DONE    = 4'd6;
localparam [3:0] ST_REFRESH      = 4'd7;
localparam [3:0] ST_REFRESH_DONE = 4'd8;
localparam [3:0] ST_EXIT         = 4'd9;
localparam [3:0] ST_CAM_REFRESH  = 4'd10;
localparam [3:0] ST_CAM_WAIT     = 4'd11;
localparam [3:0] ST_FINAL        = 4'd12;
localparam [3:0] ST_EXIT2        = 4'd13;
localparam [3:0] ST_NEXT         = 4'd14;
localparam [3:0] ST_WAIT         = 4'd15;

/***************port******************/

/***************mechine***************/

/***************reg*******************/
reg [3:0]           state_q;
reg [DATA_WIDTH-1:0] rd_head_q;
reg                 pushed_tail_q;

/***************wire******************/
wire                is_tail_entry;
wire                is_ptr_tail;

/***************component*************/

/***************assign****************/
assign is_tail_entry = cam_read_head[15] && cam_read_head[14];
assign is_ptr_tail   = ptr_ram_dout[15];

/***************always****************/
always @(posedge clk or posedge reset)
begin
    if(reset)
    begin
        state_q               <= ST_IDLE;
        read_flag             <= 1'b0;
        pcp_queue_wr          <= 1'b0;
        ptr_ram_rd_addr       <= {ADDR_WIDTH{1'b0}};
        pcp_queue_din         <= {DATA_WIDTH{1'b0}};
        cam_refresh_head_flag <= 1'b0;
        cam_refresh_head      <= {DATA_WIDTH{1'b0}};
        rd_head_q             <= {DATA_WIDTH{1'b0}};
        pushed_tail_q         <= 1'b0;
    end
    else
    begin
        // Default values
        pcp_queue_wr          <= 1'b0;
        cam_refresh_head_flag <= 1'b0;

        case(state_q)
            ST_IDLE:
            begin
                if(ptr_read && !write_flag)
                begin
                    state_q <= ST_START;
                    read_flag <= 1'b1;
                end
            end

            ST_START:
            begin
                pcp_queue_din     <= cam_read_head;
                pcp_queue_wr      <= 1'b1;
                ptr_ram_rd_addr   <= cam_read_head[ADDR_WIDTH-1:0];
                state_q     <= ST_CHECK;
            end

            ST_CHECK:
            begin
                if(is_tail_entry)
                    state_q <= ST_EXIT2;
                else
                    state_q <= ST_READ;
            end

            ST_READ:
            begin
                state_q <= ST_PUSH;
            end

            ST_PUSH:
            begin
                pcp_queue_din     <= ptr_ram_dout;
                pcp_queue_wr      <= 1'b1;
                ptr_ram_rd_addr   <= ptr_ram_dout[ADDR_WIDTH-1:0];
                rd_head_q         <= ptr_ram_dout;
                pushed_tail_q     <= is_ptr_tail;
                state_q     <= ST_PUSH_LOOP;
            end

            ST_PUSH_LOOP:
            begin
                state_q <= ST_PUSH_DONE;
            end

            ST_PUSH_DONE:
            begin
                state_q <= ST_REFRESH;
            end

            ST_REFRESH:
            begin
                state_q <= ST_REFRESH_DONE;
            end

            ST_REFRESH_DONE:
            begin
                if(pushed_tail_q)
                begin
                    state_q     <= ST_EXIT;
                end
                else
                begin
                    state_q <= ST_PUSH;
                end
            end

            ST_EXIT:
            begin
                cam_refresh_head_flag <= 1'b0;
                state_q <= ST_CAM_REFRESH;
            end

            ST_CAM_REFRESH:
            begin
                state_q <= ST_CAM_WAIT;
            end

            ST_CAM_WAIT:
            begin
                cam_refresh_head      <= ptr_ram_dout;
                cam_refresh_head_flag <= 1'b1;
                state_q         <= ST_FINAL;
            end

            ST_FINAL:
            begin
                read_flag     <= 1'b0;
                state_q <= ST_IDLE;
            end

            ST_EXIT2:
            begin
                state_q <= ST_NEXT;
            end

            ST_NEXT:
            begin
                cam_refresh_head      <= ptr_ram_dout;
                cam_refresh_head_flag <= 1'b1;
                state_q         <= ST_WAIT;
            end

            ST_WAIT:
            begin
                cam_refresh_head_flag <= 1'b0;
                cam_refresh_head      <= {DATA_WIDTH{1'b0}};
                read_flag             <= 1'b0;
                state_q         <= ST_IDLE;
            end

            default:
            begin
                read_flag             <= 1'b0;
                pcp_queue_wr          <= 1'b0;
                cam_refresh_head_flag <= 1'b0;
                state_q         <= ST_IDLE;
            end
        endcase
    end
end

endmodule
