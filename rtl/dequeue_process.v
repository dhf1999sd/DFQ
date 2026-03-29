`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company:         NNS@TSN
// Engineer:        Wenxue Wu
// Create Date:     2024/05/15
// Module Name:     dequeue_process
// Project Name:    Dynamic per flow queues
// Target Devices:  Zynq
// Tool Versions:   VIVADO 2023.2
// Description:     Dequeue process with FSM control for pointer RAM and CAM refresh
//////////////////////////////////////////////////////////////////////////////////

module dequeue_process #(
    parameter DATA_WIDTH = 20,
    parameter ADDR_WIDTH = 10
  ) (
    // Clock and Reset
    input  wire                     clk,
    input  wire                     reset,

    // Control Signals
    input  wire                     ptr_read,
    input  wire                     write_flag,
    input  wire                     read_mode_flag,
    output reg                      read_flag,

    // CAM Interface
    input  wire [DATA_WIDTH-1:0]    cam_read_head,
    output reg  [DATA_WIDTH-1:0]    cam_refresh_head,
    output reg                      cam_refresh_head_flag,

    // Pointer RAM Interface
    input  wire [DATA_WIDTH-1:0]    ptr_ram_dout,
    output reg  [ADDR_WIDTH-1:0]    ptr_ram_rd_addr,

    // PCP Queue Interface
    output reg  [DATA_WIDTH-1:0]    pcp_queue_din,
    output reg                      pcp_queue_wr
  );

  //=============================================================================
  // State Machine Definition - States ordered by execution flow
  //=============================================================================
  localparam [3:0] ST_IDLE         = 4'd0;  // Initial state
  localparam [3:0] ST_START        = 4'd1;  // Start processing
  localparam [3:0] ST_CHECK        = 4'd2;  // Check if tail entry
  localparam [3:0] ST_READ         = 4'd3;  // Read operation (non-tail path)
  localparam [3:0] ST_PUSH         = 4'd4;  // Push data to queue
  localparam [3:0] ST_PUSH_LOOP    = 4'd5;  // Push loop wait
  localparam [3:0] ST_PUSH_DONE    = 4'd6;  // Push done
  localparam [3:0] ST_REFRESH      = 4'd7;  // Refresh operation
  localparam [3:0] ST_REFRESH_DONE = 4'd8;  // Refresh done, check if ptr tail
  localparam [3:0] ST_EXIT         = 4'd9;  // Exit from non-tail path
  localparam [3:0] ST_CAM_REFRESH  = 4'd10; // CAM refresh operation
  localparam [3:0] ST_CAM_WAIT     = 4'd11; // CAM wait state
  localparam [3:0] ST_FINAL        = 4'd12; // Final state before return to idle
  localparam [3:0] ST_EXIT2        = 4'd13; // Exit from tail path
  localparam [3:0] ST_NEXT         = 4'd14; // Next operation (tail path)
  localparam [3:0] ST_WAIT         = 4'd15; // Wait state (tail path)

  reg [3:0] current_state;

  //=============================================================================
  // Internal Registers and Signals
  //=============================================================================
  reg [DATA_WIDTH-1:0] rd_head_reg;

  // Flag to check if current entry is 64B (both bit 15 and 14 are set)
  wire is_tail_entry = cam_read_head[15] && cam_read_head[14];

  // Flag to check if current pointer entry is tail (bit 15 is set)
  wire is_ptr_tail = ptr_ram_dout[15];

  //=============================================================================
  // One-Stage State Machine - Sequential Logic with Output Logic
  //=============================================================================
  always @(posedge clk or posedge reset)
  begin
    if (reset)
    begin
      current_state         <= ST_IDLE;
      read_flag             <= 1'b0;
      pcp_queue_wr          <= 1'b0;
      ptr_ram_rd_addr       <= {ADDR_WIDTH{1'b0}};
      pcp_queue_din         <= {DATA_WIDTH{1'b0}};
      cam_refresh_head_flag <= 1'b0;
      cam_refresh_head      <= {DATA_WIDTH{1'b0}};
      rd_head_reg           <= {DATA_WIDTH{1'b0}};
    end
    else
    begin
      // Default values - these signals are pulsed for one clock cycle
      pcp_queue_wr          <= 1'b0;
      cam_refresh_head_flag <= 1'b0;

      case (current_state)
        ST_IDLE:
        begin
          if (ptr_read && !write_flag)
          begin
            current_state <= ST_START;
            read_flag <= 1'b1;
          end
        end

        ST_START:
        begin
          pcp_queue_din <= cam_read_head;
          pcp_queue_wr  <= 1'b1;
          ptr_ram_rd_addr <= cam_read_head[ADDR_WIDTH-1:0];
          current_state <= ST_CHECK;
        end

        ST_CHECK:
        begin
          if (is_tail_entry)
          begin
            current_state <= ST_EXIT2;
          end
          else
          begin
            current_state <= ST_READ;
          end
        end

        ST_READ:
        begin
          current_state <= ST_PUSH;
        end

        ST_PUSH:
        begin
          pcp_queue_din   <= ptr_ram_dout;
          pcp_queue_wr    <= 1'b1;
          ptr_ram_rd_addr <= ptr_ram_dout[ADDR_WIDTH-1:0];
          current_state <= ST_PUSH_LOOP;
        end

        ST_PUSH_LOOP:
        begin
          current_state <= ST_PUSH_DONE;
        end

        ST_PUSH_DONE:
        begin
          current_state <= ST_REFRESH;
        end

        ST_REFRESH:
        begin
          current_state <= ST_REFRESH_DONE;
        end

        ST_REFRESH_DONE:
        begin
          if (is_ptr_tail)
          begin
            pcp_queue_din   <= ptr_ram_dout;
            ptr_ram_rd_addr <= ptr_ram_dout[ADDR_WIDTH-1:0];
            pcp_queue_wr    <= 1'b1;
            current_state <= ST_EXIT;
          end
          else
          begin
            current_state <= ST_PUSH;
          end
        end

        ST_EXIT:
        begin
          cam_refresh_head_flag <= 1'b0;
          current_state <= ST_CAM_REFRESH;
        end

        ST_CAM_REFRESH:
        begin
          current_state <= ST_CAM_WAIT;
        end

        ST_CAM_WAIT:
        begin
          cam_refresh_head      <= ptr_ram_dout;
          cam_refresh_head_flag <= 1'b1;
          current_state <= ST_FINAL;
        end

        ST_FINAL:
        begin
          read_flag <= 1'b0;
          current_state <= ST_IDLE;
        end

        ST_EXIT2:
        begin
          current_state <= ST_NEXT;
        end

        ST_NEXT:
        begin
          cam_refresh_head      <= ptr_ram_dout;
          cam_refresh_head_flag <= 1'b1;
          current_state <= ST_WAIT;
        end

        ST_WAIT:
        begin
          cam_refresh_head_flag <= 1'b0;
          cam_refresh_head      <= {DATA_WIDTH{1'b0}};
          current_state <= ST_IDLE;
        end

        default:
        begin
          // Reset to default values
          read_flag             <= 1'b0;
          pcp_queue_wr          <= 1'b0;
          cam_refresh_head_flag <= 1'b0;
          current_state <= ST_IDLE;
        end
      endcase
    end
  end

endmodule
