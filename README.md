# DFQ: Dynamic Per-Flow Queue Manager

FPGA-based dynamic flow queue manager for TSN (Time-Sensitive Networking) switches.

## Core Concept

Traditional switches use **static queues** (8 hardware queues), where all traffic with the same priority shares a queue, making it impossible to isolate different flows.

DFQ uses **dynamic per-flow queues**: each flow has its own virtual queue, maintained via a CAM table that stores head/tail pointers and depth, enabling flow isolation and ordered scheduling.

## Architecture

```
                    ┌─────────────────────────────┐
                    │      queue_manager.v        │  ← Top-level
                    │         (Top-level)         │
                    └─────────────────────────────┘
                                       │
         ┌─────────────────────────────┼─────────────────────────────┐
         │                             │                             │
         ▼                             ▼                             ▼
┌─────────────────┐          ┌─────────────────┐          ┌─────────────────┐
│    FMT.v        │          │  dequeue_process │          │ priority_arbiter│
│  (CAM Manager)  │          │      .v         │          │     .v          │
│                 │          │  (Dequeue FSM)  │          │                 │
│ • 32-entry CAM │          │                 │          │ Lowest-set-bit  │
│ • Flow search   │          │ • Read Pointer  │          │ Priority        │
│ • Head/Tail ptr │          │   RAM           │          │ Arbitration     │
└─────────────────┘          └─────────────────┘          └─────────────────┘
         │
         ▼
┌─────────────────┐
│   Pointer RAM   │  ← Linked-list pointers for each queue
└─────────────────┘
         │
         ▼
┌─────────────────┐
│  PCP Queues (3) │  ← Output queues grouped by priority
└─────────────────┘
```

## Priority Mapping

| PCP | Priority Queue |
|-----|----------------|
| 0-3 | Queue 0 (Highest) |
| 4-5 | Queue 1 |
| 6-7 | Queue 2 (Lowest) |

## Data Structures

### CAM Entry (77 bits)
| Bits | Width | Description |
|------|-------|-------------|
| [76] | 1 | Valid bit |
| [75:72] | 4 | Depth counter |
| [71:40] | 32 | Flow ID |
| [39:20] | 20 | Head pointer |
| [19:0] | 20 | Tail pointer |

### Metadata Special Bits
| Bit | Description |
|-----|-------------|
| [15] | Tail flag - indicates end of queue |
| [14] | Search control |

## Interfaces

### Enqueue
```verilog
input  [31:0] flow_ID       // Flow identifier
input  [ 2:0] PCP           // PCP priority
input  [19:0] metadata_in   // Metadata
input         metadata_in_wr // Enqueue write enable
output        q_full         // Queue full flag
```

### Dequeue
```verilog
output        ptr_rdy        // Pointer ready
input         metadata_out_rd // Dequeue read enable
output [19:0] metadata_out   // Metadata output
```

## Usage

1. **Initialization**: Provide `clk` and `reset` signals; the system auto-initializes the CAM table
2. **Enqueue**: Drive `flow_ID` + `PCP` + `metadata_in`, then assert `metadata_in_wr`
3. **Dequeue**: When `ptr_rdy` is high, assert `metadata_out_rd` to read `metadata_out`

## File Descriptions

| File | Description |
|------|-------------|
| queue_manager.v | Top-level module, connects all submodules |
| FMT.v | CAM table manager - search/write/refresh operations |
| dequeue_process.v | Dequeue FSM - pointer RAM traversal |
| priority_arbiter.v | Priority arbiter - lowest-set-bit selection |

## Reference

```bibtex
@article{10.1145/3718087,
  author    = {Wu, Wenxue and Zhang, Tong and Li, Zhen and Feng, Xiaoqin and Zhang, Liwei and Ren, Fengyuan},
  title     = {Dynamic Per-Flow Queues in Shared Buffer TSN Switches},
  year      = {2025},
  publisher = {Association for Computing Machinery},
  address   = {New York, NY, USA},
  volume    = {30},
  number    = {3},
  issn      = {1084-4309},
  doi       = {10.1145/3718087},
  journal   = {ACM Trans. Des. Autom. Electron. Syst.},
  month     = mar,
  articleno = {38},
  numpages  = {21}
}
```
