# DFQ: Dynamic Per-Flow Queue Manager

An FPGA-based dynamic per-flow queue manager for TSN switches.

## Overview

Traditional switches usually rely on fixed priority queues, where different flows with the same priority share the same hardware queue. This makes per-flow isolation difficult.

The core idea of DFQ is:

- Use `flow_ID` as the key to maintain an independent virtual queue for each flow
- Use `FMT` to track the head pointer, tail pointer, and depth of each flow
- Use Pointer RAM to chain packet cells
- Map PCP to priority levels and arbitrate before output

This enables finer-grained queue management and scheduling in shared-buffer TSN designs.

## Repository Layout

This repository contains two implementations:

| Directory | Description |
|------|------|
| `DFQ_CAM/` | Flow table implementation based on CAM / sequential matching |
| `DFQ_Hash/` | Flow table implementation based on hash buckets |

Both versions include the following core modules:

| File | Description |
|------|------|
| `queue_manager.v` | Top-level module that connects enqueue, flow-table, dequeue, and arbitration logic |
| `FMT.v` | Flow Mapping Table for flow lookup, allocation, and updates |
| `dequeue_process.v` | Dequeue control logic |
| `priority_arbiter.v` | Priority arbitration logic |

## Architecture

```text
queue_manager
├─ FMT                 // Flow table: lookup and head/tail/depth maintenance
├─ Pointer RAM         // Linked-list pointer storage
├─ dequeue_process     // Dequeue control
└─ priority_arbiter    // Priority arbitration
```

## Priority Mapping

When `NUM_PRIORITY = 3`, the default PCP-to-priority mapping is:

| PCP | Priority |
|-----|----------|
| 0-3 | 0 |
| 4-5 | 1 |
| 6-7 | 2 |

## Top-Level Interface

### Enqueue

```verilog
input  [31:0] flow_ID
input  [ 2:0] PCP
input  [19:0] metadata_in
input         metadata_in_wr
output        q_full
```

### Dequeue

```verilog
output        ptr_rdy
input         metadata_out_rd
output [19:0] metadata_out
```

## Usage

1. Provide `clk` and `reset` to initialize the module
2. For enqueue, drive `flow_ID`, `PCP`, and `metadata_in`, then assert `metadata_in_wr`
3. When `ptr_rdy` is high, assert `metadata_out_rd` to read `metadata_out`

## Use Cases

- Shared-buffer management for TSN switches
- Per-flow isolation with ordered dequeue
- Parameterized queue management design for FPGA implementations

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
