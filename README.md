# DFQ: Dynamic Per-Flow Queue Manager

- Overview: Flow-oriented queue management using a CAM table to maintain per-flow head/tail pointers and depth. `PCP` maps to three priority queues, and an arbiter selects outputs.
- Use cases: High-throughput data paths requiring flow- and priority-ordered enqueue/dequeue, such as network devices or on-chip networks.

## Files

- `queue_manager.v`: Top-level queue manager. Accepts `flow_ID`, `PCP`, and `metadata_in`; drives CAM (`FMT.v`), pointer RAM, priority queues, and the arbiter. Produces `metadata_out` and `ptr_rdy`.
- `FMT.v`: CAM manager. Maintains per-flow head/tail pointers and depth; supports init, search, write, and refresh.
- `dequeue_process.v`: Dequeue FSM. Reads from pointer RAM using the head pointer, pushes into the corresponding priority queue, and refreshes the CAM head.
- `priority_arbiter.v`: Priority arbiter with lowest-set-bit priority selection.

## Quick Start

- Add the `.v` files and required IPs to your project; connect system `clk` and synchronous `reset`.
- Enqueue: assert `metadata_in_wr` with `metadata_in`. `PCP` maps to three priorities (0–3 → priority 0, 4–5 → priority 1, 6–7 → priority 2).
- Dequeue: when `ptr_rdy` is high, assert `metadata_out_rd` to read `metadata_out`.

## Notes

- Metadata bits `[15]` and `[14]` indicate tail and search control; `FMT.v` and `dequeue_process.v` implement logic based on this convention.
