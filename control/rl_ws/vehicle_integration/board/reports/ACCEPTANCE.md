# Board Preparation Acceptance

Generated: 2026-10-06T03:33:16.760649+00:00

## Result

- Communication and complete network RTL simulation: PASS.
- Independent synthesis, placement, routing and 50 MHz bitstream: PASS.
- Physical FPGA programming and inference: NOT PERFORMED / NOT VERIFIED.
- Real-vehicle balance: NOT VERIFIED.
- Motor and servo operation: NOT PERFORMED. All 12 actuator outputs are constant zero.

## Evidence

- Checkpoint SHA256: `29b12e353b84380fa57ed7c3f8928fb1d51e9df99c0fa01bd3f4aec2de5a8c03`.
- Routed Fmax: 67.546 MHz, 20.000 ns requested clock; setup/hold gate passed.
- Exactly 26 physical pads verified against independent CST.
- 30 host tests, 14 successful complete UART inference comparisons, all 8 stress samples.
- Per successful inference: all 425 layers, 3 latent, 6 raw and 6 clipped actions checked.
- Core latency: 81998 clocks, 1.63996 ms at 50 MHz, excluding input loading and UART.
- Host separately decoded nominal and saturated RTL captures exactly.
- Diagnostic UART line time: 160.590 ms at 115200 baud, not a 125 Hz control interface.
- ROM initialization and inferred memory/DSP evidence are in the build report.

Bitstream: `/home/cyd/fpga/rl_ws2/vehicle_integration/board/gowin/runs/20261006T033158.298291Z/fc_v4_board/impl/pnr/fc_v4_board.fs`

Bitstream SHA256: `3a8a187ddabaf0496557378e626bd5ea58e0c85722c385731fe100fa550174d8`

Build run: `/home/cyd/fpga/rl_ws2/vehicle_integration/board/gowin/runs/20261006T033158.298291Z`

`acceptance.json` contains routed timing, resources, warning/coverage details and
SHA256 identities for source files, evidence and artifacts.

## Isolation And Hardware Boundary

Core/ROM identities match the original baseline. The unrelated live vehicle
project changed externally after the original snapshot; its original and current
hashes are preserved in `baseline.json`, verifier reports and `acceptance.json`.
It was not restored, overwritten or compiled into this bitstream. Do not claim
that it remained historically unchanged.

No programmer or serial device was opened by this preparation. FTDI is only a
candidate download cable; the CH9102 board UART and target identity are unverified.
Later programming requires separate authorization, verified board/cable identity,
physical bank2 voltage verification, actuator power isolation and mechanical
support. Removing servo pulses can release loads, and configuration-time pad
levels are not guaranteed by constant-zero RTL. Flash is separately unauthorized.
