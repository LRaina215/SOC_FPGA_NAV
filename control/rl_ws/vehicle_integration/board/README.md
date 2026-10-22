# Network-only board deployment

This directory adds a physical-board wrapper around the unchanged full_center_v4
encoder/actor inference core. The target is GW5AT-LV138PG484AC1/I0 with a 50 MHz
clock. The purpose is replaying known inputs and reading exact results, not
controlling a vehicle.

## Hardware boundary

No FPGA programming is performed by the build, simulation or host unit tests.
The wrapper has no actuator-enable command. Both motor-enable outputs, all six
motor PWM outputs and all four servo outputs must remain zero in every state,
including startup, reset, malformed communication and inference.

Logic-low outputs do not establish mechanical safety: removing servo pulses can
release a loaded mechanism. Before any later authorized download, isolate actuator
power and mechanically support the robot; confirm the connected board and cable.
Do not use the existing competition main.v for network replay: it automatically
runs the second motor and sweeps the servos.

A UART command cannot transition this wrapper into vehicle-control mode. Real
sensor adapters, actuator mapping and physical balance are separate work.

## Data contract

The host sends 27 current observation plus 135 history signed24 F12 words. History
is oldest to newest. The UART transport uses 115200 baud, 8N1 and CRC16; it is a
single-request/response diagnostic link. Input serialization and output transfer
are not part of the 81998-clock inference latency. At this baud rate a full request
and response occupy roughly 161 ms of wire time, so this is not the 8 ms control
interface. See PROTOCOL.md for byte layouts and error handling.

Responses identify the pinned checkpoint and expose all 425 layer values, latent,
six clipped actions, core errors, saturation, frame number and inference cycles.
The checkpoint identifier is a fixed build identifier, not cryptographic
attestation. Use source/ROM/bitstream hashes to identify the tested artifact.

## Delivery layout

- rtl/: UART receiver/transmitter, packet wrapper and fixed-disabled board top.
- tb/ and run_rtl.sh: serial end-to-end, arithmetic identity, malformed-packet,
  reset/recovery and output-disable tests.
- host/: packet codec, explicit-port replay client and mock/unit tests.
- gowin/ and build_board.sh: independent physical implementation project.
- PIN_VERIFICATION.md: pin directions, electrical rationale and source evidence.
- reports/: serial simulation, host conformance, implementation and connection evidence.
- baseline.json: protected inference/ROM and live-project identities before integration.

## Acceptance levels

1. Communication and complete network RTL simulation.
2. Successful physical implementation and timing checks, with a generated bitstream.
3. Real-board packet replay with all outputs matching the fixed reference.
4. Real-vehicle balance.

These are distinct levels. A bitstream file does not prove a board has been
programmed. Mock serial tests do not prove a USB connection. No level is marked
passed without its own evidence. The original offline acceptance remains in the
parent reports directory; board reports do not replace it.

## Reproduction

Run the complete build-only acceptance pipeline from any working directory:

```bash
bash /home/cyd/fpga/pt_to_fpga/full_center_v4_deploy_20261005/board/verify_board.sh
```

This reruns the host tests, board UART RTL, offline packet comparisons, independent
Gowin synthesis/place-and-route/bitstream generation and evidence summary. It does
not open a serial device or invoke a programmer. A failure exits nonzero; an old
bitstream or synthesis-only success does not pass the acceptance gate.

The latest accepted preparation report is `reports/ACCEPTANCE.md`, with detailed
source/evidence/artifact hashes in `reports/acceptance.json`. The latest build is
identified by `gowin/build_status.json`; per-run logs and artifacts remain under
`gowin/runs/<UTC timestamp>/`. The `.fs` path and SHA256 are in those reports.
Always select an accepted run and verify the file identity, not an arbitrary old
`.fs` elsewhere on the machine.

Host-only preflight without hardware:

```bash
python3 /home/cyd/fpga/pt_to_fpga/full_center_v4_deploy_20261005/board/host/client.py --dry-run --samples 3 --no-report
```

`PROTOCOL.md` documents explicit-port future replay and offline capture checks.
The full network response contains 425 layer words, three latent outputs and six
clipped actions. UART replay is a diagnostic test, not a vehicle control loop.

## Isolation Evidence

The inference RTL and weight/bias/ELU memories must match the immutable original
`baseline.json`; changes are fatal. The unrelated live vehicle `main.v` and CST
changed externally after that snapshot. Their original and current identities
are retained as drift evidence, not restored or rebaselined. The independent
build does not compile either live file, and its relevant clock/actuator mappings
are separately verified. A preparation pass does not claim that the live project
remained historically unchanged.

Read-only connection inspection currently identifies an FTDI download-cable
candidate, not a verified FPGA UART. The CH9102 board UART and connected target
identity remain unverified. No serial device was opened or transmitted to.

## Download Restriction

The approved preparation scope includes code, local simulation, synthesis,
place-and-route, bitstream generation and read-only connection inspection only.
Actual JTAG download and changes to the running FPGA require separate explicit
authorization and the physical safety checks above. Flash programming is not an
implicit extension of SRAM programming. No motor or servo operation is authorized.
