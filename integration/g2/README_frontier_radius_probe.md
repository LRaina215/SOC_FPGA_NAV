# Independent fixed-radius frontier candidate

This source-only candidate follows the completed checkpoint2 candidate. It does
not modify deployment firmware, frozen hardware inputs, or a running build.

`prepare_frontier_radius_candidate.py --out <new-directory>` copies checkpoint2
and changes only the fixed 1000 mm blacklist predicate in `nav/explore.c`.
The frontier-minimum predicate, queue, scan, scoring, ties, checkpoints, pending
message handling, stop guard, scratch, and original full capacities remain.
It runs focused host comparisons and compiles/reviews the complete firmware.

The original expression subtracts signed int32 coordinates, squares in int64,
then converts the integer square root to int32. Extreme inputs have signed
overflow or implementation-defined conversion; portable C does not define an
all-int32 original function. The candidate uses a fast domain whose int64
differences satisfy `abs(dx),abs(dy) <= 2^30`. Here the original arithmetic and
result are representable, and `floor(sqrt(q)) < 1000` is exactly equivalent to
`q < 1000000`. An axis at or beyond 1000 mm rejects exactly. After axis rejection,
both coordinates are within -999..999 and the 32-bit square sum cannot overflow.
Outside that proven domain the literal original expression is called, with the
same compiler options. Compiler-specific extreme-input comparisons are observations,
not a portable-C proof of undefined behavior.

`probe_frontier_radius_cpu.py --candidate <candidate> --out <new-probe-directory>`
prepares one real PicoRV32/128 KiB/50 MHz xsim fixture. Its normal 0-blacklist,
normal 8-far-blacklist, and loss inputs and timer/mailbox service match the prior
checkpoint CPU probe. Before timing, 1236 target-compiler extreme/radius-boundary
comparisons call the actual private helper through an appended simulation-only
wrapper. The source function body is unchanged. Original functions and hardware
time are used; the timer/mailbox register fabric is an explicit synthetic service,
not full G2, UART wire, navigation load, or physical standstill.

Start the prepared `run_xsim.py --suite frontier` once. It requires fresh
2 GiB physical / 4 GiB commit availability and refuses an existing run marker.
Its `frontier/job.json` records PID creation times, commands, and final result.
Do not rerun completed checkpoint2 evidence or start another ModelSim session.

The first radius firmware compile succeeded but the older shared ELF reader
rejected its 4-byte pre-BSS gap. The actual ELF BSS alignment is 8 bytes. The
radius-specific reader validates the exact section alignment, load end, BSS
start/end, RAM limits, scratch size, and stack. The first rejection is retained.
For that existing build only, `--review-existing` rechecks the same ELF without
compiling it again. This does not change the shared legacy image reader.

The complete radius firmware is 52796 bytes; BSS is 69984 bytes, scratch 20992
bytes, static no-IRQ stack bound 2576 bytes, with the original 6144-byte stack
reservation and 2144 additional bytes remaining. Against checkpoint2, binary
growth is 172 bytes and BSS is unchanged; 4 additional alignment bytes account
for the total 176-byte free-space change. Actual CPU timing is recorded separately.
Finite fixture results never establish whole-application WCET or a physical
500 ms stop contract.
