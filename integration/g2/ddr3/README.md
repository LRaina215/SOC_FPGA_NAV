# DDR CPU migration candidate

This is an isolated candidate. The root build and active PNR15 are unchanged.
It preserves the original 128 KiB CPU address space and all control/navigation
storage, firmware and weights. It does not expand on-chip BSRAM.

Current evidence and limits: [design report](../../../docs/4_design/g2_ddr_cpu_offload_20261025.md).

The modules provide a 4 KiB write-through cache, banked immutable startup ROM
and one-outstanding-request core/UI mailbox for the Gowin native x32 BL8 1:4
interface. A warm CPU/key reset does not reset memory. The cold reset is for
FPGA power-on only. A DDR/PLL failure is latched; automatic recovery is gated.
The original 50 MHz control clock remains independent of DDR.

`prepare_candidate.py` derives a separately named SoC from the frozen source,
checks all 32768 initial words against the four original byte-channel images,
and generates ROM banks. It supports the current 128 KiB RAM and at most a
64 KiB initialized image. `test_candidate.py` runs bounded RTL memory and real
PicoRV32/debug comparisons. `measure_backend.py` measures the small memory
module with host memory and owned-process protection.

`generate_phy.py` uses the installed IPFlow Tcl API for a fresh C-device IP.
Generation includes the vendor's IP-only synthesis. `--inspect` rereads an
existing output without regenerating. Resource accounting includes wide RAM
leaves. The installed native-interface configurator did not retain MC_SSRAM;
requests for that known ineffective mode are rejected before generation.
`generate_pll.py` similarly generates/rechecks a current-device PLL.

`prepare_joint.py` freezes all original inputs, changes only the SoC backend
and top connections, preserves every original storage asset and adds DDR-only
pins. The source SDC has no new false-path exceptions. This is a full-SYN
candidate, not a physically signed-off or downloadable design.

The CPU-only full-SYN queue was withdrawn before it started after the
2026-10-25 01:03 cost-benefit review. Its preserved state is:

`F:/nav_integration_g2_20261023/lane2_ddr_cpu_chain/job.json`

Do not relaunch it automatically. The verified candidate remains available,
but the estimated saving is only four physical blocks and the synthetic
CPU program took 2.454 times as many cycles. The active PNR15 is untouched
and has no total or idle deadline. See the design report for the larger-map
DDR feasibility direction and its unverified latency/cache requirements.
