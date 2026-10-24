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

Map-offload evaluation now has an isolated implementation and reproducible
admission models:

- `nav_ddr_map_cache.v` retains both full maps behind small write-back caches,
  with line arbitration and the native mailbox. It is a feasibility prototype,
  **not connected to G2**. Ordinary request/ACK reads cannot feed the existing
  scan pipeline at one word per cycle.
- `probe_maps.py` runs full-capacity RTL checks, module SYN after the tested
  source is unchanged, and actual mapped vendor-primitive checks. The accepted
  module is `ddr_map_cache_syn2`: seven narrow RAM leaves,1972 LUT,1556 FF.
  Both RTL and mapped tests completed200706 SM and53250 elevation accesses.
- `evaluate_map_offload.py` compares candidate/point loop orders against the
  existing RTL-checked vectors, then adds cache-conflict and arbitrary-candidate
  cases. Lower miss counts in the normal cases are not a worst-case bound.
- `assess_map_streaming.py` checks first-update row bucketing against the
  existing numeric model and calculates the remaining5ms update budget.
  Continuous row DMA, cycle-enabled algorithm adapters and the actual DDR
  service bound remain unimplemented/unverified admission requirements.

The current generated C-device PHY is `ddr3_map_c_phy7_timingfix`. The component
audit corrected the old example's CL5/160ns refresh timing to CL6/CWL5,
tRCD/tRP15ns,tRFC260ns and tREFI3.9us for the full industrial temperature range.
The old PHY4 is rejected by the updated inspection and must not be deployed.
Generation does not prove training, electrical timing or bounded service.

Read `F:/nav_integration_g2_20261023/lane2_ddr_map_evaluation/job.json` before
continuing. CPU stays in its original128KiB on-chip RAM. The storage-only
estimate is112 removed blocks minus7 cache blocks minus32 predicted DDR blocks
=73 blocks saved; new streaming/node/FIFO storage must be deducted as added.
No new joint SYN or PNR has been started. PNR15 ended naturally at184 unrouted
nets/326 RAM blocks; the original unlimited-PNR policy remains in force.

Continuous-row UPDATE prototype (latest):

- `nav_ddr_row_stream.v` transports a full256-word row per descriptor, with
  two native beat credits and separate input/output ping-pong rows. It packs
  fourteen full18-bit cells per BL8 beat:19 beats/row, no quantization.
- `nav_slf_row_update.v` keeps first-cell semantics, row buckets and a
  clock-enabled derivative of the unchanged LF arithmetic. Pause holds both
  pending writes and the previous synchronous read result.
- `probe_row_update.py` prepares the exact LF derivative, runs full-map gold
  tests and measures this isolated module. `--mapped-after` requires a valid
  simulation license; the full netlist attempt exceeded BASIC's50000-instance
  limit and did not run. Do not record it as a pass.
- `probe_row_transform.py` compares UPDATE against the unchanged real matcher
  with the existing five point-stream vectors and original extraction/transform.
  Its derived matcher retains the old scoring RAM and is UPDATE-ONLY.

Accepted experiments: `ddr_map_row_update_probe7`524288 words and
`ddr_map_row_transform_probe1`327680 words; positive native-model profiles
at most3.84062ms, original-transform case at most3.59036ms. A slower legal
native-service profile needs10.66802ms. These are NOT a verified hardware
worst-case5ms bound. Full joint synthesis/PNR remains gated.

The row layout (row*19+column/14,18-bit slots) is incompatible with the old
32-bit-slot cache prototype. Adapt and prove coherence before combining them.
Isolated SYN14 sourceRAM/4 wide leaves predicts18 physical blocks including
retained touch/LF storage. Keeping the existing7 cache blocks plus6 new row/
node blocks plus predicted32 DDR blocks gives a conservative67-block saving;
remaining adapters can add storage. No whole-joint physical count is accepted.

Packed-cache/UPDATE coherence is now implemented in `nav_ddr_struct_cache.v`
and `nav_ddr_structure_store.v`. The new structure cache uses the same19-beat
row layout as the DMA; the old8x32-bit-slot structure cache remains historical.
The store drains all dirty entries, invalidates, synchronizes native ownership
both ways, and returns only new map data to a client held across UPDATE.
`probe_structure_store.py --row-probe <accepted-row-probe> --out <new>` tests
full capacity and all256 resident indices dirty with genuinely changed values.
`--synthesis-after <accepted-store-probe> --out <new>` measures this isolated
store; `--audit-existing --out <existing-SYN>` checks the three new ownership/
fault synchronizers and four narrow cache RAM leaves, without rebuilding.

Accepted store test: `ddr_structure_store_probe3`,298249 client checks plus
524288 backing words, including an in-flight client miss at UPDATE acceptance.
The miss completes before ownership transfer. Including full dirty drain/ownership handover, positive
native-service models need at most4.05068ms; a slower permitted service model
needs10.93456ms. Actual hardware worst-case5ms remains UNVERIFIED. The original
point-transform probe has not yet been combined with this dirty-cache test.
Isolated store SYN18 source/4 wide predicts22 physical, including retained
original touch/LF; the conservative whole-storage saving remains67 blocks.
Only three control chains have a SYN structural CDC check; payload delays,
full matcher EVAL/prior/debug adapters, UI-stop/PLL guarding, elevation scan,
cold DDR initialization and real controller service bounds remain admission
requirements. No full G2 SYN, PNR or hardware programming was started.


Complete adapters (03:08 follow-up; the preceding incomplete-adapter notes
are historical) are generated by derive_ddr_matcher.py,
derive_ddr_elevation.py and derive_ddr_subsystem.py from frozen original
SYN11 sources. probe_complete_matcher.py checks every245-candidate golden
score, five full structure maps,2618 prior writes and the actual MMIO FSM.
--dirty-before-update includes256 genuinely dirty lines before each UPDATE.
probe_elevation.py checks full53-bit elevation state; probe_map_pipeline.py
uses the original eight-frame projected point stream and all four traversal
passes. The accepted native-model results are in the03:08 machine evidence.
The heavy EVAL still takes121.07ms versus the original69.22ms; this is not
an end-to-end performance acceptance. nav_ddr_health.v is independently
tested but not yet wired to joint stopping. Shared SM/EM arbitration, actual
DDR worst service/5ms proof, cold initialization and full fault/CDC admission
remain required before a joint build. No full SYN/PNR was started.
