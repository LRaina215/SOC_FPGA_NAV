# Pin Verification

Scope: isolated `fc_v4_board_top` build only, target
GW5AT-LV138PG484AC1/I0, device version C, 50 MHz oscillator. No download,
programmer execution, serial transmission, or actuator operation is authorized.

## Authoritative Inputs

- Active mappings: `/home/cyd/fpga/Documents/fpga_embedded_competition/src/fpga_embedded_competition.cst`.
  Clock and all 12 actuator assignments are copied exactly; UART is absent there.
- Core schematic: `/home/cyd/fpga/references/ACG750/硬件图纸/ACX750-GW5A138-CORE-25092801.pdf`,
  sheet 1 (banks 2/3/4, oscillator and VCCIO), sheet 2 (power pins), sheet 8 (connectors).
- Baseboard schematic: `/home/cyd/fpga/references/ACG750/硬件图纸/ACX750-GW5A138-CB-250928.pdf`,
  sheet 1 (bank supplies), sheet 6 (VCCIO2 voltage selection), sheet 8 (CH9102F UART),
  sheet 10 (P11 connector).
- Pin workbook: `/home/cyd/fpga/references/ACG750/硬件图纸/ACX750-GW5A138开发板管脚信息表.xlsx`,
  sheet `开发板管脚名`, C3= W19/50 MHz, C30=L21 UART_RXD, C31=M21 UART_TXD,
  C171:C174=servo GPIO0_0..3, C183:C190=motor GPIO0_10..17.
  Workbook was read without modification with ZIP/XML structured parsing because
  openpyxl is not installed. No package installation was performed.
- Existing live routed report was read only as a secondary package-bank check:
  `/home/cyd/fpga/Documents/fpga_embedded_competition/impl/pnr/fpga_embedded_competition.rpt.txt`.
  It is not timing evidence for this isolated board build.

## Directions And Pads

| Port | FPGA Pad | Bank | FPGA Direction | Connector/Meaning |
|---|---|---|---|---|
| clk | W19 | 4 | input | U2 50M_active oscillator via R1 33 ohm |
| UART_RXD | L21 | 3 | input | CH9102F U19 TXD pin 21 -> FPGA receive |
| UART_TXD | M21 | 3 | output | FPGA transmit -> U19 RXD pin 20 |
| MOTOR_EN | A20 | 2 | output, constant 0 | P11 pin 20 |
| PWM_U | E17 | 2 | output, constant 0 | P11 pin 14 |
| PWM_V | D15 | 2 | output, constant 0 | P11 pin 16 |
| PWM_W | F14 | 2 | output, constant 0 | P11 pin 18 |
| MOTOR_EN2 | B20 | 2 | output, constant 0 | P11 pin 19 |
| PWM_U2 | F16 | 2 | output, constant 0 | P11 pin 13 |
| PWM_V2 | D14 | 2 | output, constant 0 | P11 pin 15 |
| PWM_W2 | F13 | 2 | output, constant 0 | P11 pin 17 |
| SERVO1 | C18 | 2 | output, constant 0 | P11 pin 1 |
| SERVO2 | C19 | 2 | output, constant 0 | P11 pin 2 |
| SERVO3 | E19 | 2 | output, constant 0 | P11 pin 3 |
| SERVO4 | D19 | 2 | output, constant 0 | P11 pin 4 |

Count is **15 physical pads**, not 13: 1 clock + 2 UART + 8 motor + 4 servo.
There are 13 outputs and 2 inputs. All assignments are unique. The isolated
CST includes no LEDs, camera, SPI, encoder, JTAG user logic, or unrelated main.
Bank2 actuator pads are not configuration pins; bank3 UART pads are ordinary
IOR71 A/B. W19 is clock-capable. Dedicated JTAG/configuration pins are not
repurposed. Build acceptance requires the new routed report to show exactly
these pads, banks, directions, LVCMOS33 and BANK_VCCIO=3.3.

UART orientation was verified against U19, not guessed from USB net names.
`pdftotext -bbox -f 8 -l 8 ... -` shows vertical UART_RXD label at x677.714..683.792,
aligned with pin21 (x677.954..684.032) and TXD; UART_TXD at x684.914..690.992
aligns pin20 (x685.154..691.232) and RXD. Thus host bridge TX drives FPGA RX.
Sheet1 routes B3_R71_N to UART_RXD and B3_R71_P to UART_TXD; core sheet1
maps L21 to B3_R71_N and M21 to B3_R71_P. Sheet8 also shows U19 VIO pin5
on VCC3P3 and R94/R95 1K pulls toward VCC3P3 on the UART nets.

## Electrical And Safety Limits

CST uses LVCMOS33, BANK_VCCIO=3.3, no pulls on driven outputs, DRIVE=8
matching the active actuator CST. RX uses a weak pull-up for UART idle high.
Bank3 supply is VCC3P3 through baseboard FB15; clock/bank4 supply is VCC3P3
through core FB1. **Bank2 voltage is configurable** on baseboard sheet6:
U15/P9 can select 2.5/1.8/1.5 V or VCC3P3. The active CST and prior routed
report specify 3.3 V, but software does not measure the actual jumper or rail.
The future human-authorized download must physically verify bank2=3.3 V,
board revision, common ground, connected loads and voltage compatibility.
Do not treat BANK_VCCIO in a constraint as a hardware voltage measurement.

Wrapper source drives all 12 motor/servo outputs unconditionally to zero,
including POR and all communication/inference states. Original motor drivers
are active-high, so zero disables their logic inputs. Mechanical safety is
unknown: removal of servo pulses may release a loaded mechanism. Isolate
actuator power and support the robot before any future authorized download.
Constant-zero RTL does not guarantee zero pads during configuration.

## Timing And Configuration Rationale

`board/gowin/board.sdc` creates a 20.000 ns clock on clk. No internal clock
group, multicycle, global false-path, or reset exception masks inference or
UART logic. UART_RXD is asynchronous, so the only false path is from that
port to **bridge/rx/sync1/D**, the first synchronizer stage. The generated
netlist/timing report must confirm that target resolves; sync1 -> sync2 and
all downstream paths remain timed. If synthesis renames sync1, adjust only
the exact first-stage D pin after inspecting the netlist, never broaden it.

UART_TXD has no synchronous external receiver clock. Max/min output delay
0 ns relative to clk imposes an explicit synthetic one-cycle output budget,
not a claimed CH9102 setup/hold requirement. Twelve constant outputs have
no clocked timing paths and do not need invented IO delays. Routed timing
must report 50 MHz/Fmax, setup and hold violations, critical from/to paths,
and warning/coverage details. Unknown internal clock/constraint coverage
fails acceptance; synthesis success or an fs file alone is insufficient.

Installed local guides checked: IDE/doc/EN/SUG1220-2.1E (create_project,
run all, set_option), SUG940-2.0.5E (SDC), SUG100-4.4.7E (configuration),
Programmer/doc/EN/UG704-1.0.7E (Arora V wake-up), SUG502-2.5.2E (CLI).
`-unused_pin default` explicitly means input tri-state **with weak pull-up**,
not floating/no-bias; multiplexed/configuration pins are excluded.
`-use_jtag_as_gpio 0`, `-use_sspi_as_gpio 0`, `-bg_programming off` preserve
configuration functionality and disable optional background programming.
`-bit_incl_bsram_init 1`, `-bsram_init_print_mode default`, uncompressed
bitstream preserve memory initialization evidence. No output-activation
option is invented: UG704 section4.2.4 specifies GOE enables outputs before
GSR/GWDISn release. POR is not a substitute for power isolation or external
safe bias during configuration. `-wakeup_mode` defaults off and is not enabled.

## Build And Evidence

Run preflight with `bash board/build_board.sh`. After the coordinator declares
the repaired RTL suite stable, build with `bash board/build_board.sh --build --rtl-ready`.
Read-only connection report: `bash board/build_board.sh --connection-status`.
No programmer/download command is part of either build script.

Every build gets `board/gowin/runs/<UTC timestamp>/`, complete `build.log`,
`effective_options.tcl`, source/protected before/after JSON hashes, all Gowin
reports, `build_status.json`, warning/coverage text and routed timing text.
`board/gowin/build_status.json` points at the latest run. The build checks
`board/baseline.json` core/ROM identities are fatal gates; the baseline stays
immutable. Live-project main/CST are not compiled and their observed external
drift is recorded explicitly against the historical baseline, not overwritten
or claimed historically unchanged. Their current before/after hashes are
captured. The entire original isolated `gowin/` synthesis tree is also
snapshotted; core/ROM edits or build-input/original-synthesis changes reject
artifacts. Both core RTL dependencies are explicit; only board/rtl/*.v is
globbed. The original synthesis root remains untouched. The CLI uses the
installed Qt offscreen backend for this process only, with no browser/UI.
ROM depths/nonzero words, inferred DSP/BSRAM resources, nonzero INIT_RAM
parameters and fs SHA256 are independently recorded. A hardware replay is
not claimed by these software checks.

Connection evidence is persistent in `board/reports/connection_status.json`.
Read-only USB enumeration currently sees FTDI 0403:6014 and its ttyUSB0 link,
which is a **JTAG/download-cable candidate**, not verified board UART.
The CH9102F board USB-UART is not detected. No tty node is opened, no packet
sent, no JTAG scan performed, and the board connection remains unverified.
`Programmer/bin/programmer_cli` exists locally; SUG502 section4.7 documents
SRAM operation index2. A later download needs separate human authorization,
verified target/cable identity and selected fs hash. Flash programming, probe
configuration switching and credentials/permissions/system changes are not authorized.

**No programming, download, or actuation has been performed.**
