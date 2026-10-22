# UART Inference Protocol v1

This interface replays observations/history through the inference core and reads
verification results. It has no ARM, motor, servo, control-enable, reset, or CLEAR
command. Host mocks and offline captured replies do not establish physical-board
operation or vehicle safety. No devices need to be opened to run host tests.

## Line and frame format

UART is 115200 baud, 8 data bits, no parity, one stop bit (8N1), with no hardware
flow control. All multibyte integers are little-endian, except the checkpoint
SHA256, which is raw digest bytes in normal digest order, not byte-reversed.
Signed24 words are three-byte little-endian two's-complement integers, signed24
F12 except the golden saturation boolean, which is F0.

There is no separate addressable register bus/map. These are packet byte offsets:

| Packet byte offset | Width | Field |
|---|---:|---|
| 0 | 2 | Magic `A5 5A` |
| 2 | 1 | Version `01` |
| 3 | 1 | Type, uint8 |
| 4 | 2 | Payload length, uint16LE, excludes header and CRC |
| 6 | 4 | Frame ID, uint32LE |
| 10 | payload length | Payload |
| 10 + payload length | 2 | CRC16, uint16LE |

The fixed header is 10 bytes; CRC adds 2 bytes, so total overhead is 12 bytes.
CRC16/CCITT-FALSE uses initial `FFFF`, polynomial `1021`, no reflection, no final
XOR, over **version through the final payload byte**, excluding magic and CRC.
The known answer for ASCII `123456789` is `29B1` (wire CRC bytes `B1 29`).

| Type | Name | Payload bytes |
|---|---|---:|
| `01` | INFER | 486 |
| `02` | PING | 0 |
| `81` | INFER result | 1340 |
| `82` | PING result | 32 |
| `FF` | ERROR | 1 |

A reply echoes the request frame ID. Host frame IDs are monotonically increasing
without uint32 wrap within an invocation; PING uses one less than the first INFER
ID. `--frame-id-start` changes the sequence when attaching to a board that has
already accepted an inference. This is not a reset or a globally unique ID promise.

## Payload offsets

INFER contains 162 signed24LE words: current observation27 followed by history135
(oldest-to-newest). Observation occupies payload bytes 0..80; history occupies
81..485. The full request must pass CRC/version/type/length validation before any
core input writes or acceptance. Each accepted inference reloads all 162 words.

The exact checkpoint digest is:

`29b12e353b84380fa57ed7c3f8928fb1d51e9df99c0fa01bd3f4aec2de5a8c03`

PING reply is these 32 raw bytes, beginning `29 B1 2E 35`, not an ASCII string.

| Result payload offset | Packet offset | Bytes | Field |
|---|---:|---:|---|
| 0 | 10 | 32 | Checkpoint digest |
| 32 | 42 | 4 | Core cycles, uint32LE; expected 81998 |
| 36 | 46 | 1 | Sticky core flags |
| 37 | 47 | 1 | Per-inference saturation, exactly 0 or 1 |
| 38 | 48 | 18 | Six clipped actions, signed24LE |
| 56 | 66 | 9 | Three encoder latent outputs, signed24LE |
| 65 | 75 | 1275 | All 425 layer outputs, signed24LE |

Payload size is exactly `32 + 4 + 1 + 1 + 18 + 9 + 1275 = 1340`. Layer order is
encoder0[128], encoder2[64], encoder4[3], actor0[128], actor2[64], actor4[32],
actor6[6]. Corresponding word offsets are 0, 128, 192, 195, 323, 387, 419. Hidden
layers are post-ELU; actor6 is raw, while the separately returned actions clip to
integer +/-4096. Every layer, latent, and clipped action is compared bit-exactly.

Core flags: bit0 busy-write rejection; bit1 busy/simultaneous start rejection;
bit2 bad input address; bit3 incomplete fresh frame; bit4 intermediate signed24
saturation. Bits0..3 and reserved bits must be zero for host acceptance. Bit4 is
sticky since reset, whereas the saturation byte is per-inference and must match
`expected_saturated.mem`. Saturation excludes final action clipping and input
quantization saturation. PING cannot establish whether reset happened or bit4
was already set. By default the first nonsaturated result may have bit4 zero or
one; the host then validates its observed sticky baseline on subsequent results.
A saturated result must always set bit4. Use `--initial-sticky-saturation 0` for a
known fresh-reset run (including reset-based RTL captures), or `1` for known prior
saturation. The host never issues a reset or CLEAR to establish this baseline.

## Errors and sequencing

ERROR payload is a single code byte:

| Code | Meaning |
|---:|---|
| 1 | Unsupported version |
| 2 | Unsupported type |
| 3 | Wrong length |
| 4 | CRC mismatch |
| 5 | Receive framing error / partial-frame timeout |
| 6 | Busy |
| 7 | Duplicate of last accepted INFER since reset |
| 8 | Core error |

Only the last **accepted INFER** is the duplicate reference. PING and rejected
requests never replace it. After accepted IDs X, Y, another X is not a duplicate
of the last accepted Y. The host still allocates new IDs and never retries an
ambiguous timed-out inference automatically.

The host permits one request-response in flight, waiting for and validating the
complete reply before sending the next request. RTL may queue at most one error
behind a completed normal response; excess errors may be dropped but must never
overwrite/drop a normal inference result. This is not permission to pipeline
requests. Unexpected frame/type, ERROR, CRC failure or timeout aborts the host
session. Stream parsing supports fragmented reads, leading-noise resynchronization,
bounded lengths and partial-frame timeout; protocol faults fail visibly. There
is no automatic device scanning, retry, flush-reset, or arming behavior.

## Timing

Each inference transfers 498 request bytes and 1352 response bytes, totaling
1850 bytes. At 8N1 there are 10 line bits per byte, so line time is
`1850 * 10 / 115200 = 160.590 ms`. Core computation at 50MHz adds
`81998 / 50000000 = 1.63996 ms`, plus loader/dispatch and USB/OS overhead. Practical
minimum roundtrip is about 0.162 seconds, approximately 6.2Hz, **not 125Hz**.
The client reports measured host roundtrip seconds separately from core cycles
and core seconds at 50MHz. Core latency excludes UART serialization and loading.

## Host invocation

Python3 standard library suffices for dry-run, offline checks, and tests. Only
hardware mode requires the optional `pyserial` package (`python3 -m pip install
pyserial` if the operator chooses to install it). No dependency is downloaded by
these tools or tests. No serial package is imported in dry-run/offline mode.

```bash
ROOT=/home/cyd/fpga/pt_to_fpga/full_center_v4_deploy_20261005
PYTHONDONTWRITEBYTECODE=1 python3 -m unittest discover -s "$ROOT/board/host/tests" -v
python3 "$ROOT/board/host/client.py" --dry-run --samples 3 --no-report
python3 "$ROOT/board/host/client.py" --dry-run --vector-dir "$ROOT/assets/vectors/stress" --samples 8 --no-report
```

`--vector-dir` defaults to `assets/vectors` (synthetic). `--samples N` selects N
samples starting at optional `--start-index` (default 0). Six files are mandatory:
inputs, expected_layers, expected_raw, expected_clipped, expected_latent and
expected_saturated, each named `.mem`. All require exact six-digit hexadecimal
width, nonempty complete sample counts and consistent counts across files. The
loader also cross-checks latent/raw layer slices and clipped actions. No silent
fallback occurs if a golden file is missing.

Hardware mode requires explicit `--port DEVICE`. It first verifies the PING reply
CRC, version, type, frame ID, length and exact digest, then sends INFER requests
and validates all outputs, metadata and timing fields. Serial is configured with
DTR/RTS false **before** opening and no software reset or modem-line toggle is
issued. OS drivers or USB adapters can glitch those pins during open independently
of pyserial; to guarantee no physical reset, use an adapter/wiring without reset
lines connected. The software cannot certify adapter behavior. No port is opened
while writing/running the host unit tests.

For an operator-controlled future session, an example is:

```bash
python3 "$ROOT/board/host/client.py" --port /dev/EXPLICIT_DEVICE --samples 3 --initial-sticky-saturation 0
```

Importing modules writes no reports. Explicit CLI invocations default to timestamped
JSON and log files under `board/reports`; `--report-dir` overrides this and
`--no-report` prints JSON only. Dry-run reports say preflight-only and do not claim
output comparison. The client records failures and exits nonzero. None of these
reports alone claims physical deployment/safety; mocked hardware paths remain tests.

## Captured RTL replies

`--offline HEX_FILE` checks one captured complete wire packet without hardware,
including CRC/version/type/length/frame/digest and all golden fields. The file
contains whitespace-separated byte hex or contiguous hex, without annotations.
A sidecar `HEX_FILE.json` or same-stem `.json` is required. Metadata is a JSON
object containing `frame_id` and preferably `sample_index`; unknown fields are
preserved in the report. An explicit `--frame-id` can supply/confirm the expected
ID; any metadata/sample mismatch is rejected. `--start-index` selects the golden
sample, and `--samples` must be 1 in this mode.

```bash
python3 "$ROOT/board/host/client.py" --offline "$ROOT/board/reports/rtl_reply_nominal.hex" --start-index 0 --initial-sticky-saturation 0 --no-report
python3 "$ROOT/board/host/client.py" --offline "$ROOT/board/reports/rtl_reply_stress.hex" --vector-dir "$ROOT/assets/vectors/stress" --start-index 1 --initial-sticky-saturation 0 --no-report
```

`transport.check_capture` exposes the same comparison for callers. Captured UART
RTL simulation packets provide cross-implementation framing evidence, not proof
that a board was programmed or physical UART pins operated.
