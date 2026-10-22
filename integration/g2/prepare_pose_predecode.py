"""Move only POSE_COMMIT address decoding before the existing bus register.

The same bus branch captures both the original write enable/address and this
pulse. No point-stream latency, frame metadata or bus acknowledgement changes.
"""
from pathlib import Path
ROOT=Path(__file__).resolve().parents[2]


def generate(target):
    s=(ROOT/'rtl/top/nav_subsystem.v').read_text(encoding='utf-8')
    def replace(old,new):
        nonlocal s
        assert s.count(old)==1,old
        s=s.replace(old,new)
    replace('    reg         rf_wr, rf_rd;','    reg         rf_wr, rf_rd;\n    reg pose_predecoded_q;')
    replace('.pose_commit(p_LIDAR_POSE_COMMIT)', '.pose_commit(pose_predecoded_q)')
    replace('.frame_start(p_LIDAR_POSE_COMMIT & !match_only)', '.frame_start(pose_predecoded_q & !match_only)')
    replace("        bus_ack <= 1'b0; rf_wr <= 1'b0; rf_rd <= 1'b0;", "        pose_predecoded_q <= 1'b0;\n        bus_ack <= 1'b0; rf_wr <= 1'b0; rf_rd <= 1'b0;")
    replace("                    if (bus_we) rf_wr <= 1'b1; else rf_rd <= 1'b1;", """                    if (bus_we) begin
                        rf_wr <= 1'b1;
                        // Same transaction/word alignment as the old regfile
                        // decode, registered on the same edge as rf_wr/a_q.
                        pose_predecoded_q <= ({bus_addr[15:2],2'b00} == RA_LIDAR_POSE_COMMIT);
                    end else rf_rd <= 1'b1;""")
    Path(target).write_text(s,encoding='utf-8',newline='\n')
