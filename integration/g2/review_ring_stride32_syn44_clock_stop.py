"""SYN43->44 actual two-clock/original26/IR/lease read-only review.

Reuse the completed-netlist reader, with the three intentional ring RTL edits
excluded from whole-file identity and their unchanged stop/sequential sections
checked explicitly. No EDA is started and no build input is written.
"""
from pathlib import Path
import argparse
import review_slf128_syn42_clock_stop as reviewer

BASE=Path('F:/nav_integration_g2_20261023')
OLD=BASE/'SLF128_storage_safeFW_syn43'
NEW=BASE/'SLF128_ring_stride32_syn44'


def source_contract():
    paths=['g2_top.v','control4500_core.v','control/policy_action_map.v',
           'control/ir_remote_control.v','control/motor520_closed_loop.v',
           'control/motor520_hybrid_speed.v','command_lease_guard.v',
           'nav_command_units.v','nav/rtl/plan/safety_guard.v',
           'nav/rtl/common/nav_cdc.v','nav/rtl/common/nav_fifo_ob.v']
    same={p:(OLD/p).read_bytes()==(NEW/p).read_bytes()for p in paths}
    assert all(same.values()),same
    def block(root,path,start,end=None):
        t=(root/path).read_text()
        i=t.index(start)
        return t[i:]if end is None else t[i:t.index(end,i)]
    stop='    reg [31:0] sensor_age;'
    end='    wire capture,frame_valid,history_initialized;'
    assert block(OLD,'control/mem_policy.v',stop,end)==block(NEW,'control/mem_policy.v',stop,end)
    sequential='    always @(posedge clk or negedge rst_n) begin'
    assert block(OLD,'control/rl_policy_tick_v5.v',sequential)==block(NEW,'control/rl_policy_tick_v5.v',sequential)
    assert 'localparam integer CLK_HZ = 50000000;'in(NEW/'g2_top.v').read_text()
    return dict(passed=True,byte_identical=same,mem_policy_sensor_stop_block_identical=True,
                policy_all_sequential_blocks_identical=True,core_source_CLK_HZ=50000000,
                intentional_ring_RTL=['control/mem_policy.v','control/rl_observation_v5.v','control/rl_policy_tick_v5.v'],
                limits='Ring combinational/prefetch changes have separate actual mapping and real4500 RTL evidence; whole mapped core equivalence is not asserted.')


def run(out):
    reviewer.OLD=OLD
    reviewer.NEW=NEW
    reviewer.OUT=out
    reviewer.source_contract=source_contract
    return reviewer.run()


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--out',type=Path,required=True)
    a=p.parse_args()
    raise SystemExit(run(a.out))
