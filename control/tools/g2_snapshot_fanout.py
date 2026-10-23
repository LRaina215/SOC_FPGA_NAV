"""Expose only the policy snapshot data mux selector for bounded fanout.

The CE/reset nets are deliberately not annotated: Gowin syn_maxfan excludes
them. All snapshot bits, reset contents and update edges remain unchanged.
"""
import argparse
from pathlib import Path
ROOT=Path(__file__).resolve().parents[2]


def generate(path, maximum=128):
    path=Path(path); source=path.read_text(encoding='utf-8')
    assert maximum>=8
    assert 'snapshot_shift_select' not in source
    anchor='    wire deadline_missed = tick_count==PERIOD_CLKS-1 && state!=IDLE;'
    assert source.count(anchor)==1
    source=source.replace(anchor,anchor+f'''
    // Data-mux control only. No clock/CE/reset fanout attribute is implied.
    wire snapshot_shift_select /* synthesis syn_maxfan = {maximum} */;
    assign snapshot_shift_select = state==LOAD;
    wire snapshot_update = run_ok && !deadline_missed &&
        (snapshot_shift_select || (state==SNAPSHOT && frame_valid));
    always @(posedge clk or negedge rst_n) begin
        if(!rst_n) snapshot<=0;
        else if(snapshot_update)
            snapshot<=snapshot_shift_select ? {{24'd0,snapshot[4031:24]}} : {{history_flat,current_flat}};
    end
''')
    for old,new in (
        ('load_index<=0; snapshot<=0;','load_index<=0;'),
        ('snapshot<={history_flat,current_flat}; load_index<=0; state<=LOAD;','load_index<=0; state<=LOAD;'),
        ("                            snapshot<={24'd0,snapshot[4031:24]};\n",'')):
        assert source.count(old)==1,(path,old)
        source=source.replace(old,new)
    path.write_text(source,encoding='utf-8')


if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('path',type=Path);p.add_argument('--maximum',type=int,default=128)
    a=p.parse_args();generate(a.path,a.maximum)
