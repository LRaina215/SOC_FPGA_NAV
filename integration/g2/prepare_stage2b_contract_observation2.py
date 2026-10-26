"""Prepare the time0-observation correction in NEW validation inputs only.

No simulator/tool starts here. Hardware RTL and the original frozen fixture
stay untouched; validate_stage2b_back_to_bsram.py is the explicit run entry.
"""
import argparse
from datetime import datetime
import json
from pathlib import Path
import shutil

BASE=Path('F:/nav_integration_g2_20261023')
NAMES=['old_ram.v','new_ram.v','old_bias.v','new_bias.v','tb_stage2b_storage_contract.sv','sin_lut.hex','bias_narrow.mem']
OLD='  always@(negedge clk)begin\n    checks++;'
NEW='  // Ignore only the initialization X-to-0 clock event at time0.\n  // The first real falling edge still checks q=0 before any enabled read.\n  always@(negedge clk)if($time>0)begin\n    checks++;'

def meta(p):
    s=p.stat();return dict(path=str(p),bytes=s.st_size,mtime_ns=s.st_mtime_ns)
def write(p,value):p.write_text(json.dumps(value,ensure_ascii=False,indent=2)+'\n',encoding='utf-8')

def prepare(candidate,out):
    assert not out.exists()
    old=(candidate/'validation'/NAMES[4]).read_text(encoding='utf-8-sig')
    new=Path(__file__).with_name('tb_stage2b_storage_contract_observation2.sv').read_text(encoding='utf-8-sig')
    assert old.count(OLD)==1 and old.replace(OLD,NEW,1)==new
    manifest=json.loads((candidate/'manifest.json').read_text(encoding='utf-8-sig'))
    protected=[candidate/'validation'/n for n in NAMES]+[Path(p) for p in manifest['source_files']]
    before=[meta(p) for p in protected]
    (out/'validation').mkdir(parents=True)
    for name in NAMES:
        dest=out/'validation'/name
        if name==NAMES[4]:dest.write_text(new,encoding='utf-8',newline='\n')
        else:shutil.copy2(candidate/'validation'/name,dest)
    assert all((candidate/'validation'/n).read_bytes()==(out/'validation'/n).read_bytes() for n in NAMES if n!=NAMES[4])
    entry=json.loads((candidate/'validation/entry.json').read_text(encoding='utf-8-sig'))
    entry.update(cwd=str(out/'validation'),hardware_candidate=str(candidate),fixture_correction='Skip only time0 X-to-0 clock event; all real negedges unchanged')
    write(out/'validation/entry.json',entry)
    write(out/'validation/input_bindings.json',dict(frozen_at=datetime.now().astimezone().isoformat(),inputs=[meta(out/'validation'/n) for n in NAMES],no_SHA=True))
    assert before==[meta(p) for p in protected]
    write(out/'correction.json',dict(phase='new_fixture_inputs_prepared_not_run',original_candidate=str(candidate),
      exact_change=dict(old=OLD,new=NEW),six_other_validation_files_byte_identical=True,hardware_RTL_edited=False,
      protected_metadata_before=before,protected_metadata_unchanged=True,Gowin_started=False,simulator_started=False))

if __name__=='__main__':
    parser=argparse.ArgumentParser();parser.add_argument('--candidate',type=Path,default=BASE/'stage2B_bitmap_mailbox_slist_trig_console_bias_prepared1')
    parser.add_argument('--out',type=Path,required=True);args=parser.parse_args();prepare(args.candidate.resolve(),args.out.resolve())
