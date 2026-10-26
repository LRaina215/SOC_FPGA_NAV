"""One independent official-primitive simulation of actual SYN42 weights."""
import argparse
from datetime import datetime
import json
from pathlib import Path
import re
import shutil
import subprocess
import time
import traceback
import psutil

from review_stage3_slf128_memories import LIBRARY, meta
from retry_frozen_synthesis import memory_status

TB='''`timescale 1ns/1ps
module tb_actual_packed;
reg clk=0;always #10 clk=~clk;
reg en=0;reg[15:0]addr=0;wire[15:0]actual,reference;
reg gsri=0;GSR GSR(gsri);
fixed4500_weight_rom dut(clk,en,addr,actual);
source_weight_rom ref_dut(clk,en,addr,reference);
reg[15:0]gold[0:40703];reg[15:0]expect_word;
integer checks=0,reads=0,idles=0;
task step(input bit e,input[15:0]a);
begin
 @(negedge clk);#1;en=e;addr=a;
 @(posedge clk);#2;
 if(e)begin expect_word=gold[a];reads++;end else idles++;
 checks++;
 if($isunknown({actual,reference})||actual!==expect_word||reference!==expect_word)
  $fatal(1,"WEIGHT_MAP_FAIL check=%0d reads=%0d en=%b addr=%0d expected=%h actual=%h source=%h",checks,reads,e,a,expect_word,actual,reference);
end endtask
initial begin
 $readmemh("weights.mem",gold);#1;gsri=0;repeat(3)@(negedge clk);#1;gsri=1;repeat(3)@(negedge clk);
 // Explicit bank1->bank2 return, plus the complete original weight domain.
 step(1,8192);step(1,16385);step(0,16'hffff);step(1,8193);step(1,16384);
 for(integer k=0;k<40704;k++)begin step(1,k);step(0,k^65535);step(0,(k+12345)&65535);end
 $display("TEST PASSED ACTUAL_PACKED_WEIGHTS all40704=1 reads=%0d idle_hold=%0d checks=%0d official_unmodified_prim_sim=1",reads,idles,checks);$finish;
end
initial begin #4000000;$fatal(1,"finite actual packed weight watchdog");end
endmodule
'''


def main():
    ap=argparse.ArgumentParser(description=__doc__);ap.add_argument('--module',type=Path,required=True);ap.add_argument('--prepared',type=Path,required=True);ap.add_argument('--out',type=Path,required=True);a=ap.parse_args();assert not a.out.exists();a.out.mkdir(parents=True)
    shutil.copyfile(a.module,a.out/'actual.v');shutil.copyfile(a.prepared/'src/mem/weights.mem',a.out/'weights.mem')
    (a.out/'reference.v').write_text((a.prepared/'fixed4500_weight_rom.v').read_text().replace('module fixed4500_weight_rom(','module source_weight_rom(',1))
    (a.out/'tb.sv').write_text(TB);(a.out/'src/mem').mkdir(parents=True)
    for f in (a.prepared/'src/mem').glob('weight9_*.mem'):shutil.copyfile(f,a.out/'src/mem'/f.name)
    bindings=[meta(a.module),meta(LIBRARY)]+[meta(p)for p in a.out.rglob('*')if p.is_file()]
    (a.out/'frozen_inputs.json').write_text(json.dumps(bindings,indent=2)+'\n')
    me=psutil.Process();r=dict(controller_pid=me.pid,controller_created_epoch=me.create_time(),passed=False,SYN_started=False,PNR_started=False,automatic_retry=False,official_library_unchanged=meta(LIBRARY),actual_module=meta(a.module),runtimes=[])
    def status(phase,**kw):r.update(phase=phase,updated_at=datetime.now().astimezone().isoformat(),**kw);(a.out/'job.json').write_text(json.dumps(r,indent=2)+'\n')
    try:
        for tool,args in [('xvlog',['-sv',str(LIBRARY),'actual.v','reference.v','tb.sv']),('xelab',['-debug','off','-timescale','1ns/1ps','tb_actual_packed','-s','actual_weights']),('xsim',['actual_weights','-runall'])]:
            while True:
                m=memory_status();status('waiting_memory_'+tool,available_physical_bytes=m.avail_phys,available_commit_bytes=m.avail_page)
                if m.avail_phys>=2*1024**3 and m.avail_page>=4*1024**3:break
                time.sleep(10)
            for b in bindings:assert b==meta(Path(b['path'])),('Frozen input changed',b['path'])
            cmd=['cmd','/c',str(Path('E:/2026.1/Vivado/bin')/(tool+'.bat')),*args]
            with(a.out/(tool+'_stdout.log')).open('w')as stream:
                p=subprocess.Popen(cmd,cwd=a.out,stdout=stream,stderr=subprocess.STDOUT,creationflags=subprocess.CREATE_NO_WINDOW)
                rt=dict(tool=tool,command=cmd,wrapper_pid=p.pid,wrapper_created_epoch=psutil.Process(p.pid).create_time(),started_at=datetime.now().astimezone().isoformat());status('running_'+tool,runtime=rt)
                p.wait(timeout=300);rt.update(exit_code=p.returncode,ended_at=datetime.now().astimezone().isoformat());r['runtimes'].append(rt)
            assert p.returncode==0,(tool,p.returncode)
        log=(a.out/'xsim.log').read_text(errors='replace');assert 'TEST PASSED ACTUAL_PACKED_WEIGHTS'in log and not re.search(r'Fatal:|WEIGHT_MAP_FAIL',log)
        status('actual_weight_map_passed',passed=True,markers=[s for s in log.splitlines()if 'TEST PASSED'in s])
    except Exception as e:status('failed_preserved',passed=False,error=str(e),traceback=traceback.format_exc())
    (a.out/'result.json').write_text(json.dumps(r,indent=2)+'\n');print(json.dumps(r));return 0 if r['passed']else 1


if __name__=='__main__':raise SystemExit(main())
