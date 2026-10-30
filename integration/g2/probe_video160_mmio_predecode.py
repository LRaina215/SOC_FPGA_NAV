"""Admitted real register-file probe, then one bounded joint SYN if it passes."""
import argparse,json,subprocess,sys,time,shutil
from pathlib import Path
from datetime import datetime
import psutil
from retry_frozen_synthesis import memory_status
from run_video160_joint_syn import run as synthesize

VIV=Path('E:/2026.1/Vivado/bin')
REUSE=Path('F:/nav_integration_g2_20261023/K_video160_units_probe3')

def save(p,v):p.write_text(json.dumps(v,indent=2),encoding='utf-8')
def run(source,out,synthesis):
    assert not out.exists()and not synthesis.exists();out.mkdir()
    me=psutil.Process();state=dict(source=str(source),controller_pid=me.pid,controller_created_epoch=me.create_time(),
        synthesis_output=str(synthesis),automatic_PNR=False,programmed=False)
    def status(phase,**kw):
        state.update(phase=phase,updated_at=datetime.now().astimezone().isoformat(),**kw)
        save(out/'job.json',state);print(json.dumps(dict(phase=phase,**kw)),flush=True)
    try:
        while True:
            m=memory_status();status('waiting_for_actual_memory_admission',physical=m.avail_phys,commit=m.avail_page)
            if m.avail_phys>=2*1024**3 and m.avail_page>=4*1024**3:break
            time.sleep(30)
        fixture=source/'isolated_read_decode_probe'
        commands=[('xvlog',['-i',str(source/'nav/rtl/regs'),str(fixture/'reference.v'),str(source/'nav/rtl/regs/nav_regfile.v')]),
            ('xvlog',['-sv',str(fixture/'tb_mmio_read_predecode.sv')]),
            ('xelab',['tb_mmio_read_predecode','-s','mmio_snap']),('xsim',['mmio_snap','-runall'])]
        for index,(tool,args)in enumerate(commands):
            status('running_isolated_RTL_'+tool)
            with(out/f'{index}_{tool}.log').open('w')as log:
                p=subprocess.Popen(['cmd','/c',str(VIV/(tool+'.bat')),*args],cwd=out,stdout=log,
                    stderr=subprocess.STDOUT,creationflags=subprocess.CREATE_NO_WINDOW)
                identity=psutil.Process(p.pid);started=time.monotonic();low=0
                while p.poll()is None:
                    m=memory_status();low=low+1 if m.avail_page<512*1024**2 or (m.avail_phys<256*1024**2 and m.avail_page<1024**3)else 0
                    if low>=2 or time.monotonic()-started>300:
                        for c in reversed(identity.children(recursive=True)):
                            try:c.kill()
                            except psutil.NoSuchProcess:pass
                        p.kill();p.wait();raise RuntimeError('Isolated probe memory/runtime protection; only its own tree stopped')
                    time.sleep(1)
            assert p.returncode==0,(tool,p.returncode,str(out/f'{index}_{tool}.log'))
        text=(out/'xsim.log').read_text(encoding='utf-8',errors='replace')
        assert 'MMIO_READ_PREDECODE_PASS'in text and 'Fatal:'not in text
        marker=next(x for x in text.splitlines()if 'MMIO_READ_PREDECODE_PASS'in x)
        # Reuse unchanged video/HUD/PID tests with exact source binding.
        old=json.loads((REUSE/'result.json').read_text(encoding='utf-8'));assert old['passed']
        frozen=out/'source';frozen.mkdir()
        for rel in ('video/g2_video_framebuffer.v','video/g2_video_hud.v','control/motor520_closed_loop.v'):
            f=source/rel;assert f.read_bytes()==(REUSE/'source'/f.name).read_bytes();shutil.copyfile(f,frozen/f.name)
        result=dict(passed=True,source=str(source),checks=old['checks'],reused_from=str(REUSE),
            additional_checks=[dict(unit='actual_register_file',summary=marker)],
            scope='Actual old/new RF at E0 address capture, live RO inputs, RW and read-pop outputs; finite fixture. No wholeG2 or physical timing acceptance.')
        save(out/'result.json',result);status('isolated_RTL_passed_starting_one_joint_SYN',summary=marker)
        code=synthesize(source,synthesis,out)
        status('joint_SYN_finished_requires_actual_mapping_review',returncode=code);return code
    except Exception as error:
        import traceback
        status('failed_preserved_no_retry',error=str(error),traceback=traceback.format_exc());return 1

if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--source',type=Path,required=True);p.add_argument('--out',type=Path,required=True);p.add_argument('--synthesis-out',type=Path,required=True)
    a=p.parse_args();raise SystemExit(run(a.source.resolve(),a.out.resolve(),a.synthesis_out.resolve()))
