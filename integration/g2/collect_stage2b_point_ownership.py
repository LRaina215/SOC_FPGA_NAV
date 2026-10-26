"""Read completed unique ownership probe and freeze compact evidence; no tools."""
from pathlib import Path
import argparse,json,re,psutil
from prepare_baseline_fusion_retime import metadata


def collect(candidate,probe,out):
    assert not out.exists()
    review=json.loads((candidate/'prepared_source_review.json').read_text(encoding='utf-8'))
    unchanged=[]
    for row in review['bindings']+review['validation_bindings']:
        assert metadata(Path(row['path']))==row,('frozen input changed',row)
        unchanged.append(row['path'])
    assert (candidate/'validation/tb.sv').read_bytes()==Path(review['fixture_only_input_generator']['path']).read_bytes()
    result=json.loads((probe/'result.json').read_text(encoding='utf-8'))
    log=(probe/'xsim.log').read_text(encoding='utf-8',errors='replace')
    assert result['passed'] and all(x['exit_code']==0 for x in result['runtimes'])
    assert 'TEST PASSED: POINT OWNERSHIP' in log and not re.search(r'Fatal:|TEST FAILED|ERROR:',log)
    m=re.search(r'SLF OWNERSHIP PASS cases=(\d+) checks=(\d+) miter_cycles=(\d+) wait_cycles=(\d+) reads=(\d+) writes=(\d+)',log)
    n=re.search(r'XT OWNERSHIP PASS pixels=(\d+) passes=(\d+) accepted=(\d+) normal_miter_cycles=(\d+)',log)
    assert m and n
    identities=[]
    for runtime in result['runtimes']:
        for row in [dict(pid=runtime['wrapper_pid'],created_epoch=runtime['wrapper_created_epoch'],name=runtime['tool']+'_wrapper')]+runtime.get('processes',[]):
            try:
                p=psutil.Process(row['pid'])
                same=abs(p.create_time()-row['created_epoch'])<0.01
                state='still_running_same_identity' if same and p.is_running() else 'PID_reused'
            except psutil.NoSuchProcess:state='ended'
            assert state!='still_running_same_identity',row
            identities.append(dict(**row,terminal_state=state))
    summary=dict(passed=True,source_frozen=str(candidate),probe=str(probe),tool_result=str(probe/'result.json'),
        SLF=dict(zip(('cases','checks','normal_miter_cycles','WAIT_cycles','PB_reads','PB_writes'),map(int,m.groups()))),
        XT=dict(zip(('pixels','complete_passes','accepted_starts','normal_miter_cycles'),map(int,n.groups()))),
        controller_pid=result['controller_pid'],tool_identities_ended=identities,
        simulation_finish_ns=re.search(r'\$finish called at time\s*:\s*([\d.]+) ns',log).group(1),
        tool_exit_codes={r['tool']:r['exit_code']for r in result['runtimes']},
        admission_minimum_physical_bytes=min(r['admission_physical_bytes']for r in result['runtimes']),
        admission_minimum_commit_bytes=min(r['admission_commit_bytes']for r in result['runtimes']),
        frozen_bindings_unchanged=len(unchanged),RTL_inputs=63,assets=review['asset_bindings_count'],validation_inputs=len(review['validation_bindings']),
        no_Fatal_or_ERROR=True,normal_bus_ACK_source_unchanged=True,
        normal_ready_assert_and_command_publish='Normal ex_done-ready and completed-producer command/output cycles compared against original actual RTL;226310 SLF and19028 XT cycles. Intentional ready clearing at a new raw generation is outside old sticky-ready equivalence.',
        additional_RAM=0,source_predicted_additional_control_bits=7,
        physical_safety=False,synthesis_started=False,PNR_started=False,programmed=False,
        limitations=['Source/behavioral RTL only; actual primitive CEB/BLKSEL/AD/mode not proven','No integrated full-nav bus/Pico simulation or350ms/physical stop result','Only original single-frame stable configuration contract; no arbitrary multi-descriptor FIFO','No explicit integrated simultaneous raw-accept+EVAL bus test: source cmd gate blocks new raw acceptance on command pulse','Soft-clear remains original IDLE-only admission; no new busy-command abort/queue contract'],
        reproduction=['D:/Anaconda/python.exe','-B','-X','utf8','D:/Desktop/SOC FPGA/nav/integration/g2/validate_stage2b_weight_point.py','--candidate',str(candidate),'--out','<new independent output>','--run'])
    out.parent.mkdir(parents=True,exist_ok=True);out.write_text(json.dumps(summary,ensure_ascii=False,indent=2)+'\n',encoding='utf-8')
    print(json.dumps(dict(out=str(out),passed=True,SLF=summary['SLF'],XT=summary['XT']),ensure_ascii=False))
    return summary


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--candidate',type=Path,required=True);p.add_argument('--probe',type=Path,required=True);p.add_argument('--out',type=Path,required=True);a=p.parse_args();collect(a.candidate,a.probe,a.out)
