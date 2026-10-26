"""One read-only completed SYN44->45 review, with a persistent admission wait.

No EDA is launched. The two workers run in series and release their parsed
graphs on exit. Source bindings are checked before actual netlist parsing.
This scope excludes the separately assigned action mapper/NAV/PID mapping.
"""
from pathlib import Path
from datetime import datetime
from collections import Counter
import argparse,json,os,psutil,subprocess,sys,time,traceback

BASE=Path('F:/nav_integration_g2_20261023')
OLD=BASE/'SLF128_ring_stride32_syn44'
NEW=BASE/'SLF128_measured_timing_syn45'
PREP=BASE/'SLF128_measured_timing_SYN45_prepared1'
DEFAULT_OUT=BASE/'SYN45_storage_clock_ring_review1'
CHANGED={'control/mem_policy.v','control/policy_action_map.v',
         'control/rl_policy_tick_v5.v','control4500_core.v','nav/rtl/top/nav_subsystem.v'}
GIB=1024**3


def load(p):return json.loads(p.read_text(encoding='utf-8-sig'))
def save(p,v):p.write_text(json.dumps(v,ensure_ascii=False,indent=2)+'\n',encoding='utf8')
def meta(p):s=p.stat();return dict(path=str(p),bytes=s.st_size,mtime_ns=s.st_mtime_ns)
def stamp():return datetime.now().astimezone().isoformat()


def source_contract():
    paths=['g2_top.v','control/rl_observation_v5.v','control/ir_remote_control.v',
           'control/motor520_closed_loop.v','control/motor520_hybrid_speed.v',
           'command_lease_guard.v','nav_command_units.v','nav/rtl/plan/safety_guard.v',
           'nav/rtl/common/nav_cdc.v','nav/rtl/common/nav_fifo_ob.v',
           'nav/rtl/common/nav_async_fifo.v','nav/rtl/mac/rgmii_mac100.v']
    # Find the existing MAC source path through the frozen manifest, rather
    # than assuming a directory name when loading a later candidate.
    source_paths=[Path(p).relative_to(NEW).as_posix()for p in load(NEW/'manifest.json')['source_files']]
    paths.remove('nav/rtl/mac/rgmii_mac100.v')
    paths += [p for p in source_paths if Path(p).name=='rgmii_mac100.v']
    assert len(paths)==12
    unchanged={p:(OLD/p).read_bytes()==(NEW/p).read_bytes()for p in paths}
    assert all(unchanged.values()),unchanged
    def block(root,path,start,end=None):
        t=(root/path).read_text();i=t.index(start)
        return t[i:]if end is None else t[i:t.index(end,i)]
    assert block(OLD,'control/mem_policy.v','    reg [31:0] sensor_age;',
                 '    wire capture,frame_valid,history_initialized;')==block(NEW,'control/mem_policy.v','    reg [31:0] sensor_age;',
                 '    wire capture,frame_valid,history_initialized;')
    sequential='    always @(posedge clk or negedge rst_n) begin'
    assert block(OLD,'control/rl_policy_tick_v5.v',sequential)==block(NEW,'control/rl_policy_tick_v5.v',sequential)
    assert 'localparam integer CLK_HZ = 50000000;'in(NEW/'g2_top.v').read_text()
    return dict(passed=True,unchanged_RTL=unchanged,mem_policy_sensor_stop_block_exact=True,
                policy_all_sequential_blocks_exact=True,unchanged_ring_producer_source=True,
                source_50MHz_125Hz_contract=True,
                limits='Actual clock/26/IR/lease and ring boundaries are rechecked. Action mapper, NAV and PID implementation have separate reviewers; no complete mapped-core proof.')


def source():
    maps=[];assets=[]
    for root in (OLD,PREP,NEW):
        paths=[Path(p).relative_to(root).as_posix()for p in load(root/'manifest.json')['source_files']]
        assert len(paths)==len(set(paths))==63
        maps.append({p:root/p for p in paths})
        assets.append({p.relative_to(root).as_posix():p for p in root.rglob('*')if p.is_file()and'impl'not in p.relative_to(root).parts and p.suffix.lower()in{'.mem','.hex','.mif','.bin'}})
    a,p,b=maps;assert a.keys()==p.keys()==b.keys()
    changed={n for n in a if a[n].read_bytes()!=b[n].read_bytes()}
    assert changed==CHANGED,(changed,CHANGED)
    assert all(p[n].read_bytes()==b[n].read_bytes()for n in b)
    assert assets[0].keys()==assets[1].keys()==assets[2].keys()and len(assets[0])==104
    assert all(assets[0][n].read_bytes()==assets[1][n].read_bytes()==assets[2][n].read_bytes()for n in assets[0])
    assert len([n for n in assets[0]if n.startswith('src/mem/weight9_')])==35
    assert (NEW/'firmware/nav_fw.bin').stat().st_size==53572
    preparation=load(PREP/'measured_timing_source_preparation.json')
    assert preparation['passed']and set(preparation['changed_RTL_relative_SYN44'])==CHANGED
    assert preparation['source_hardware_inputs']==63
    stop=source_contract()
    return dict(passed=True,RTL=63,changed_RTL=sorted(changed),other58_RTL_byte_identical=True,
                actual_all63_RTL_byte_identical_to_selected_candidate=True,all104_assets_byte_identical=True,
                packed_weight_assets=35,FW_BIN_bytes=53572,stop_ring_source_contract=stop,
                preparation=meta(PREP/'measured_timing_source_preparation.json'),
                source_bindings=[meta(v)for v in b.values()],actual_VG=meta(NEW/'impl/gwsynthesis/g2_top.vg'))


def worker(stage,out):
    if stage=='ring_memory':
        import review_ring_stride32_syn44 as r
        r.OLD=OLD;r.NEW=NEW;r.PREP=PREP;r.CHANGED=CHANGED
        r.OLD_OBSERVER_ADDRESS_DSPS=0;r.source=source
        return r.run(out)
    if stage=='clock_stop':
        import review_slf128_syn42_clock_stop as r
        r.OLD=OLD;r.NEW=NEW;r.OUT=out;r.source_contract=source_contract
        code=r.run()
        cdc=out/'actual_original26_and_CDC52.json'
        if cdc.exists():
            actual=load(cdc)
            if actual.get('passed'):
                rows=actual['rows'];assert len(rows)==26
                reference=BASE/'SYN44_clock_CDC_stop_review1/actual_original26_and_CDC52.json'
                oldrows={(x['group'],x.get('bit')):x for x in load(reference)['rows']}
                op=r.parameter_reader(r.definitions(OLD/'impl/gwsynthesis/g2_top.vg'))
                np=r.parameter_reader(r.definitions(NEW/'impl/gwsynthesis/g2_top.vg'))
                parameter_rows=[]
                for x in rows:
                    previous=oldrows[(x['group'],x.get('bit'))]
                    for role in ('source','first','second'):
                        a,b=previous[role],x[role]
                        assert a['kind']==b['kind'],(x['group'],x.get('bit'),role,a,b)
                        pa,pb=op(a),np(b)
                        assert set(pa)<= {'INIT'} and set(pb)<= {'INIT'},(a,pa,b,pb)
                        default=1 if b['kind']in ('DFFPE','DFFSE')else 0
                        ia,ib=r.literal(pa.get('INIT',default)),r.literal(pb.get('INIT',default))
                        assert ia==ib,(x['group'],x.get('bit'),role,pa,pb)
                        parameter_rows.append(dict(group=x['group'],bit=x.get('bit'),role=role,
                            before_instance=a['name'],after_instance=b['name'],kind=b['kind'],
                            before_parameters=pa,after_parameters=pb,effective_INIT=ib))
                paths=[dict(source_pin=x['source_pin'],first_pin=x['first_pin'],
                            budget_ns=x['budget_ns'],group=x['group'],bit=x.get('bit'),
                            source_clock=x['source_clock'],target_clock=x['target_clock'],
                            source_instance=x['source']['name'],first_instance=x['first']['name'],
                            second_instance=x['second']['name'],
                            source_register=x['source']['name'],first_register=x['first']['name'],
                            route_max_delay_ns=x['budget_ns'])for x in rows]
                assert len({(x['source_pin'],x['first_pin'])for x in paths})==26
                groups=dict(Counter(x['group']for x in paths))
                assert groups==dict(rx_write_gray=12,core_read_gray=12,rx_clock_monitor=1,rx_sampling_select=1),groups
                save(out/'actual_roles26.json',dict(passed=True,actual_netlist=meta(NEW/'impl/gwsynthesis/g2_top.vg'),
                    paths=paths,group_counts=groups,all78_role_kind_INIT_parameters_equal=True,
                    role_parameters=parameter_rows,parameter_reference=meta(reference),origin=str(cdc),
                    role_scope='Actual SYN45 sourceQ-to-firstD, original26 connectivity/parameters and firstQ single secondD load revalidated; physical delays/Actived/MTBF remain pending.'))
        return code
    raise AssertionError(stage)


def run(out):
    out.mkdir(parents=True,exist_ok=True)
    me=psutil.Process();state=dict(controller_pid=me.pid,controller_creation=me.create_time(),
            at=stamp(),phase='claimed',old=str(OLD),new=str(NEW),prepared=str(PREP),
            no_EDA_started=True,automatic_PNR=False,passed=False)
    # The exclusive claim makes an already-started audit non-repeatable by
    # another copy. A failed audit is preserved and needs a new output path.
    with (out/'run_claim.json').open('x',encoding='utf8')as f:json.dump(state,f,ensure_ascii=False,indent=2)
    def phase(name,**kw):state.update(phase=name,at=stamp(),**kw);save(out/'job.json',state)
    try:
        from retry_frozen_synthesis import memory_status
        save(out/'plan.json',dict(old=str(OLD),new=str(NEW),prepared=str(PREP),
            workers=['ring_memory','clock_stop'],sequential_worker_graphs=True,
            exact_changed_RTL=sorted(CHANGED),memory_minimum_GiB=dict(physical=2,commit=4),
            automatic_EDA=False,action_NAV_PID_mapping_excluded=True,
            proof_scope='Actual new storage parameters/INIT/CPU image, ring address/cursor/data/control boundary, every clockpin, original26 and IR/lease next/output partitions; not whole core or physical acceptance.'))
        while True:
            build=load(NEW/'job.json')
            if build['phase']=='synthesis_complete':assert build['tool_exit']==0;break
            assert not build['phase'].startswith(('failed','cancel')),build
            phase('waiting_natural_synthesis',actual_synthesis_phase=build['phase'],tool_pid=build.get('tool_pid'))
            time.sleep(15)
        phases={}
        for stage in ('ring_memory','clock_stop'):
            while True:
                m=memory_status()
                if m.avail_phys>=2*GIB and m.avail_page>=4*GIB:break
                phase('waiting_review_memory',next_worker=stage,free_physical_bytes=m.avail_phys,available_commit_bytes=m.avail_page)
                time.sleep(15)
            command=[sys.executable,'-B','-X','utf8',str(Path(__file__).resolve()),'--worker',stage,'--out',str(out/stage)]
            stdout=out/(stage+'.stdout.log')
            with stdout.open('w',encoding='utf8')as stream:
                child=subprocess.Popen(command,stdout=stream,stderr=subprocess.STDOUT,cwd=str(Path(__file__).resolve().parent))
                cp=psutil.Process(child.pid)
                phase('review_worker_running',worker=stage,child_pid=child.pid,child_creation=cp.create_time(),command=command,stdout=str(stdout),free_physical_bytes=m.avail_phys,available_commit_bytes=m.avail_page)
                code=child.wait()
            result=load(out/stage/'result.json')if(out/stage/'result.json').exists()else{}
            phases[stage]=dict(exit_code=code,passed=code==0 and result.get('passed')is True,result=str(out/stage/'result.json'))
            phase('review_worker_complete',worker=stage,workers=phases)
            # Continue to the other independent section even after a localized
            # mapping exception; the failed result and stdout stay intact.
        passed=all(p['passed']for p in phases.values())
        state.update(passed=passed,phase='review_complete'if passed else'failed_preserved',workers=phases,
            physical_unknown=True,deployment_accepted=False,completed_at=stamp())
        save(out/'result.json',state);save(out/'job.json',state)
        print(json.dumps(state,ensure_ascii=False));return 0 if passed else 1
    except Exception:
        state.update(phase='failed_preserved',error=traceback.format_exc(),at=stamp())
        save(out/'result.json',state);save(out/'job.json',state);print(state['error']);return 1


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--out',type=Path,default=DEFAULT_OUT)
    p.add_argument('--worker',choices=['ring_memory','clock_stop'])
    a=p.parse_args()
    raise SystemExit(worker(a.worker,a.out)if a.worker else run(a.out))
