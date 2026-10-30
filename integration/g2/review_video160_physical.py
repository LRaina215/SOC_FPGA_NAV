"""Close physical RAM write-data replicas and summarize measured timing."""
import argparse,json,re
from collections import Counter
from pathlib import Path
from datetime import datetime
from synthesis_connectivity import PORT,read
from audit_netlist_memories import PARAM
from revalidate_lidar_cdc import definitions
from review_small_block_stores import parameter_reader
from review_slf128_pair_physical_memories import review
from audit_global_muxes import audit

def save(p,v):p.write_text(json.dumps(v,ensure_ascii=False,indent=2)+'\n',encoding='utf-8')
def norm(s):return re.sub(r'\s+','',s).lstrip('\\')

def run(source,physical,out):
    assert not out.exists();out.mkdir();mem=out/'memories';mem.mkdir()
    memory=review(source,physical,mem,expected_source_count=307,expected_physical_count=318,expected_BIN_bytes=54164)
    text=(physical/'impl/pnr/g2_top.vo').read_text(encoding='utf-8')
    pattern=re.compile(r'^(DFF\w*)\s+(\\\S+|[^\s(]+)\s*\(\s*\n(.*?)\)\s*;',re.M|re.S)
    cells={}
    for kind,raw,body in pattern.findall(text):
        n=raw.lstrip('\\');ports={p:norm(e)for p,e in PORT.findall(body)}
        if n.startswith('u_nav/wd_q_')or ports.get('Q','').startswith('u_nav/gowin_reg_wd_q'):
            cells[n]=dict(kind=kind,ports=ports)
    params={n:{}for n in cells}
    for n,k,v in PARAM.findall(text):
        n=n.strip().lstrip('\\')
        if n in params:params[n][k]=v.strip()
    original=read(source/'impl/gwsynthesis/g2_top.vg',keep=lambda n:n=='u_nav'or n.startswith('u_nav/wd_q_'))
    sp=parameter_reader(definitions(source/'impl/gwsynthesis/g2_top.vg'))
    assert len(original)==32
    for n,c in original.items():
        p=cells[n]
        assert c['kind']==p['kind']and sp(c)==params[n]
        assert {k:[v]for k,v in p['ports'].items()}==c['ports'],('original data FF changed',n)
    replicas=[];aliases={}
    for name,c in cells.items():
        m=re.search(r'gowin_reg_wd_q\[(\d+)\]',c['ports']['Q'])
        if not m:continue
        bit=int(m[1]);old=next(n for n in original if re.fullmatch(r'u_nav/wd_q_'+str(bit)+r'_s\d+',n));a=cells[old]
        assert c['kind']==a['kind']and params[name]==params[old]
        assert {k:v for k,v in c['ports'].items()if k!='Q'}=={k:v for k,v in a['ports'].items()if k!='Q'},name
        assert c['ports']['Q']not in aliases
        aliases[c['ports']['Q']]=a['ports']['Q']
        replicas.append(dict(copy=name,original=old,bit=bit,kind_parameters_and_every_transition_input_equal=True))
    assert {x['bit']for x in replicas}==set(range(32))
    raw=json.loads((mem/'actual_BSRAM_cells.json').read_text(encoding='utf-8'))
    closed=[];remaining=[]
    for row in memory['all_rows']:
        failed={k for k,v in row['checks'].items()if not v}
        if not failed:continue
        assert failed=={'DI_halves_reassembled'},row
        src=raw['source'][row['source']];lo=raw['physical'][row['source']];hi=raw['physical'][row['added']]
        half=row['physical_width_each'];actual=hi['ports']['DI'][-half:]+lo['ports']['DI'][-half:]
        canonical=[aliases.get(norm(n),norm(n))for n in actual]
        assert canonical==[norm(n)for n in src['ports']['DI']],row['source']
        closed.append(dict(memory=row['source'],issue='Physical replicated write-data FF names',all_DI_bits_same_registered_value=True))
    storage_pass=all(memory[k]for k in ('counts_match','inputs_unchanged_after_review','all_distributed_INIT_preserved'))
    storage_pass=storage_pass and all(memory['CPU128KiB']['checks'].values())and len(closed)==sum(not all(x['checks'].values())for x in memory['all_rows'])
    assert storage_pass
    gate=json.loads((physical/'g2_gate.json').read_text(encoding='utf-8'))
    coverage_raw=json.loads((physical/'setup_endpoint_coverage.json').read_text(encoding='utf-8'))
    coverage,ends=coverage_raw if isinstance(coverage_raw,list)else (coverage_raw,json.loads((physical/'negative_setup_endpoints.json').read_text()))
    def category(n):
        if '/hud/page_'in n:return 'video_HUD_payload'
        if n.startswith('u_video/'):return 'video_synchronizer'
        if n.startswith('u_nav/bus_rdata_'):return 'NAV_MMIO_read'
        if '/observation/'in n:return 'observation_scaling'
        if n.startswith('u_soc/'):return 'CPU_read_data'
        if '/wheel_control/'in n:return 'wheel_PID'
        return 'other'
    buckets=Counter(category(x['to_node'])for x in ends)
    globals=audit(physical);save(out/'global_allocation.json',globals)
    result=dict(recorded_at=datetime.now().astimezone().isoformat(),source=str(source),physical=str(physical),
        routing_completed=gate['routing_passed'],explicit_unrouted_count=gate['unrouted_nets'],physical_BSRAM=318,
        physical_storage_and_CPU_INIT_closed=True,raw_memory_checker=str(mem/'result.json'),
        physical_wd_replicas_proved=len(replicas),source_data_bits=32,replica_input_closures=closed,
        setup=gate['setup_violations'],hold=gate['hold_violations'],clocks=gate['clocks'],
        per_clock=gate['per_clock_violations'],outside_per_clock=gate['violations_outside_per_clock_summary'],
        setup_coverage=coverage,listed_failed_endpoint_categories=dict(buckets),
        worst_core_path=next(x for x in ends if x['from_clock']==x['to_clock']),
        hold_paths=gate['negative_reported_hold_paths'],reset_checks=gate['reported_reset_pulse_checks'],
        camera_PR1014_warning='PR1014'in(physical/'gw_sh.log').read_text(),
        camera_primary_also_reported=any(x['net']=='CAMERA_PCLK_d'and x['resource']=='PRIMARY'for x in globals['global_nets']),
        complete_CDC_reset_IO_and_timing_signoff=False,programmed=False,deployment_accepted=False)
    save(out/'write_data_replicas.json',dict(passed=True,rows=replicas));save(out/'result.json',result)
    return result

if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--source',type=Path,required=True);p.add_argument('--physical',type=Path,required=True);p.add_argument('--out',type=Path,required=True)
    a=p.parse_args();r=run(a.source,a.physical,a.out);print(json.dumps({k:r[k]for k in ('routing_completed','physical_BSRAM','physical_storage_and_CPU_INIT_closed','setup','hold','listed_failed_endpoint_categories')}))
