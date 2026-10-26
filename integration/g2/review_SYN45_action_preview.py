"""Read actual SYN45 six action-DSP modes and input/register ownership.

Small partition only: actuator-map, genuine core action FFs and shared parent
combinational cells. No EDA, netlist rewriting or old object-name assumptions.
"""
from pathlib import Path
from datetime import datetime
from collections import Counter,defaultdict
import argparse,html,json,re,traceback
import psutil
from audit_routing_topology import primitive_directions
from revalidate_lidar_cdc import definitions
from review_small_block_stores import parameter_reader
from synthesis_connectivity import read
from prove_soc_console_remap import Logic,literal,z3
from prepare_baseline_fusion_retime import metadata,save
from retry_frozen_synthesis import memory_status

BASE=Path('F:/nav_integration_g2_20261023');REPO=Path(__file__).resolve().parents[2]
BUILD=BASE/'SLF128_measured_timing_syn45';SRC=BASE/'SLF128_measured_timing_SYN45_prepared1'
OVERLAY=BASE/'SLF128_ring44_action_preview_overlay2'
PARENT='u_control/model_thread/';MAP=PARENT+'actuator_map/';CORE=PARENT+'policy/core/'
LIB=Path('D:/Gowin2/Gowin_V1.9.12.04_x64/IDE/simlib/gw5a/prim_sim.v')
GAINS=[19,-19,-600,19,19,600]

def collect(out):
    r=json.loads((out/'result.json').read_text());assert r['passed']
    assert metadata(Path(r['actual_vg']['path']))==r['actual_vg']
    assert metadata(Path(r['DSP_modes_result']['path']))==r['DSP_modes_result']
    rpt=BUILD/'impl/gwsynthesis/g2_top_syn.rpt.html';t=rpt.read_text()
    def clean(s):return' '.join(html.unescape(re.sub('<[^>]+>',' ',s)).split())
    paths=[]
    for number,table in re.findall(r'<h3>Path(?:&nbsp;?|\s)+(\d+)</h3>.*?<b>Path Summary:.*?<table[^>]*>(.*?)</table>',t,re.S):
        rows=re.findall(r'<tr>\s*<td[^>]*>(.*?)</td>\s*<td[^>]*>(.*?)</td>\s*</tr>',table,re.S)
        d={clean(k):clean(v)for k,v in rows};d['Path']=int(number);paths.append(d)
    assert len(paths)==25 and paths[0]['Data Arrival Time']=='20.829'and paths[0]['Slack']=='-0.515'
    freq=re.search(r'Max Frequency Summary:.*?<table[^>]*>(.*?)</table>',t,re.S)[1]
    frequency=[[clean(x)for x in re.findall(r'<td[^>]*>(.*?)</td>',row,re.S)]for row in re.findall(r'<tr>(.*?)</tr>',freq,re.S)]
    frequency=[x for x in frequency if x]
    timing=dict(report=metadata(rpt),frequency_rows=frequency,listed_path_count=len(paths),paths=paths,
      all_listed_action_path_count=sum('actuator_map'in(x.get('From','')+x.get('To',''))for x in paths),
      warning='SYN estimates only;25listedpaths are not allSTA endpoints. RX table target50MHz is a synthesis-report convention, physicalSDC remains100MHz; no signoff claim')
    save(out/'SYN45_estimated_timing_snapshot.json',timing)
    summary=dict(r,actual_result=metadata(out/'result.json'),timing_summary=timing,
      controller_natural_exit_required_before_EDA_admission=True,
      unchanged_source_map_and_E2_E3_basis='Ring44overlay2 exact3sourcebinding; nooldSYN28netlist/CST imported')
    save(REPO/'docs/4_design/evidence/g2_20261023/SYN45_action_preview_actual_20261027.json',summary)
    print(json.dumps(dict(passed=True,result=str(out/'result.json'),six_DSPs=6,UNSAT=12,
      listed_paths=len(paths),action_paths_listed=timing['all_listed_action_path_count'])))

def main():
    ap=argparse.ArgumentParser(description=__doc__);ap.add_argument('--out',type=Path,default=BASE/'SYN45_action_preview_actual_review1')
    ap.add_argument('--collect-only',action='store_true');a=ap.parse_args()
    if a.collect_only:collect(a.out);return
    assert not a.out.exists();a.out.mkdir();state=dict(passed=False,phase='starting_actual_partition_review',
      controller_pid=psutil.Process().pid,controller_created_epoch=psutil.Process().create_time(),
      actual6_product_registers_verified=False,actual_source_core_actions_verified=False,
      EDA_started=False,scope='actual six product modes/FF input boundary + exact source proof binding, not whole mapped G2 equivalence or physical timing')
    save(a.out/'job.json',state)
    try:
        job=json.loads((BUILD/'job.json').read_text());assert job['phase']=='synthesis_complete'and job['tool_exit']==0
        try:assert abs(psutil.Process(job['tool_pid']).create_time()-job['tool_created_epoch'])>.02,'original SYN45 still alive'
        except psutil.NoSuchProcess:pass
        mem=memory_status();assert mem.avail_phys>=2*1024**3 and mem.avail_page>=4*1024**3,('read-only admission',mem)
        state['admission']=dict(physical=mem.avail_phys,commit=mem.avail_page)
        closure=json.loads((OVERLAY/'prepared_review.json').read_text());edge=json.loads((OVERLAY/'edge_relation_proof.json').read_text())
        assert closure['passed']and edge['passed']and all(c['result']=='unsat'for c in edge['checks'])
        for n in closure['modified_RTL']:assert(SRC/n).read_bytes()==(OVERLAY/n).read_bytes(),('current three-source binding',n)
        vg=BUILD/'impl/gwsynthesis/g2_top.vg';before=metadata(vg)
        def keep(n):
            ancestor=MAP.startswith(n+'/')or CORE.startswith(n+'/')
            direct_parent=n.startswith(PARENT)and'/'not in n[len(PARENT):]
            return ancestor or n.startswith(MAP)or n.startswith(CORE+'action')or direct_parent
        cells=read(vg,keep=keep);params=parameter_reader(definitions(vg));dirs=primitive_directions(LIB)
        drivers=defaultdict(list)
        for n,c in cells.items():
            for p,ns in c['ports'].items():
                if dirs[c['kind']].get(p)=='output':
                    for net in ns:
                        if net not in('GND','VCC'):drivers[net].append((n,p))
        def roots(ns):
            seen=set();found=set();pending=list(ns)
            while pending:
                net=pending.pop()
                if net in seen or net in('GND','VCC'):continue
                seen.add(net);d=drivers.get(net,[]);assert len(d)==1,('unresolved actual used cone',net,d)
                n,p=d[0];c=cells[n]
                if c['kind'].startswith('DFF'):found.add(n);continue
                assert re.fullmatch('LUT[1-6]',c['kind'])or c['kind']in('ALU','INV','BUF','IBUF','BUFG'),(n,c['kind'])
                for pin,more in c['ports'].items():
                    if dirs[c['kind']].get(pin)=='input':pending.extend(more)
            return sorted(found)
        dsps={n:c for n,c in cells.items()if n.startswith(MAP)and c['kind'].startswith('MULT')}
        assert len(dsps)==6 and all(c['kind']=='MULTALU27X18'for c in dsps.values()),[(n,c['kind'])for n,c in dsps.items()]
        library=LIB.read_text();macro=re.search(r'\bmodule MULTALU27X18\b.*?\bendmodule',library,re.S)[0]
        defaults={k:v.strip()for k,v in re.findall(r'\bparameter\s+(\w+)\s*=\s*([^;]+);',macro)}
        assert defaults and all(s in macro for s in('assign mult_out = a0 * b0','alu_out = acc_load + m_out0 + c_sig + d_casi','out_reg_sync <= alu_out','assign DOUT = d_out[47:0]'))
        qsymbols={c['ports']['Q'][0]:z3.Bool('actualQ:'+n)for n,c in cells.items()if c['kind'].startswith('DFF')}
        logic=Logic(cells,params,dirs,qsymbols,{})
        def vec(ns):return z3.Concat(*[z3.If(logic.expr(n),z3.BitVecVal(1,1),z3.BitVecVal(0,1))for n in ns])
        checks=[]
        def equal(label,l,r):
            s=z3.Solver();s.set(timeout=30000);s.add(l!=r);status=s.check()
            checks.append(dict(label=label,status=str(status)));assert status==z3.unsat,(label,status,str(s.model())if status==z3.sat else s.reason_unknown())
        rows=[]
        for n,c in dsps.items():
            explicit=params(c);assert set(explicit)<=set(defaults),(n,set(explicit)-set(defaults))
            p={**defaults,**explicit};ports=c['ports']
            assert all(p[k]=='"FALSE"'for k in p if k.startswith('DYN_'))
            assert all(literal(p[k])==0 for k in('P_SEL','P_ADDSUB','A_SEL','ACC_SEL','CASI_SEL','ADD_SUB_0','ADD_SUB_1','PRE_LOAD'))
            assert p['MULT12X12_EN']==p['FB_PREG_EN']==p['SOA_PREG_EN']=='"FALSE"'
            assert all(v==('"CLK0"'if k=='OREG_CLK'else'"BYPASS"')for k,v in p.items()if k.endswith('_CLK'))
            assert p['OREG_CE']=='"CE0"'and p['OREG_RESET']=='"RESET0"'
            assert ports['CLK']==['GND','clk50_d']and ports['CE'][-1]=='VCC'and set(ports['RESET'])=={'GND'}
            assert set(ports['C']+ports['D']+ports['CASI']+ports['SIA'])=={'GND'}
            assert all(set(ports[k])=={'GND'}for k in('ACCSEL','PSEL','ASEL','PADDSUB','CSEL','CASISEL','ADDSUB'))
            r=roots([x for key in('A','B','C','D')for x in ports[key]])
            indices=set();bits={}
            for ff in r:
                match=re.fullmatch(re.escape(CORE)+r'action([0-5])_([0-9]+)_s[0-9]+',ff)
                assert match,('not corresponding genuine core FF',n,ff)
                indices.add(int(match[1]));bits[int(match[2])]=cells[ff]['ports']['Q'][0]
                assert cells[ff]['ports']['CLK']==['clk50_d']
            assert len(indices)==1 and set(bits)==set(range(24)),(n,indices,sorted(bits))
            index=next(iter(indices));gain=GAINS[index]
            actual_action=vec([bits[i]for i in range(23,-1,-1)])
            clipped=z3.If(actual_action>4096,z3.BitVecVal(4096,24),z3.If(actual_action< -4096,z3.BitVecVal(-4096,24),actual_action))
            aa,bb=vec(ports['A']),vec(ports['B'])
            expected=z3.SignExt(40,clipped)*z3.BitVecVal(gain,64)
            actual=z3.SignExt(37,aa)*z3.SignExt(46,bb)
            equal('action%d all2^24 original signed clip×gain'%index,actual,expected)
            # The primitive stores48bits, while RTL product is signed64. No
            # significant bit is lost; upper bits must be sign extension.
            equal('action%d signed48 DSP product losslessly extends original64'%index,z3.SignExt(16,z3.Extract(47,0,actual)),expected)
            variables={str(x)for pin in('A','B')for net in ports[pin]for x in z3.z3util.get_vars(logic.expr(net))}
            assert variables<={str(qsymbols[q])for q in bits.values()},('unexplained external state',n,variables)
            rows.append(dict(name=n,index=index,gain=gain,kind=c['kind'],explicit_parameters=explicit,effective_parameters=p,
              ports=ports,input_actual_FF_boundaries=r,core_action_actual_Qs=bits,
              actual_product48_matches_original64=True,OREG='actual clk50_d/CE1/reset0',all_other_pipeline_stages_bypassed=True,
              late_registered_policy_actions_used=False))
        assert sorted(x['index']for x in rows)==list(range(6))
        product_FF=[n for n,c in cells.items()if n.startswith(MAP+'product_')and c['kind'].startswith('DFF')]
        assert not product_FF,product_FF
        assert metadata(vg)==before
        save(a.out/'actual_six_DSP_modes.json',dict(DSPs=rows,checks=checks,library=metadata(LIB),small_partition_cells=len(cells)))
        state.update(passed=True,phase='actual_action_product_partition_passed',actual6_product_registers_verified=True,
          actual_source_core_actions_verified=True,actual_signed_clip_gain_modes_verified=True,
          source_edge_contract_preserved=True,actual_separate_product_FF_count=0,DSP_count=6,all_action_bits_per_group=1<<24,
          mapped_math_UNSAT_checks=len(checks),small_partition_cells=len(cells),action_primitive_counts=dict(Counter(c['kind']for n,c in cells.items()if n.startswith(MAP))),
          actual_vg=metadata(vg),DSP_modes_result=metadata(a.out/'actual_six_DSP_modes.json'),
          source_proof=metadata(OVERLAY/'prepared_review.json'),edge_proof=metadata(OVERLAY/'edge_relation_proof.json'),
          current_source_bindings=[metadata(SRC/n)for n in closure['modified_RTL']],
          source_rounding_and_output_edge='currentmap exactly overlay2/oldprovenmap; source19UNSAT edge proof binds captureE→consumeE+1/latestff/publicreset-stop. Remaining LUT-rounding fullmapped-output partition not separatelyprovedhere',
          no_new_clock_or_public_pipeline_stage=True,full_mapped_core_or_wholeG2_equivalence=False,
          actual_50MHz_physical_STA_or_routability_proven=False)
    except Exception as e:state.update(phase='strict_actual_review_failed_preserved',error=str(e),traceback=traceback.format_exc())
    state['ended_at']=datetime.now().astimezone().isoformat();save(a.out/'result.json',state);save(a.out/'job.json',state)
    print(json.dumps({k:state.get(k)for k in('passed','actual6_product_registers_verified','actual_source_core_actions_verified','mapped_math_UNSAT_checks','error')}))

if __name__=='__main__':main()
