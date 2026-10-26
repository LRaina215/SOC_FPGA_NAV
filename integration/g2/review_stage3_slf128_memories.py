"""Prepare, then audit the selected SLF128/storage synthesis memory contract.

No EDA or simulation is launched.  --prepare-only records expectations.  Given
an actual VG, this checks storage/INITs and actual SDP array enables (CEB and
BLKSELB, deliberately not OCE).  A satisfiable collision query is a remaining
reachable-state proof obligation, never silently accepted as safe or reported
as an observed hardware collision.  Physical PNR acceptance stays separate.
"""
import argparse
from collections import Counter
from functools import lru_cache
import json
from pathlib import Path
import re

from audit_netlist_memories import BSRAM, MODULE, PARAM
from synthesis_connectivity import CELL, DECL, TOKEN, read


DEFAULT_PREPARED = Path('F:/nav_integration_g2_20261023/stage3_SLF128_storage_RTL_prepared5')
DEFAULT_VG = Path('F:/nav_integration_g2_20261023/SLF128_storage_safeFW_syn41/impl/gwsynthesis/g2_top.vg')
DEFAULT_REFERENCE = Path('F:/nav_integration_g2_20261023/baseline_fusion50_ring_MAX_syn40/impl/gwsynthesis/g2_top.vg')
LIBRARY = Path('D:/Gowin2/Gowin_V1.9.12.04_x64/IDE/simlib/gw5a/prim_sim.v')
CORE = 'u_control/model_thread/policy/core'
# Physical predictions are for the currently selected split1 option only.
GROUPS = [
    dict(name='structure128', prefix='u_nav/u_match/u_sm/', count=16, widths=[18], init='zero', read_gate='u_nav/u_match/u_sm',read_pin='sm_re'),
    dict(name='touch128', prefix='u_nav/u_match/u_touch/', count=1, widths=[1], init='zero', read_gate='u_nav/u_match/u_touch',read_pin='tb_re'),
    dict(name='point_planes', prefix='u_nav/u_match/u_pb/', count=8, widths=[18], init='zero', read_gate='u_nav/u_match/u_pb',read_pin='ac_re'),
    dict(name='bitmap', prefix='u_nav/u_map/u_tv/u_bm/', count=1, widths=[1], init='zero', read_gate='u_nav/u_map/u_tv/u_bm',read_pin='bm_re'),
    dict(name='TXshadow', prefix='u_nav/u_mbx/u_txs/', count=1, widths=[32], init='zero', read_gate='u_nav/u_mbx/u_txs',read_pin='mb_shadow_read'),
    dict(name='slot_list', prefix='u_nav/u_match/u_slist/', count=1, widths=[16,18], init='zero', read_gate='u_nav/u_match/u_slist',read_pin='sl_re'),
    *[dict(name='trig_'+p.replace('/','_'), prefix=p+'/', count=2, widths=[16], init='sin_lut.hex')
      for p in ('u_nav/u_match/u_cos','u_nav/u_tr/u_cos')],
    *[dict(name=f'console{i}', prefix=f'u_soc/g_con[{i}].u_con/', count=1, widths=[8], init='zero', read_gate=f'u_soc/g_con[{i}].u_con',read_pin='con_read_needed') for i in range(4)],
    dict(name='weight9', prefix=CORE+'/weights/', count=35, widths=[8,9], init='packed35'),
    dict(name='LF_rows', prefix='u_nav/u_match/u_lf/', count=3, widths=[32], init='zero'),
]
CHANGED_PREFIXES = [g['prefix'] for g in GROUPS] + ['u_soc/g_lane[','u_nav/u_match/u_sin/','u_nav/u_tr/u_sin/']


def meta(p):
    s=Path(p).stat();return dict(path=str(p),bytes=s.st_size,mtime_ns=s.st_mtime_ns)


def literal(s):
    s=s.strip();m=re.fullmatch(r"(?:\d+)'([bhd])([0-9a-fA-F]+)",s)
    if m:return int(m[2],dict(b=2,h=16,d=10)[m[1]])
    return int(s)


class Tree:
    def __init__(self, path):
        self.path=path;self.modules=dict(MODULE.findall(path.read_text(encoding='utf8')))
        self.params={};self.kinds={};self.boundary={}
        self.walk('g2_top','',{})

    @lru_cache(None)
    def definition(self,kind):
        body=self.modules[kind]
        assert not re.search(r'^\s*assign\s',body,re.M),'Unsupported netlist assigns require explicit alias review'
        decl={n.lstrip('\\'):(d,int(h) if h else None,int(l) if l else None) for d,h,l,n in DECL.findall(body)}
        param={}
        for n,k,v in PARAM.findall(body):param.setdefault(n.strip().lstrip('\\'),{})[k]=v.strip()
        instances=[(t,n.lstrip('\\'),p) for t,n,p in CELL.findall(body)]
        return decl,param,instances

    def walk(self,kind,hierarchy,bindings):
        from synthesis_connectivity import PORT
        decl,params,instances=self.definition(kind)
        def local(n):return n if n in ('GND','VCC') else bindings.get(n,(hierarchy+'/' if hierarchy else '')+n)
        def indices(h,l):return range(h,l+(1 if l>=h else -1),1 if l>=h else -1)
        def expand(expr):
            result=[];pos=0
            while pos<len(expr):
                if not expr[pos:].strip():break
                m=TOKEN.match(expr,pos);assert m,('Unsupported expression',kind,expr[pos:]);pos=m.end()
                width,base,value,name,hi,lo,punct=m.groups()
                if punct:continue
                if width:
                    assert not re.search('[xXzZ]',value)
                    v=int(value,dict(b=2,h=16,d=10)[base]);result+=['VCC' if v>>i&1 else 'GND' for i in range(int(width)-1,-1,-1)]
                else:
                    name=name.lstrip('\\')
                    if hi is not None:result += [local(f'{name}[{i}]') for i in indices(int(hi),int(lo or hi))]
                    elif name in decl and decl[name][1] is not None:result += [local(f'{name}[{i}]') for i in indices(*decl[name][1:])]
                    else:result.append(local(name))
            return result
        for child,n,p in instances:
            full=(hierarchy+'/'+n).strip('/');ports={pin.lstrip('\\'):expand(e) for pin,e in PORT.findall(p)}
            if child in self.modules:
                self.boundary[full]=ports
                cd,_,_=self.definition(child);mapped={}
                for pin,nets in ports.items():
                    if pin in cd and cd[pin][1] is not None:
                        ids=list(indices(*cd[pin][1:]));assert len(ids)==len(nets)
                        mapped.update({f'{pin}[{i}]':v for i,v in zip(ids,nets)})
                    else:
                        assert len(nets)==1,('Unknown bus port',full,pin);mapped[pin]=nets[0]
                self.walk(child,full,mapped)
            else:self.kinds[full]=child;self.params[full]=params.get(n,{})


def library_defaults():
    result={}
    for kind,body in re.findall(r'\bmodule\s+(\w+)\b(.*?)\bendmodule',LIBRARY.read_text(),re.S):
        result[kind]={k:v.strip() for k,v in re.findall(r'\bparameter\s+(\w+)\s*=\s*([^;]+);',body)}
    return result


def init_vector(kind,p,defaults):
    d=defaults[kind];keys=sorted(k for k in d if k.startswith('INIT_RAM_'))
    assert len(keys)==64,('Unsupported memory INIT geometry',kind)
    value=0;shift=0
    for k in keys:
        raw=p.get(k,d[k]);m=re.fullmatch(r"(\d+)'h([0-9a-fA-F]+)",raw);assert m,(kind,k,raw)
        bits=int(m[1]);value|=int(m[2],16)<<shift;shift+=bits
    return value,shift


def word_width(p):
    return max(literal(p[k]) for k in ('BIT_WIDTH','BIT_WIDTH_0','BIT_WIDTH_1') if k in p)


def canonical(kind,p,defaults):
    d=dict(defaults[kind]);d.update(p)
    return {k:(v if v.startswith('"') else literal(v)) for k,v in d.items()}


def cpu_image(mem,defaults):
    image=bytearray(131072);seen=set()
    for n,(kind,p) in mem.items():
        if not n.startswith('u_soc/g_lane['):continue
        m=re.fullmatch(r'u_soc/g_lane\[([0-3])\]\.u_ram/mem_mem_([01])_([0-7])_s',n)
        assert m and kind=='SP' and literal(p['BIT_WIDTH'])==1,('Unreviewed CPU bitplane mapping',n)
        lane,bank,bit=map(int,m.groups());assert (lane,bank,bit) not in seen;seen.add((lane,bank,bit))
        v,bits=init_vector(kind,p,defaults);assert bits==16384
        for j in range(16384):image[(bank*16384+j)*4+lane]|=((v>>j)&1)<<bit
    assert len(seen)==64
    return bytes(image)


def prepare(prepared):
    composition=json.loads((prepared/'composition_review.json').read_text())
    assert composition['SLF_LOG2N']==7 and composition['source_files']==63
    budget=composition['source_budget']['totals'];assert budget==dict(source=270,wide=12,physical=282)
    return dict(preparation_passed=True,actual_mapping_reviewed=False,expected_source_BSRAM=270,
                expected_wide_split_leaves=12,predicted_physical_BSRAM=282,
                prediction_only=True,migration_groups=GROUPS,
                retained_distributed=['u_nav/u_mbx/u_injb','u_nav/u_map/u_tv/u_ct','u_nav/u_match/u_cand','u_nav/u_match/u_res'],
                gaps=['No new VG yet: actual counts/geometry/INIT/read enables unknown',
                      'Selected rebuilt BIN/lane files exist only as source evidence until reconstructed from actual VG INITs',
                      'An OCE hold never proves physical array read suppression',
                      'If collision SAT with free actual FF Qs, source-specific reachable-state invariants remain required; no assumptions or old fixture PASS are silently substituted',
                      'Packed-weight INIT matching does not by itself prove read address/output recombination; mapped arithmetic/address cones need separate review',
                      'Point-plane geometry/INIT and read-gate relation do not prove full ownership FSM mapping',
                      'After PNR recheck split pairs, every physical INIT, actual timing/CDC/reset/IO; SYN cannot supply CLS/global-resource acceptance'],
                automatic_PNR=False,deployment_accepted=False)


def audit(prepared,vg,reference,binary,output,result,local_review,LF_trace):
    from audit_routing_topology import primitive_directions
    from prove_soc_console_remap import Logic,z3
    build=vg.parents[2]
    manifest=json.loads((prepared/'manifest.json').read_text())
    hardware=[Path(p).relative_to(prepared) for p in manifest['source_files']]
    assert len(hardware)==len(set(hardware))==63
    assert all((prepared/p).read_bytes()==(build/p).read_bytes() for p in hardware),'Actual synthesis inputs differ from selected frozen63 RTL'
    defaults=library_defaults();tree=Tree(vg);old=Tree(reference)
    mem={n:(k,tree.params[n]) for n,k in tree.kinds.items() if k in BSRAM}
    oldmem={n:(k,old.params[n]) for n,k in old.kinds.items() if k in BSRAM}
    wide=[n for n,(k,p) in mem.items() if word_width(p) in (32,36)]
    result.update(actual_source_BSRAM=len(mem),actual_wide_leaves=len(wide),physical_split1_prediction=len(mem)+len(wide))
    (output/'actual_memories.json').write_text(json.dumps({n:dict(kind=k,parameters=p) for n,(k,p) in mem.items()},indent=2)+'\n')
    # The one-leaf difference is accepted only after all66 observable LF bits
    # and both independent trig read ports have been bound to this actual VG.
    lr=json.loads((local_review/'result.json').read_text());assert lr['passed'] and lr['LF_packing']['actual_blocks']==3
    for binding in lr['inputs']:
        assert binding==meta(Path(binding['path'])),('Local mapping input changed',binding['path'])
    for p in local_review.glob('*_parser_view.v'):
        original=p.name.removesuffix('_parser_view.v')
        body=dict(MODULE.findall(p.read_text(encoding='utf8')))['g2_top']
        assert body==tree.modules[original],('Local module proof is not this actual VG',original)
    assert len(mem)==269 and len(wide)==11,('Unexpected actual mapping totals',len(mem),len(wide))
    result['count_difference_proof']=dict(source_prediction=270,actual=269,reason='LF four logical19-bit rows pack three32-bit leaves because only66 output bits are consumed',local_review=meta(local_review/'result.json'),all_retained_LF_lanes_and_trig_read_ports_bound=True)
    checked=[]
    for n,(k,p) in oldmem.items():
        if any(n.startswith(prefix) for prefix in CHANGED_PREFIXES):continue
        assert n in mem and mem[n][0]==k,('Original memory missing/retyped',n)
        assert canonical(k,p,defaults)==canonical(k,mem[n][1],defaults),('Original memory geometry/INIT changed',n)
        checked.append(n)
    result['original_unchanged_BSRAM_geometry_and_INIT']=checked
    image=cpu_image(mem,defaults);binbytes=binary.read_bytes();assert len(binbytes)<=131072
    assert image==binbytes+bytes(131072-len(binbytes)), 'CPU INIT is not selected new BIN plus zero padding'
    words=b''.join(int(x,16).to_bytes(4,'little') for x in (prepared/'ram_w.hex').read_text().split());assert words==image
    for i in range(4):assert bytes(int(x,16) for x in (prepared/f'ram_b{i}.hex').read_text().split())==image[i::4]
    result['new_CPU_image']=dict(passed=True,bytes=131072,lanes=4,bitplanes=64,binary=meta(binary),BIN_bytes=len(binbytes))
    for prefix in result['retained_distributed']:
        assert not any(n.startswith(prefix+'/') for n in mem),('Unexpected retained store promoted',prefix)
    group_results=[];covered=set(checked)|{n for n in mem if n.startswith('u_soc/g_lane[')}
    packed_files={p.name:[int(x,16)for x in p.read_text().split()] for pattern in ('weight9_low*.mem','weight9_high*.mem') for p in (prepared/'src/mem').glob(pattern)}
    packed_bound={}
    for g in GROUPS:
        leaves={n:(k,p) for n,(k,p) in mem.items() if n.startswith(g['prefix'])}
        covered.update(leaves)
        assert len(leaves)==g['count'],('Wrong group geometry/count',g['name'],len(leaves))
        for n,(k,p) in leaves.items():
            assert word_width(p) in g['widths'],('Wrong memory width',n,p)
            v,bits=init_vector(k,p,defaults)
            if g['init']=='zero':assert v==0,('Nonzero new initialized RAM',n)
            elif g['init']=='sin_lut.hex':
                expected=[int(x,16)&65535 for x in (prepared/'sin_lut.hex').read_text().split()]
                assert len(expected)==1024 and word_width(p)==16
                assert [(v>>(i*16))&65535 for i in range(1024)]==expected,('Trig INIT differs',n)
            elif g['init']=='packed35':
                # The tool can place two distinct payloads under the first
                # bank's instance name; a leaf-name-to-file guess is unsafe.
                width=word_width(p);assert width in (8,9)
                words=[(v>>(i*width))&((1<<width)-1)for i in range(2048)]
                matches=[file for file,values in packed_files.items()if values==words and file.startswith('weight9_low'if width==9 else'weight9_high')]
                assert len(matches)==1,('Actual packed INIT has no unique exact source file',n,matches)
                file,=matches;assert file not in packed_bound.values(),('Packed payload duplicated instead of retained',file)
                packed_bound[n]=file
        group_results.append(dict(name=g['name'],leaves=list(leaves),geometry_INIT_passed=True))
    bias={n:(k,p) for n,(k,p) in mem.items() if n.startswith(CORE+'/bias_mem_')}
    covered.update(bias)
    assert len(bias)==1,'Expected one native wide bias leaf before split'
    for n,(k,p) in bias.items():
        width=word_width(p);assert width in (32,36)
        v,bits=init_vector(k,p,defaults);expected=[int(x,16)&((1<<26)-1) for x in (prepared/'src/mem/bias_narrow.mem').read_text().split()]
        assert len(expected)==425 and [(v>>(i*width))&((1<<26)-1) for i in range(425)]==expected
    assert covered==set(mem),('Unaccounted actual BSRAM',sorted(set(mem)-covered))
    assert len(packed_bound)==len(packed_files)==35 and set(packed_bound.values())==set(packed_files)
    result['packed_weight_35_INIT_payload_binding']=packed_bound
    result['packed_weight_public_semantics']=dict(passed=False,official_primitive_counterexample='F:/nav_integration_g2_20261023/SLF128_actual_weights_official_primitive_probe1/xsim.log',address=16385,original_and_source_RTL='0176',actual_mapped='3976',scope='Exact INIT contents alone do not prove the actual CE/read-hold mapping')
    result['changed_group_geometry_INIT']=group_results
    # Actual array read gating.  All FF Qs and opaque data outputs are free;
    # the consumer FSM's reachability is NOT presumed by the solver.
    cells=read(vg);directions=primitive_directions(LIBRARY)
    qs={q:z3.Bool('actual_FF:'+q) for c in cells.values() if c['kind'].startswith('DFF') for q in c['ports'].get('Q',[])}
    supported=lambda k:bool(re.fullmatch(r'LUT[1-6]',k)) or k.startswith('MUX2_LUT') or k in ('ALU','INV','IBUF','BUFG','VCC','GND') or k.startswith('DFF')
    opaque={(n,p,i):z3.Bool(f'actual_opaque:{n}:{p}:{i}') for n,c in cells.items() if not supported(c['kind']) for p,ns in c['ports'].items() if directions[c['kind']].get(p)=='output' for i in range(len(ns))}
    logic=Logic(cells,lambda c:tree.params[c['name']],directions,qs,opaque)
    def vector(nets):return [logic.expr(n) for n in nets]
    def block(c,p,suffix):
        want=literal(p.get('BLK_SEL_'+('0' if suffix=='A' else '1'),defaults[c['kind']]['BLK_SEL_'+('0' if suffix=='A' else '1')]))
        nets=c['ports']['BLKSEL'+suffix];return z3.And(*[b==bool(want>>(len(nets)-1-i)&1) for i,b in enumerate(vector(nets))])
    gate_results=[];gaps=[dict(store=CORE+'/weights',kind='Actual official-primitive public weight mismatch',missing='Correct crossed-enable high-ROM merge mapping; original42 counterexample preserved')]
    for g in GROUPS:
        if 'read_gate' not in g:continue
        endpoint=tree.boundary[g['read_gate']][g['read_pin']];assert len(endpoint)==1
        logical_re=logic.expr(endpoint[0])
        for n,(k,p) in mem.items():
            if not n.startswith(g['prefix']):continue
            assert k in ('SDPB','SDPX9B'),('Unreviewed writable primitive',n,k)
            c=cells[n];full=dict(defaults[k]);full.update(p)
            readmode=literal(full['READ_MODE']);assert readmode in(0,1),('Unsupported actual read mode',n,readmode)
            if readmode==1:gaps.append(dict(store=n,kind='Actual output-register absorption requires consumer capture binding',missing='Bind actual OCE/READ_MODE1 pl_reg to original consumer FF; physical CEB query is still evaluated below'))
            read_active=z3.And(logic.expr(c['ports']['CEB'][0]),block(c,p,'B'))
            solver=z3.Solver();solver.set(timeout=30000);solver.add(read_active,z3.Not(logical_re));status=solver.check()
            assert status==z3.unsat,('Actual CEB/BLKSELB can read when logical re is false; OCE alone is insufficient',n,str(status))
            write_active=z3.And(logic.expr(c['ports']['CEA'][0]),block(c,p,'A'))
            ada=c['ports']['ADA'];adb=c['ports']['ADB'];assert len(ada)==len(adb)==14
            def chunks(addr,width):
                # UG300 and the installed public SDPB model: a32/36-bit word
                # spans two18-bit physical chunks; AD[4:1] are write masks.
                # Narrow modes remain conservatively grouped at AD[13:4].
                bits=vector(addr)
                if width in (32,36):return [bits[:9]+[z3.BoolVal(False)],bits[:9]+[z3.BoolVal(True)]]
                assert width in (1,2,4,8,9,16,18),('Unreviewed SDP width',width)
                return [bits[:10]]
            wa=literal(full['BIT_WIDTH_0']);rb=literal(full['BIT_WIDTH_1'])
            same_chunk=z3.Or(*[z3.And(*[x==y for x,y in zip(a,b)]) for a in chunks(ada,wa) for b in chunks(adb,rb)])
            solver=z3.Solver();solver.set(timeout=30000);solver.add(write_active,read_active);simultaneous=solver.check()
            assert simultaneous!=z3.unknown,('Port-activity query unknown',n,solver.reason_unknown())
            if simultaneous==z3.unsat:collision=simultaneous;method='Actual write/read enables mutually exclusive, address independent'
            else:
                solver.add(same_chunk);collision=solver.check();method='Actual physical chunk-set intersection, mode aware'
            assert collision!=z3.unknown,('Collision query unknown',n,solver.reason_unknown())
            row=dict(name=n,actual_CEB=c['ports']['CEB'],actual_BLKSELB=c['ports']['BLKSELB'],actual_OCE=c['ports']['OCE'],logical_re=endpoint[0],readactive_implies_re='UNSAT',physical_chunk='32/36: {AD[13:5],0/1}; narrower: conservative AD[13:4]',write_width=wa,read_width=rb,method=method,collision_with_all_actual_state_free=str(collision))
            gate_results.append(row)
            if collision==z3.sat:gaps.append(dict(store=n,missing='Bound actual caller reachable-state/one-writer ownership invariant; free-state SAT is not an observed transaction or permission to ignore collision'))
    trace=json.loads((LF_trace/'result.json').read_text());assert trace['trace_complete']
    assert trace['actual_module']==meta(Path(trace['actual_module']['path']))
    lfbody=dict(MODULE.findall(Path(trace['actual_module']['path']).read_text(encoding='utf8')))['slf_lf']
    assert lfbody==tree.modules['slf_lf'],'LF control trace is not this actual mapped module'
    result['LF_reachable_control_trace']=dict(report=meta(LF_trace/'result.json'),complete_scan_cycles=trace['scan_run_cycles'],collision_events=trace['collision_events'])
    if trace['collision_events']:gaps.append(dict(store='u_nav/u_match/u_lf/three packed rows',kind='Actual mapped finite reachable collision',events=trace['collision_events'],missing='Explicit array read suppression on the final write drain edge; OCE/output unused is not sufficient'))
    result.update(actual_array_read_gates=gate_results,remaining_collision_proof_obligations=gaps,actual_mapping_reviewed=True,
                  storage_INIT_and_read_gate_passed=True,passed=not gaps,PNR_preflight_storage_gate_passed=not gaps,
                  automatic_PNR=False,deployment_accepted=False)


def main():
    ap=argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--prepared',type=Path,default=DEFAULT_PREPARED)
    ap.add_argument('--reference-vg',type=Path,default=DEFAULT_REFERENCE)
    ap.add_argument('--vg',type=Path,default=DEFAULT_VG);ap.add_argument('--bin',type=Path)
    ap.add_argument('--prepare-only',action='store_true');ap.add_argument('--out',type=Path,required=True)
    ap.add_argument('--local-review',type=Path,default=Path('F:/nav_integration_g2_20261023/SLF128_local_memory_mapping_review1'))
    ap.add_argument('--LF-trace',type=Path,default=Path('F:/nav_integration_g2_20261023/SLF128_LF_actual_control_trace1'))
    a=ap.parse_args();a.out.mkdir(parents=True,exist_ok=False)
    result=dict(passed=False,automatic_PNR=False,deployment_accepted=False)
    try:
        result.update(prepare(a.prepared));result['prepared']=meta(a.prepared/'composition_review.json')
        if a.prepare_only:result['passed']=True;result['scope']='Contract preparation only; no mapped storage or collision acceptance'
        else:
            if a.bin is None:a.bin=a.prepared/'firmware/nav_fw.bin'
            assert a.vg and a.bin,'Actual VG and selected new BIN required'
            result['inputs']=[meta(p) for p in (a.vg,a.reference_vg,a.bin,LIBRARY)]
            audit(a.prepared,a.vg,a.reference_vg,a.bin,a.out,result,a.local_review,a.LF_trace)
            assert result['inputs']==[meta(p) for p in (a.vg,a.reference_vg,a.bin,LIBRARY)],'A mapped review input changed while reading'
    except Exception as e:result.update(passed=False,error=f'{type(e).__name__}: {e}')
    (a.out/'result.json').write_text(json.dumps(result,ensure_ascii=False,indent=2)+'\n',encoding='utf8')
    print(json.dumps({k:v for k,v in result.items() if k not in ('migration_groups','changed_group_geometry_INIT','actual_array_read_gates','original_unchanged_BSRAM_geometry_and_INIT')},ensure_ascii=False))
    return 0 if result['passed'] else 1


if __name__=='__main__':raise SystemExit(main())
