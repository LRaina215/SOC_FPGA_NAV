"""Strict actual joint RAM/INIT/read-gate snapshot after the two local repairs.

One process owns the full graph and exits before any clock/CDC parser starts.
Local LF control and public weight numerical checks remain separate required
proofs; source or small-module passes are not substituted for the new joint VG.
"""
import argparse
import json
from pathlib import Path
import re
import traceback
import xml.etree.ElementTree as ET
from audit_netlist_memories import BSRAM
from audit_routing_topology import primitive_directions
from prove_soc_console_remap import Logic, literal, z3
from review_stage3_slf128_memories import (CORE, GROUPS, LIBRARY, Tree, meta, library_defaults, canonical, cpu_image, init_vector, word_width)
from synthesis_connectivity import read


def main():
    ap=argparse.ArgumentParser(description=__doc__)
    for n in('prepared','vg','reference_memories','out'):ap.add_argument('--'+n.replace('_','-'),type=Path,required=True,dest=n)
    a=ap.parse_args();a.out.mkdir(parents=True,exist_ok=False);r=dict(passed=False,complete=False,whole_storage_gate_passed=False,automatic_PNR=False)
    def save(): (a.out/'result.json').write_text(json.dumps(r,ensure_ascii=False,indent=2)+'\n',encoding='utf8')
    try:
        bindings=[meta(p)for p in(a.prepared/'manifest.json',a.prepared/'firmware/nav_fw.bin',a.vg,a.reference_memories,LIBRARY)]
        manifest=json.loads((a.prepared/'manifest.json').read_text());paths=[Path(p).relative_to(a.prepared)for p in manifest['source_files']];assert len(paths)==63
        build=a.vg.parents[2]
        assert all((a.prepared/p).read_bytes()==(build/p).read_bytes()for p in paths),'Actual joint sources are not the selected frozen63RTL'
        defaults=library_defaults();tree=Tree(a.vg);old=json.loads(a.reference_memories.read_text())
        mem={n:dict(kind=k,parameters=tree.params[n])for n,k in tree.kinds.items()if k in BSRAM}
        (a.out/'actual_memories.json').write_text(json.dumps(mem,indent=2)+'\n')
        wide=[n for n,c in mem.items()if word_width(c['parameters'])in(32,36)]
        r.update(inputs=bindings,actual_source_BSRAM=len(mem),actual_wide_leaves=len(wide),split1_physical_prediction=len(mem)+len(wide),physical_prediction_not_measured=True,
                 synthesis_resources=ET.parse(build/'impl/gwsynthesis/g2_top_syn_rsc.xml').getroot().attrib)
        assert len(mem)==269 and len(wide)==11,('Unexpected new joint RAM mapping',len(mem),len(wide))
        changed_prefixes=('u_nav/u_match/u_lf/',CORE+'/weights/')
        unchanged=[]
        assert set(old)==set(mem),('New joint RAM leaf set changed',sorted(set(mem)-set(old)),sorted(set(old)-set(mem)))
        for n,c in mem.items():
            if n.startswith(changed_prefixes):continue
            x=old[n];assert c['kind']==x['kind']and canonical(c['kind'],c['parameters'],defaults)==canonical(x['kind'],x['parameters'],defaults),('Unmodified RAM geometry or INIT changed',n)
            unchanged.append(n)
        r['unchanged_RAM_vs_actual42_all_geometry_INIT']=unchanged
        rawmem={n:(c['kind'],c['parameters'])for n,c in mem.items()};image=cpu_image(rawmem,defaults);binary=(a.prepared/'firmware/nav_fw.bin').read_bytes();assert len(binary)==53572
        assert image==binary+bytes(131072-len(binary))
        assert image==b''.join(int(s,16).to_bytes(4,'little')for s in(a.prepared/'ram_w.hex').read_text().split())
        for i in range(4):assert image[i::4]==bytes(int(s,16)for s in(a.prepared/f'ram_b{i}.hex').read_text().split())
        r['CPU_INIT']=dict(passed=True,bitplanes=64,bytes=131072,BIN_bytes=len(binary),selected_BIN_and_all_lanes_word_exact=True)
        lf={n:c for n,c in mem.items()if n.startswith('u_nav/u_match/u_lf/')};assert len(lf)==3
        for n,c in lf.items():
            p=c['parameters'];assert c['kind']=='SDPB'and literal(p['BIT_WIDTH_0'])==literal(p['BIT_WIDTH_1'])==32 and literal(p['READ_MODE'])==0
            assert init_vector(c['kind'],p,defaults)[0]==0
        files={p.name:[int(s,16)for s in p.read_text().split()]for pattern in('weight9_low*.mem','weight9_high*.mem')for p in(a.prepared/'src/mem').glob(pattern)}
        weight={n:c for n,c in mem.items()if n.startswith(CORE+'/weights/')};assert len(weight)==35;bound={}
        for n,c in weight.items():
            p=c['parameters'];width=word_width(p);assert width in(8,9)and c['kind']in('pROM','pROMX9')and literal(p['READ_MODE'])==0
            data,bits=init_vector(c['kind'],p,defaults);words=[(data>>(i*width))&((1<<width)-1)for i in range(2048)]
            match=[f for f,w in files.items()if w==words and f.startswith('weight9_low'if width==9 else'weight9_high')]
            assert len(match)==1 and match[0]not in bound.values(),('Missing/duplicated actual weight INIT payload',n,match);bound[n]=match[0]
        assert set(bound.values())==set(files)
        r['weight35_INIT_payloads']=bound
        # Extract only the actual modules needed by following local proofs.
        local=a.out/'local_modules';local.mkdir();selected={tree.module_kinds[p]for p in('u_nav/u_match/u_lf',CORE+'/weights','u_nav/u_match','u_nav/u_tr')}
        selected.update(tree.module_kinds[p]for p in('u_nav/u_match/u_touch','u_nav/u_match/u_cos','u_nav/u_tr/u_cos'))
        modules={}
        for name in sorted(selected):
            p=local/(name+'.v');p.write_text('module '+name+'('+tree.modules[name]+'endmodule\n',encoding='utf8');modules[name]=meta(p)
        r['local_modules']=modules;save()
        # The new actual CEB/BLKSEL must still gate physical reads.  This full
        # graph is released at process exit; no second parser is launched here.
        cells=read(a.vg);directions=primitive_directions(LIBRARY);qs={q:z3.Bool('FF:'+q)for c in cells.values()if c['kind'].startswith('DFF')for q in c['ports']['Q']}
        supported=lambda k:bool(re.fullmatch(r'LUT[1-6]',k))or k.startswith('MUX2_LUT')or k in('ALU','INV','IBUF','BUFG','VCC','GND')or k.startswith('DFF')
        opaque={(n,p,i):z3.Bool(f'opaque:{n}:{p}:{i}')for n,c in cells.items()if not supported(c['kind'])for p,ns in c['ports'].items()if directions[c['kind']].get(p)=='output'for i in range(len(ns))}
        logic=Logic(cells,lambda c:tree.params[c['name']],directions,qs,opaque)
        def active(c,sel):
            suffix='A'if sel=='0'else'B';pin='CE'+suffix;p={**defaults[c['kind']],**tree.params[c['name']]};want=literal(p['BLK_SEL_'+sel]);ns=c['ports']['BLKSEL'+suffix]
            return z3.And(logic.expr(c['ports'][pin][0]),*[logic.expr(n)==bool(want>>(len(ns)-1-i)&1)for i,n in enumerate(ns)])
        gates=[]
        for g in GROUPS:
            if'read_gate'not in g:continue
            nets=tree.boundary[g['read_gate']][g['read_pin']];assert len(nets)==1;re_=logic.expr(nets[0])
            for n,c in cells.items():
                if not n.startswith(g['prefix'])or c['kind']not in('SDPB','SDPX9B'):continue
                rd=active(c,'1');s=z3.Solver();s.set(timeout=30000);s.add(rd,z3.Not(re_));answer=s.check();assert answer==z3.unsat,('New actual array reads outside logical request',n,str(answer))
                gates.append(dict(primitive=n,logical_read_pin=g['read_pin'],logical_read_net=nets[0],array_read_implies_logical_re='UNSAT',CEA=c['ports']['CEA'],CEB=c['ports']['CEB'],BLKSELA=c['ports']['BLKSELA'],BLKSELB=c['ports']['BLKSELB'],ADA=c['ports']['ADA'],ADB=c['ports']['ADB'],OCE=c['ports']['OCE'],READ_MODE=literal(tree.params[n]['READ_MODE'])))
        assert len(gates)==32
        r.update(complete=True,passed=True,actual_32_array_read_gate_bindings=gates,actual_32_CEB_BLKSEL_read_gates_passed=True,
                 pending=['Actual joint LF run-prefetch/read-disable/data packing and complete drain trace','Actual joint packed-weight public40704 numeric reads/idle holds; small C proof alone not inherited','Consumer/caller reachability bindings and all physical PNR/timing/CDC/reset/IO/board gates'],
                 scope='All269 actual RAM leaf sets/geometry/INIT, new CPU128KiB image,35weight payloads and32true array read gates; not full data/collision/public weight acceptance')
        assert bindings==[meta(p)for p in(a.prepared/'manifest.json',a.prepared/'firmware/nav_fw.bin',a.vg,a.reference_memories,LIBRARY)]
    except Exception as e:r.update(error=f'{type(e).__name__}: {e}',traceback=traceback.format_exc())
    save();print(json.dumps({k:v for k,v in r.items()if k not in('unchanged_RAM_vs_actual42_all_geometry_INIT','weight35_INIT_payloads','actual_32_array_read_gate_bindings','local_modules')},ensure_ascii=False));return 0 if r['passed']else 1


if __name__=='__main__':raise SystemExit(main())
