"""Bind the actual LF packed data lanes and merged trig read ports locally.

No synthesis, simulation, or full G2 graph is resident.  This reader rejects
changes instead of accepting the observed counts without their data/port map.
"""
import argparse
import json
from pathlib import Path
import re

from audit_netlist_memories import PARAM
from audit_routing_topology import primitive_directions
from prove_soc_console_remap import Logic, literal, z3
from review_stage3_slf128_memories import LIBRARY, Tree, meta, init_vector, library_defaults
from synthesis_connectivity import read


def main():
    ap=argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--local',type=Path,required=True)
    ap.add_argument('--prepared',type=Path,required=True)
    ap.add_argument('--out',type=Path,required=True)
    ap.add_argument('--run-prefetch-trace',type=Path,help='Required passed actual LF trace for a repaired run-prefetch module')
    a=ap.parse_args();a.out.mkdir(parents=True,exist_ok=False)
    r=dict(passed=False,automatic_PNR=False,collision_gate_passed=False)
    try:
        inputs=[a.local/(n+'.v') for n in ('slf_lf','slf_match','traj_eval','nav_ram_sdp_25','nav_ram_sdp_28','nav_ram_sdp_20')]+[a.prepared/'sin_lut.hex',LIBRARY]
        r['inputs']=[meta(p) for p in inputs]
        def local_graph(module):
            p=a.local/(module+'.v');raw=p.read_text(encoding='utf8')
            view=a.out/(module+'_parser_view.v');view.write_text(raw.replace('module '+module+'(','module g2_top(',1),encoding='utf8')
            cells=read(view);tree=Tree(view)
            return cells,tree
        cells,tree=local_graph('slf_lf')
        repaired_trace=None
        if a.run_prefetch_trace:
            repaired_trace=json.loads(a.run_prefetch_trace.read_text());assert repaired_trace['passed']and repaired_trace['run_prefetch_required']
            assert repaired_trace['actual_module']==meta(a.local/'slf_lf.v')
        rows={n:c for n,c in cells.items() if c['kind']=='SDPB' and n.startswith('lb1_lb1_')}
        assert set(rows)=={'lb1_lb1_0_0_s','lb1_lb1_0_1_s','lb1_lb1_0_2_s'}
        lf_write_proofs=[]
        if repaired_trace:
            dirs=primitive_directions(LIBRARY);qsyms={q:z3.Bool('LF_actual_FF:'+q)for c in cells.values()if c['kind'].startswith('DFF')for q in c['ports']['Q']}
            lf_logic=Logic(cells,lambda c:tree.params[c['name']],dirs,qsyms,{})
            expected_write=z3.And(z3.Not(lf_logic.expr('r2')),lf_logic.expr('v1'),z3.Not(lf_logic.expr('c1[8]')))
        def bus(n,h,l=0):return [f'{n}[{i}]' for i in range(h,l-1,-1)]
        expected_do=[bus('l2',12)+bus('l1',18),bus('DO',31,25)+bus('l3',18)+bus('l2',18,13),bus('DO_0',31,12)+bus('l4',18,10)+bus('DO_0',2)]
        cur=['q_occ']+bus('sm_q',13,11)+['VCC']+bus('sm_q',10,8)+['VCC']+bus('sm_q',17,8)
        assert len(cur)==19
        expected_di=[bus('l1',12)+cur,bus('l3',6)+bus('l2',18)+bus('l1',18,13),['GND']*20+bus('l3',18,7)]
        for i,n in enumerate(sorted(rows)):
            c=rows[n];p=tree.params[n]
            assert c['ports']['DO']==expected_do[i],('Packed output lane changed',n)
            assert c['ports']['DI']==expected_di[i],('Packed input lane changed',n)
            assert c['ports']['CLKA']==c['ports']['CLKB']==['clk50_d']
            if repaired_trace is None:assert c['ports']['CEA']==['lb1_17']and c['ports']['CEB']==['VCC']
            else:
                s=z3.Solver();s.add(lf_logic.expr(c['ports']['CEA'][0])!=expected_write);answer=s.check();assert answer==z3.unsat,('Actual repaired LF write control differs',n,str(answer))
                lf_write_proofs.append(dict(primitive=n,CEA=c['ports']['CEA'],expected='!reset&&v1&&!c1[8]',miter='UNSAT'))
                proof=[p for p in repaired_trace['run_prefetch_actual_array_gates']if p['primitive']==n];assert len(proof)==1 and proof[0]['CEB']==c['ports']['CEB']and proof[0]['read_active_equals_not_reset_and_run']=='UNSAT'
            assert c['ports']['OCE']==c['ports']['RESET']==['GND']
            assert c['ports']['ADA']==['GND']+bus('c1',7)+['GND']+['VCC']*4
            assert c['ports']['ADB']==['GND']+bus('c'if repaired_trace else'c1_c',7)+['GND']*5
            assert literal(p['BIT_WIDTH_0'])==literal(p['BIT_WIDTH_1'])==32 and literal(p['READ_MODE'])==0
            v,bits=init_vector('SDPB',p,library_defaults());assert v==0 and bits==16384
        r['LF_packing']=dict(passed=True,actual_blocks=3,width=32,source_rows=4,source_words_per_row=130,source_cell_bits=19,retained_observable_bits_per_column=66,retained_observable_bits_total=8580,
            lane_relations=['l1[18:0] <- cur[18:0]','l2[18:0] <- l1[18:0]','l3[18:0] <- l2[18:0]','l4[18:10] <- l3[18:10]'],
            discarded_bits='Source l4[9:0] are never consumed by the LF computation; actual spare payload bits are unobserved',
            native_word_capacity=512,actual_column_range=[0,129],address_bits='AD[13:5]',byte_write_masks='ADA[4:1]=1111, not address bits',
            read_control='CEB constant1: reachable collision proof is separate and required'if repaired_trace is None else'Actual CEB/BLKSEL equals !reset&&run;16900scan+drain has0chunks conflicting, bound separate actual trace')
        if repaired_trace:r['LF_write_control']=lf_write_proofs
        # Bind every retained LF output bit to its corresponding physical
        # write bit; checking all66 defeats a capacity/count-only relaxation.
        for target,source in [('l1',cur),('l2',bus('l1',18)),('l3',bus('l2',18)),('l4',bus('l3',18,10))]:
            for q,d in zip(bus(target,18,10 if target=='l4' else 0),source):
                found=[(n,c['ports']['DI'][j]) for n,c in rows.items() for j,nq in enumerate(c['ports']['DO']) if nq==q]
                assert len(found)==1 and found[0][1]==d,('Missing or mispacked observable LF bit',q,d,found)
        defaults=library_defaults();trig=[]
        expected=[int(s,16)&65535 for s in (a.prepared/'sin_lut.hex').read_text().split()];assert len(expected)==1024
        for parent,wrapper,sinidx,cosidx,sinout,cosout,readmode in [('slf_match','nav_ram_sdp_25','lut_si','lut_ci','sn','cs',1),('traj_eval','nav_ram_sdp_28','si','ci','sq','cq',0)]:
            cc,tt=local_graph(wrapper);rom={n:c for n,c in cc.items() if c['kind']=='pROM'};assert len(rom)==2
            addresses=[bus(sinidx,9),bus(cosidx,9,8)+bus(sinidx,7)]
            outputs=[sinout,cosout]
            for i,(n,c) in enumerate(sorted(rom.items())):
                p=tt.params[n]
                assert literal(p['BIT_WIDTH'])==16 and literal(p['READ_MODE'])==readmode
                assert c['ports']['AD']==addresses[i]+['GND','GND','VCC','VCC']
                assert c['ports']['DO'][-16:]==bus(outputs[i],15)
                assert c['ports']['CLK']==['clk50_d'] and c['ports']['CE']==['lut_re'] and c['ports']['RESET']==['GND']
                assert c['ports']['OCE']==(['n13179_4'] if readmode else ['GND'])
                v,bits=init_vector('pROM',p,defaults);assert bits==16384
                assert [(v>>(16*j))&65535 for j in range(1024)]==expected
            # The actual parent binding makes two independent addresses and
            # outputs despite both ROM leaves now living under u_cos.
            pc,pt=local_graph(parent)
            bind=[c for c in pc.values() if c['kind']==wrapper];assert len(bind)==1 and bind[0]['name']=='u_cos'
            for pin in ('clk50_d','lut_re',sinidx,cosidx,sinout,cosout):
                nets=bind[0]['ports'][pin]
                width=10 if pin==sinidx else 2 if pin==cosidx else 16 if pin in (sinout,cosout) else 1
                assert nets==(bus(pin,9,8) if pin==cosidx else bus(pin,width-1) if width>1 else [pin])
            directions=primitive_directions(LIBRARY)
            for c in pc.values():directions.setdefault(c['kind'],{})
            qs={q:z3.Bool('actual_FF:'+q) for c in pc.values() if c['kind'].startswith('DFF') for q in c['ports']['Q']}
            logic=Logic(pc,lambda c:pt.params[c['name']],directions,qs,{})
            if parent=='slf_match':
                # The synthesis tool absorbed original sn/cs captures into
                # the pROM output register. CE still captures lut_re data;
                # OCE must equal the exact old S_T0 && j==0 capture event.
                st=z3.And(*[logic.expr(f'st[{i}]')==bool(7>>i&1) for i in range(5)])
                jzero=z3.And(*[z3.Not(logic.expr(f'j[{i}]')) for i in range(13)])
                want=z3.And(z3.Not(logic.expr('r2')),st,jzero)
                solver=z3.Solver();solver.set(timeout=30000);solver.add(logic.expr('n13179_4')!=want);answer=solver.check()
                assert answer==z3.unsat,('Retimed match trig output capture differs',str(answer),str(solver.model()) if answer==z3.sat else '')
                retime=dict(proved=True,method='Exact actual Boolean miter UNSAT',old_capture='!reset && st==S_T0(7) && j==0',new_capture='Both actual pROM OCE=n13179_4; public READ_MODE1 pl_reg<=bp_reg semantics',old_external_sn_cs_FFs_absorbed=True)
                # The same legal output-register absorption occurs for the
                # touch bitmap's utb_q capture in S_U1(19).
                tc,ttouch=local_graph('nav_ram_sdp_20');trams=[c for c in tc.values()if c['kind']=='SDPB'];assert len(trams)==1
                tr=trams[0];tp=ttouch.params[tr['name']]
                assert literal(tp['READ_MODE'])==1 and literal(tp['BIT_WIDTH_0'])==literal(tp['BIT_WIDTH_1'])==1
                assert tr['ports']['CEB']==['tb_re'] and tr['ports']['OCE']==['n14001_4'] and tr['ports']['RESET']==['GND'] and tr['ports']['DO'][-1]=='utb_q'
                touchwant=z3.And(z3.Not(logic.expr('r2')),*[logic.expr(f'st[{i}]')==bool(19>>i&1)for i in range(5)])
                solver=z3.Solver();solver.set(timeout=30000);solver.add(logic.expr('n14001_4')!=touchwant);answer=solver.check()
                assert answer==z3.unsat,('Retimed touch output capture differs',str(answer))
                touchbind=pc['u_touch']['ports'];assert touchbind['tb_re']==['tb_re'] and touchbind['n14001_4']==['n14001_4'] and touchbind['utb_q']==['utb_q']
                r['touch_output_retiming']=dict(passed=True,actual_wrapper='nav_ram_sdp_20',READ_MODE=1,CEB='tb_re',OCE='n14001_4',method='Exact actual Boolean miter UNSAT: OCE == !reset && st==S_U1(19)',public_consumer='Original utb_q <= tb_q absorbed into SDPB pl_reg; actual output bound directly to utb_q')
            else:retime=dict(proved=True,method='Direct same CE/address/read_mode0 output port binding; no added pipeline')
            trig.append(dict(parent=parent,actual_wrapper=wrapper,actual_blocks=2,read_modes=[readmode,readmode],all_2x1024x16_INIT_words_match=True,independent_sin_cos_addresses_and_outputs_bound=True,retiming=retime))
        r.update(passed=True,trig=trig,scope='Actual data/port/INIT and consumer retiming mapping; whole-G2 physical/caller acceptance remains separate')
        assert r['inputs']==[meta(p) for p in inputs]
    except Exception as e:r.update(error=f'{type(e).__name__}: {e}')
    (a.out/'result.json').write_text(json.dumps(r,ensure_ascii=False,indent=2)+'\n',encoding='utf8');print(json.dumps(r,ensure_ascii=False))
    return 0 if r['passed'] else 1


if __name__=='__main__':raise SystemExit(main())
