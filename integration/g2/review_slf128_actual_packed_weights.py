"""Check every selected weight against actual merged ROM ports and INITs.

This is a read-only one-clock evaluation of the small actual weight module,
using public pROM/pROMX9 and LUT/FF semantics.  No HDL tool or model substitute
is launched.  Counts/names alone never establish merged ROM equivalence.
"""
import argparse
import json
from pathlib import Path
import re
import traceback

from audit_routing_topology import primitive_directions
from prove_soc_console_remap import Logic, literal, z3
from review_stage3_slf128_memories import LIBRARY, Tree, meta, init_vector, library_defaults
from synthesis_connectivity import read


def main():
    ap=argparse.ArgumentParser(description=__doc__);ap.add_argument('--module',type=Path,required=True);ap.add_argument('--prepared',type=Path,required=True);ap.add_argument('--out',type=Path,required=True);a=ap.parse_args();a.out.mkdir(parents=True,exist_ok=False)
    r=dict(passed=False,automatic_PNR=False)
    try:
        raw=a.module.read_text();assert raw.startswith('module fixed4500_weight_rom(')
        view=a.out/'local_parser_view.v';view.write_text(raw.replace('module fixed4500_weight_rom(','module g2_top(',1))
        tree=Tree(view);cells=read(view);directions=primitive_directions(LIBRARY);defaults=library_defaults()
        roms={n:c for n,c in cells.items()if c['kind']in('pROM','pROMX9')};assert len(roms)==35
        qs={q:z3.Bool('state:'+q)for c in cells.values()if c['kind'].startswith('DFF')for q in c['ports']['Q']}
        opaque={(n,'DO',i):z3.Bool(f'ROM:{n}:{i}')for n,c in roms.items()for i in range(len(c['ports']['DO']))}
        logic=Logic(cells,lambda c:tree.params[c['name']],directions,qs,opaque)
        ff={c['ports']['Q'][0]:c for c in cells.values()if c['kind'].startswith('DFF')}
        roots={};rom_data={};file_matches={};selected=set()
        for n,c in roms.items():
            p=tree.params[n];width=literal(p['BIT_WIDTH']);assert width in(8,9)and literal(p['READ_MODE'])==0
            assert c['ports']['CLK']==['clk50_d']and c['ports']['RESET']==['GND']
            v,bits=init_vector(c['kind'],p,defaults);assert bits==2048*width,('Actual weight INIT capacity',n,bits,width)
            words=[(v>>(width*i))&((1<<width)-1)for i in range(2048)]
            files=list((a.prepared/'src/mem').glob('weight9_low*.mem'if width==9 else'weight9_high*.mem'))
            match=[p for p in files if[int(s,16)for s in p.read_text().split()]==words]
            assert len(match)==1,('Actual weight ROM INIT has no unique original payload',n,[str(p)for p in match])
            assert match[0].name not in selected;selected.add(match[0].name);file_matches[n]=dict(original_payload=meta(match[0]),width=width)
            rom_data[n]=words
            roots[n+':CE']=logic.expr(c['ports']['CE'][0])
            #8/9-bit modes both address2048 words with AD[13:3].
            assert len(c['ports']['AD'])==14
            for i,net in enumerate(c['ports']['AD'][:11]):roots[n+':AD:'+str(13-i)]=logic.expr(net)
        for q,c in ff.items():roots['next:'+q]=logic.next_state(c)
        for i in range(16):roots['value:'+str(i)]=logic.expr(f'weight_q[{i}]')
        # Merged pROM banks can read speculatively every cycle and emulate
        # source read-hold with delayed enables, data-hold FFs and a mux.  An
        # idle *private* FF/ROM hold requirement would reject that valid map.
        # Check the public value through accepted/idle transitions below.
        for i in range(16):
            assert all(not str(x).startswith('external:')for x in z3.z3util.get_vars(roots['value:'+str(i)])),('Output directly depends on changing address/enable',i)
        nodes=[];ids={}
        def add(e):
            key=e.get_id()
            if key in ids:return ids[key]
            ch=[add(x)for x in e.children()];k=e.decl().kind()
            if z3.is_true(e):code='True'
            elif z3.is_false(e):code='False'
            elif not ch:code='env['+repr(str(e))+']'
            elif k==z3.Z3_OP_NOT:code='not x'+str(ch[0])
            elif k in(z3.Z3_OP_AND,z3.Z3_OP_OR):code=(' and 'if k==z3.Z3_OP_AND else' or ').join('x'+str(j)for j in ch)
            elif k==z3.Z3_OP_XOR:code=' != '.join('x'+str(j)for j in ch);assert len(ch)==2
            elif k==z3.Z3_OP_ITE:code=f'x{ch[1]} if x{ch[0]} else x{ch[2]}'
            elif k==z3.Z3_OP_EQ:code=' == '.join('x'+str(j)for j in ch)
            else:raise AssertionError(('Unsupported Boolean node',k,str(e)))
            ids[key]=len(nodes);nodes.append(code);return ids[key]
        rootids={n:add(e)for n,e in roots.items()}
        code='def evaluate(env):\n'+''.join('    x'+str(i)+' = '+x+'\n'for i,x in enumerate(nodes))+'    return ('+','.join('x'+str(i)for i in rootids.values())+',)\n'
        scope={};exec(compile(code,'<actual mapped Boolean DAG>','exec'),scope);evaluate=scope['evaluate'];indices={n:i for i,n in enumerate(rootids)}
        def evaluate_dict(env):return dict(zip(rootids,evaluate(env)))
        oldenv={str(s):bool(literal(tree.params[ff[q]['name']].get('INIT',"1'b0")))for q,s in qs.items()}
        oldenv.update({str(s):False for s in opaque.values()});oldenv['external:n383_8']=True
        expected=[int(s,16)&65535 for s in(a.prepared/'src/mem/weights.mem').read_text().split()];assert len(expected)==40704
        checked=0;idle=0;env=dict(oldenv);last_value=None
        for addr,want in enumerate(expected):
            for enable,address in [(True,addr),(False,addr^65535),(False,(addr+12345)&65535)]:
                env.update({f'external:weight_address[{i}]':bool(address>>i&1)for i in range(16)});env['external:n383_8']=enable
                before=evaluate_dict(env);after=dict(env)
                for q in ff:after['state:'+q]=before['next:'+q]
                for n,c in roms.items():
                    if before[n+':CE']:
                        index=sum(int(before[n+':AD:'+str(i)])<<(i-3)for i in range(3,14))
                        data=rom_data[n][index];width=literal(tree.params[n]['BIT_WIDTH'])
                        for i in range(len(c['ports']['DO'])):after[f'ROM:{n}:{i}']=bool(data>>(len(c['ports']['DO'])-1-i)&1)if i>=len(c['ports']['DO'])-width else False
                out=evaluate_dict(after);got=sum(int(out['value:'+str(i)])<<i for i in range(16))
                last_value=want if enable else last_value
                if got!=last_value:
                    r['mismatch_diagnostic']=dict(address=addr,enable=enable,want=last_value,got=got,
                        actual_FF_after={q:after['state:'+q]for q in ff},
                        active_ROM_reads=[dict(primitive=n,payload=file_matches[n]['original_payload']['path'],index=sum(int(before[n+':AD:'+str(i)])<<(i-3)for i in range(3,14)),value=rom_data[n][sum(int(before[n+':AD:'+str(i)])<<(i-3)for i in range(3,14))])for n in roms if before[n+':CE']])
                    raise AssertionError(('Actual packed weight public read/hold differs',addr,enable,hex(last_value),hex(got)))
                checked+=enable;idle+=not enable;env=after
        r.update(passed=True,actual_module=meta(a.module),public_library=meta(LIBRARY),original_weights=meta(a.prepared/'src/mem/weights.mem'),actual_ROMs=len(roms),all_actual_ROM_payloads_uniquely_bound=file_matches,actual_one_clock_reads_checked=checked,all_40704_original_values_equal=True,actual_logic_and_ROM_input_output_evaluated=True,public_idle_hold_cycles_checked=idle,private_ROM_read_and_delayed_hold_FF_retiming_permitted=True,scope='Finite exact mapped read/hold sequence covering all40704 valid original weight addresses and81408 changing-address idle edges;35actual ROM INIT/address/CE/output recombination and selector capture. Not whole-state formal equivalence, full4500 runtime or physical timing acceptance')
    except Exception as e:r.update(error=f'{type(e).__name__}: {e}',traceback=traceback.format_exc())
    (a.out/'result.json').write_text(json.dumps(r,ensure_ascii=False,indent=2)+'\n');print(json.dumps({k:v for k,v in r.items()if k!='all_actual_ROM_payloads_uniquely_bound'},ensure_ascii=False));return 0 if r['passed']else 1


if __name__=='__main__':raise SystemExit(main())
