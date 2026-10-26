"""Light SYN40 clock/known26/stop supplement; never starts EDA.

complete(ctx,out) consumes the main reviewer's already parsed dictionaries:
before,after,om,nm,op,np,directions,oc,nc,old,new,prepared.  oc/nc map a net
to (physical input root,inversion parity).  No VG is read here.  out must be
a new directory.  MAX deadline logic has a separate proof and is not required
to have the SYN23 FF names or primitive count.

The checks follow synthesis_connectivity, revalidate_lidar_cdc.revalidate,
review_model25_observation_ring_syn39 clock tracing, and the selected IR role
inspection in review_syn39_ir_oled_light; missing or changed bindings are
retained explicitly, never accepted by removing generated suffixes.
"""
import argparse
from collections import Counter,defaultdict
from functools import lru_cache
import json
from pathlib import Path
import re
import traceback

BASE=Path('F:/nav_integration_g2_20261023')
OLD=BASE/'baseline_fusion_only_syn23'
PREPARED=BASE/'baseline_fusion50_ring_MAX_prepared1'
PNR35=BASE/'baseline_fusion32_CDC52_pnr35/cdc_reference_audit.json'
PROVEN_MAX=BASE/'baseline_fusion_model25_bounded_MAX_syn37/command_lease_guard.v'
IR='u_control/remote_thread'
LEASE='u_command_lease'
ROOTS={'clk50','rgmii_rxc'}
CLOCK_PIN=re.compile(r'(?:CLK\d*|CLKA|CLKB|WCLK|RCLK|CLOCK|CLKIN\d*)')
FORBIDDEN=re.compile(r'(?:^|/)(?:u_model_pll|model_clock_guard|bmi_bridge|config_bridge|action_bridge|fusion_bridge|status_bridge)(?:/|$)')

def load(p):return json.loads(Path(p).read_text(encoding='utf-8-sig'))
def save(p,v):
    assert not p.exists(),('Evidence already exists',str(p))
    p.write_text(json.dumps(v,ensure_ascii=False,indent=2)+'\n',encoding='utf-8')
def meta(p):
    p=Path(p);s=p.stat();return dict(path=str(p),bytes=s.st_size,mtime_ns=s.st_mtime_ns)

def source_contract(old,candidate):
    fixed=['g2_top.v','control4500_core.v','control/ir_remote_control.v',
           'control/motor520_closed_loop.v','control/motor520_hybrid_speed.v',
           'control/policy_action_map.v']
    watched=[root/name for root in (old,candidate) for name in fixed+['control/mem_policy.v']]
    watched += [candidate/'command_lease_guard.v',PROVEN_MAX]
    before=[meta(p) for p in watched]
    equality={name:(old/name).read_bytes()==(candidate/name).read_bytes() for name in fixed}
    def stop_block(root):
        text=(root/'control/mem_policy.v').read_text(encoding='utf-8-sig')
        start=text.index('    reg [31:0] sensor_age;')
        end=text.index('    wire capture,',start)
        return text[start:end]
    blocks={label:stop_block(root) for label,root in (('SYN23',old),('candidate',candidate))}
    max_equal=(candidate/'command_lease_guard.v').read_bytes()==PROVEN_MAX.read_bytes()
    unchanged=before==[meta(p) for p in watched]
    return dict(passed=all(equality.values()) and blocks['SYN23']==blocks['candidate'] and max_equal and unchanged,
      source_files_identical_to_SYN23=equality,mem_policy_sensor_age_stop_latch_ready_run_active_flags_identical=blocks['SYN23']==blocks['candidate'],
      mem_policy_stop_blocks=blocks,command_lease_source_identical_to_separately_proven_MAX=max_equal,
      MAX_reference=str(PROVEN_MAX),MAX_proof_not_replaced_by_this_review=True,
      metadata_before=before,source_metadata_unchanged=unchanged,
      scope='Source equality for actual stop/rearm/enable/feedback consumers; the three ring storage changes are outside this supplement.')

def output_finder(cells,directions):
    @lru_cache(None)
    def outputs(net):
        found=[]
        for name,c in cells.items():
            for pin,nets in c['ports'].items():
                if net in nets and directions[c['kind']][pin]=='output':
                    found += [(name,pin,bit) for bit,value in enumerate(nets) if value==net]
        return tuple(found)
    return outputs

def trace_clocks(cells,directions,provided_clock,outputs):
    traced={}
    def trace(net,trail=()):
        if net in traced:return traced[net]['root'],traced[net]['parity']
        assert net not in trail,('Clock cycle',net,trail)
        rows=outputs(net);assert len(rows)==1,('Clock driver not unique',net,rows)
        name,pin,bit=rows[0];cell=cells[name];kind=cell['kind']
        assert pin=='O' and bit==0,('Unexpected clock output pin',net,rows)
        if kind=='IBUF':
            assert cell['ports']['I'] in (['clk50'],['rgmii_rxc']),('Unexpected physical input clock',cell)
            root,parity=cell['ports']['I'][0],0
        else:
            assert kind in ('INV','BUF','BUFG'),('PLL/gating/unproved clock driver',net,cell)
            assert len(cell['ports']['I'])==1
            root,parity=trace(cell['ports']['I'][0],trail+(net,));parity^=int(kind=='INV')
        traced[net]=dict(root=root,parity=parity,driver=dict(instance=name,pin=pin,bit=bit,kind=kind),input_nets=cell['ports']['I'])
        assert tuple(provided_clock(net))==(root,parity),('Main clock map disagrees with independent driver trace',net)
        return root,parity
    counts=Counter();net_counts=Counter();constant=0;checked=0
    for name,c in cells.items():
        assert c['kind'] in directions,('Unknown primitive direction',name,c['kind'])
        for pin,nets in c['ports'].items():
            assert pin in directions[c['kind']],('Unknown primitive pin direction',name,pin)
            if directions[c['kind']][pin]!='input' or not CLOCK_PIN.fullmatch(pin):continue
            for bit,net in enumerate(nets):
                if net in ('VCC','GND'):constant+=1;continue
                root=trace(net);counts[root]+=1;net_counts[net]+=1;checked+=1
    assert {r for r,p in counts}==ROOTS,('Not exactly the original two roots',counts)
    return dict(passed=True,all_nonconstant_clock_pins_individually_traced=True,checked_clock_pins=checked,
      constant_clock_pins=constant,root_pin_counts=[dict(root=r,inverted=bool(p),pins=n) for (r,p),n in sorted(counts.items())],
      actual_clock_nets={n:dict(**traced[n],clock_input_pins=net_counts[n]) for n in sorted(net_counts)},
      supporting_driver_nets=traced,physical_clock_route_and_timing_proven=False)

def two_clock_review(ctx,old_outputs,new_outputs):
    old=trace_clocks(ctx['before'],ctx['directions'],ctx['oc'],old_outputs)
    new=trace_clocks(ctx['after'],ctx['directions'],ctx['nc'],new_outputs)
    common=0
    for name,b in ctx['after'].items():
        a=ctx['before'].get(name)
        if not a:continue
        for pin,nets in b['ports'].items():
            if not CLOCK_PIN.fullmatch(pin) or pin not in a['ports']:continue
            for bit,net in enumerate(nets):
                if bit>=len(a['ports'][pin]):continue
                other=a['ports'][pin][bit]
                if net in ('VCC','GND') or other in ('VCC','GND'):
                    assert net==other,('Constant/live clock changed',name,pin,bit)
                else:
                    assert tuple(ctx['oc'](other))==tuple(ctx['nc'](net)),('Common clock pin root/phase changed',name,pin,bit)
                    common+=1
    return dict(passed=True,SYN23=old,SYN40=new,common_actual_instance_clock_pins_checked=common,
      all_common_clock_pin_roots_and_phase_equal=True,unmatched_FF_names_not_assumed_equivalent=True)

def original26(ctx,old_outputs,new_outputs):
    from revalidate_lidar_cdc import module_at
    reference=load(PNR35);assert reference['passed'] and len(reference['paths'])==26
    before,after=ctx['before'],ctx['after'];op,np=ctx['op'],ctx['np'];directions=ctx['directions']
    rows=[];aliases={};first_nets=set()
    for row in reference['paths']:
        roles={}
        for role in ('source_register','first_register','second_register'):
            name=row[role];a,b=before[name],after[name]
            assert a['kind']==b['kind'] and a['kind'].startswith('DFF'),('Original26 kind changed',name)
            assert op(a)==np(b),('Original26 explicit parameters/INIT changed',name,op(a),np(b))
            frozen=row['actual_source_first_second'][role]
            assert a['kind']==frozen['primitive']['kind'] and op(a)==frozen['synthesis_explicit_parameters'],('SYN23 differs from original26 frozen role',name)
            different={p for p in a['ports'].keys()|b['ports'].keys() if a['ports'].get(p)!=b['ports'].get(p)}
            assert different<={'RESET','CLK'},('Original26 non-reset/data binding changed',name,different)
            assert [tuple(ctx['oc'](n)) for n in a['ports']['CLK']]==[tuple(ctx['nc'](n)) for n in b['ports']['CLK']],('Original26 clock-root/phase changed',name)
            if 'RESET' in different:
                an,=a['ports']['RESET'];bn,=b['ports']['RESET'];pair=(an,bn)
                if pair not in aliases:
                    ad,bd=old_outputs(an),new_outputs(bn);assert len(ad)==len(bd)==1
                    assert ad[0][0]==bd[0][0] and ad[0][1:]==bd[0][1:]==('Q',0),('Reset source role changed',pair,ad,bd)
                    ac,bc=before[ad[0][0]],after[bd[0][0]]
                    assert ac['kind']==bc['kind'] and ac['kind'].startswith('DFF') and op(ac)==np(bc)
                    assert {p:v for p,v in ac['ports'].items() if p!='Q'}=={p:v for p,v in bc['ports'].items() if p!='Q'}
                    hierarchy=ac['name'].rsplit('/',1)[0] if '/' in ac['name'] else ''
                    assert hierarchy,'Changed top reset alias needs a separate proof'
                    assert module_at(ctx['om'],hierarchy)==module_at(ctx['nm'],hierarchy)
                    aliases[pair]=dict(before=an,after=bn,driver=ac['name'],driver_kind_parameters_inputs_and_containing_module_equal=True)
            roles[role]=dict(name=name,kind=b['kind'],ports=b['ports'],parameters=np(b))
        s,f,t=(after[row[k]] for k in ('source_register','first_register','second_register'))
        assert s['ports']['Q']==f['ports']['D'] and len(s['ports']['Q'])==1
        assert new_outputs(f['ports']['D'][0])==((s['name'],'Q',0),)
        assert f['ports']['Q']==t['ports']['D'] and len(f['ports']['Q'])==1
        assert new_outputs(t['ports']['D'][0])==((f['name'],'Q',0),)
        assert row['source_clock'] in ('rgmii_rxc_d','clk50_d') and row['target_clock'] in ('rgmii_rxc_d','clk50_d')
        source_root='rgmii_rxc' if row['source_clock']=='rgmii_rxc_d' else 'clk50'
        target_root='rgmii_rxc' if row['target_clock']=='rgmii_rxc_d' else 'clk50'
        assert tuple(ctx['nc'](s['ports']['CLK'][0]))==(source_root,0)
        for c in (f,t):
            assert tuple(ctx['nc'](c['ports']['CLK'][0]))==(target_root,0)
            assert c['ports'].get('CE',['VCC'])==['VCC']
        first_nets.add(f['ports']['Q'][0]);rows.append(dict(group=row['group'],roles=roles,first_Q=f['ports']['Q'][0],second=t['name'],reference_budget_ns=row['route_max_delay_ns']))
    loads=defaultdict(list)
    for name,c in after.items():
        for pin,nets in c['ports'].items():
            if directions[c['kind']][pin] in ('input','inout'):
                for bit,net in enumerate(nets):
                    if net in first_nets:loads[net].append((name,pin,bit))
    for row in rows:
        actual=loads[row['first_Q']];assert actual==[(row['second'],'D',0)],('Original26 first-stage extra load',row,actual)
        row['actual_first_Q_input_loads']=actual
    return dict(passed=True,paths=26,source_first_second_kind_explicit_parameters_INIT_clock_and_connections_preserved=True,
      first_stage_Q_unique_second_D_load_recounted_in_entire_new_design=True,rows=rows,proved_reset_aliases=list(aliases.values()),
      reference=str(PNR35),physical_delay_reset_release_and_MTBF_proven=False)

def stop_actual(ctx):
    from review_model25_joint_syn import module_bindings
    both=[]
    for cells,modules,parameters,clock in ((ctx['before'],ctx['om'],ctx['op'],ctx['oc']),(ctx['after'],ctx['nm'],ctx['np'],ctx['nc'])):
        bindings=module_bindings(modules,[IR,LEASE]);binding=bindings[IR]
        stop=binding['ports'].get('ir_emergency_stop',binding['ports'].get('emergency_stop'))
        assert stop and len(stop)==1,('Actual IR stop port unavailable',binding)
        driver=[c for c in cells.values() if c['ports'].get('Q')==stop]
        assert len(driver)==1 and driver[0]['kind'].startswith('DFF'),('Actual IR stop state not unique',stop,driver)
        c=driver[0];assert c['ports']['D']==['VCC'] and tuple(clock(c['ports']['CLK'][0]))==('clk50',0)
        both.append(dict(IR_binding=binding,lease_boundary=bindings[LEASE],IR_stop_latch=dict(name=c['name'],kind=c['kind'],ports=c['ports'],parameters=parameters(c))))
    a,b=(x['IR_stop_latch'] for x in both)
    # The state is selected by its actual stop output, not an automatic FF name.
    checks=dict(IR_stop_kind_parameters_equal=a['kind']==b['kind'] and a['parameters']==b['parameters'],
      IR_stop_data_reset_CE_binding_equal={p:v for p,v in a['ports'].items() if p not in ('Q','CLK')}=={p:v for p,v in b['ports'].items() if p not in ('Q','CLK')},
      IR_entire_actual_boundary_equal=both[0]['IR_binding']['ports']==both[1]['IR_binding']['ports'])
    return dict(passed=all(checks.values()),checks=checks,reference=both[0],candidate=both[1],
      lease_boundary_literal_equal=both[0]['lease_boundary']['ports']==both[1]['lease_boundary']['ports'],
      MAX_lease_primitive_or_FF_name_equality_required=False,full_lease_or_whole_control_equivalence_claimed=False,
      scope='IR stop FF role, kind/INIT/data/reset/CE and actual module boundary; source identity supplies stop/rearm/enable/feedback semantics. Changed reset/CE/boundary spellings are not presumed aliases.')

def complete(ctx,out):
    out=Path(out);out.mkdir(parents=True,exist_ok=False)
    result=dict(passed=False,scope='SYN40 only: actual two input clock roots, original26 and source/selected IR stop preservation; no resources/RAM/CPU/TR/physical acceptance',
      new_EDA_started=False,VG_parsed_by_helper=False,physical_timing_status='unknown',deployment_accepted=False,
      old=str(ctx['old']),new=str(ctx['new']),prepared=str(ctx['prepared']),sections={})
    old_outputs=output_finder(ctx['before'],ctx['directions']);new_outputs=output_finder(ctx['after'],ctx['directions'])
    jobs=[('source_stop_contract',lambda:source_contract(Path(ctx['old']),Path(ctx['new']))),
      ('actual_two_clocks',lambda:two_clock_review(ctx,old_outputs,new_outputs)),
      ('actual_original26',lambda:original26(ctx,old_outputs,new_outputs)),('actual_IR_stop',lambda:stop_actual(ctx))]
    for name,operation in jobs:
        try:value=operation()
        except Exception as error:value=dict(passed=False,status='failed_or_unverified',error=str(error),traceback=traceback.format_exc())
        save(out/(name+'.json'),value);result['sections'][name]=dict(passed=value.get('passed',False),details=name+'.json')
    forbidden=[dict(name=n,kind=c['kind']) for n,c in ctx['after'].items() if 'PLL' in c['kind'].upper() or FORBIDDEN.search(n)]
    result['absent_PLL_and_five_related_bridges']=not forbidden;result['forbidden_actual_leaves']=forbidden
    result['passed']=not forbidden and all(v['passed'] for v in result['sections'].values())
    result['synthesis_frequency_estimates_are_physical_timing']=False
    save(out/'result.json',result);return result

if __name__=='__main__':
    parser=argparse.ArgumentParser();parser.add_argument('--prepare-only',action='store_true',required=True)
    parser.add_argument('--source',type=Path,default=PREPARED);parser.add_argument('--out',type=Path,required=True)
    args=parser.parse_args();args.out.mkdir(parents=True,exist_ok=False)
    source=source_contract(OLD,args.source)
    save(args.out/'source_stop_contract.json',source)
    save(args.out/'result.json',dict(passed=False,phase='helper_prepared_actual_VG_pending',source_checks_passed=source['passed'],
      module='review_model50_syn40_clock_stop',function='complete(ctx,out)',VG_read=False,new_EDA_started=False,
      required_ctx=['before','after','om','nm','op','np','directions','oc','nc','old','new','prepared'],
      pending=['Each actual clock input pin and IBUF/BUF/INV driver trace','No actual PLL/five related bridge leaves','Original26 kind/parameters/INIT/data/clock and whole-design first-stage load count','Actual IR stop role and reset/CE/boundary'],
      physical_timing_status='unknown',deployment_accepted=False))
    print(json.dumps(dict(phase='helper_prepared_actual_VG_pending',source_checks_passed=source['passed'],out=str(args.out))))
