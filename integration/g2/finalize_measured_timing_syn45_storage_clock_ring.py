"""Combine preserved SYN45 actual review sections; never launches EDA.

Only independently passed sections are accepted. The original failed binding
attempts remain as evidence, and the ring/IR supplements replace only their
failed sections. Physical build acceptance is deliberately not generated.
"""
from pathlib import Path
from datetime import datetime
from collections import Counter
import argparse,json,re

BASE=Path('F:/nav_integration_g2_20261023')
FIRST=BASE/'SYN45_storage_clock_ring_review1'
IR=BASE/'SYN45_ring_IR_binding_review2/actual_IR_stop_partition'
NEW=BASE/'SLF128_measured_timing_syn45'
OLD=BASE/'SLF128_ring_stride32_syn44'
DEFAULT_ALIAS=BASE/'SYN45_action_last_high_alias_supplement1/result.json'


def load(p):return json.loads(p.read_text(encoding='utf-8-sig'))
def save(p,v):p.write_text(json.dumps(v,ensure_ascii=False,indent=2)+'\n',encoding='utf8')
def meta(p):s=p.stat();return dict(path=str(p),bytes=s.st_size,mtime_ns=s.st_mtime_ns)
def lit(v):
    m=re.fullmatch(r"(?:\d+)'([bhd])([0-9a-fA-F]+)",str(v))
    return int(m[2],dict(b=2,h=16,d=10)[m[1]])if m else int(v)


def roles26(out):
    p=FIRST/'clock_stop/actual_original26_and_CDC52.json'
    old=BASE/'SYN44_clock_CDC_stop_review1/actual_original26_and_CDC52.json'
    a,b=load(old),load(p);assert a['passed']and b['passed']
    previous={(x['group'],x.get('bit')):x for x in a['rows']}
    rows=b['rows'];assert len(rows)==26
    paths=[];parameters=[]
    for x in rows:
        before=previous[(x['group'],x.get('bit'))]
        for role in ('source','first','second'):
            c,d=before[role],x[role];assert c['kind']==d['kind']
            pa,pb=before[role+'_reset']['parameters'],x[role+'_reset']['parameters']
            assert set(pa)<= {'INIT'}and set(pb)<= {'INIT'},(pa,pb)
            default=1 if d['kind']in ('DFFPE','DFFSE')else 0
            ia,ib=lit(pa.get('INIT',default)),lit(pb.get('INIT',default));assert ia==ib
            parameters.append(dict(group=x['group'],bit=x.get('bit'),role=role,
                before_instance=c['name'],after_instance=d['name'],kind=d['kind'],
                before_parameters=pa,after_parameters=pb,effective_INIT=ib))
        paths.append(dict(source_pin=x['source_pin'],first_pin=x['first_pin'],budget_ns=x['budget_ns'],
            group=x['group'],bit=x.get('bit'),source_clock=x['source_clock'],target_clock=x['target_clock'],
            source_register=x['source']['name'],first_register=x['first']['name'],
            second_register=x['second']['name'],route_max_delay_ns=x['budget_ns']))
    counts=dict(Counter(x['group']for x in paths))
    assert counts==dict(rx_write_gray=12,core_read_gray=12,rx_clock_monitor=1,rx_sampling_select=1)
    assert len({(x['source_pin'],x['first_pin'])for x in paths})==26 and len(parameters)==78
    result=dict(passed=True,actual_netlist=meta(NEW/'impl/gwsynthesis/g2_top.vg'),paths=paths,
        group_counts=counts,all78_role_kind_INIT_parameters_equal=True,role_parameters=parameters,
        actual_connectivity_and_parameter_evidence=meta(p),previous_role_reference=meta(old),
        physical_delays_Actived_and_MTBF_proven=False)
    save(out/'actual_roles26.json',result)
    return result


def run(out,ring_dir,alias):
    assert not out.exists();out.mkdir(parents=True)
    source=load(FIRST/'ring_memory/source.json');memory=load(FIRST/'ring_memory/all_memories_CPU.json')
    ring=load(ring_dir/'result.json');ir=load(IR/'result.json');proof=load(alias)
    assert source['passed']and memory['passed']and ring['passed']and ir['passed']and proof['passed']
    assert ring['checks']>=1038 and ring['actual_frame_RAM16SDP4']==78
    assert ir['actual_IR_FFs']==154 and ir['total_checks']==182
    cp=FIRST/'clock_stop'
    clock=load(cp/'actual_two_clocks.json');fifo=load(cp/'actual_FIFO_state_reset_classes.json');lease=load(cp/'actual_lease_stop_partition.json')
    assert clock['passed']and fifo['passed']and lease['passed']
    assert lease['actual_FFs']==200 and memory['BSRAM']==269
    original=roles26(out)
    paths=dict(source=str(FIRST/'ring_memory/source.json'),all_memories_CPU=str(FIRST/'ring_memory/all_memories_CPU.json'),
        all_actual_memories=str(FIRST/'ring_memory/all_actual_memories.json'),actual_ring=str(ring_dir/'result.json'),
        actual_ring_checks=str(ring_dir/'actual_ring_checks.json'),actual_IR=str(IR/'result.json'),
        actual_IR_bindings=str(IR/'IR_actual_bindings_checks.json'),alias_proof=str(alias),
        actual_two_clocks=str(cp/'actual_two_clocks.json'),actual_FIFO=str(cp/'actual_FIFO_state_reset_classes.json'),
        actual_lease=str(cp/'actual_lease_stop_partition.json'),actual_roles26=str(out/'actual_roles26.json'))
    result=dict(passed=True,completed_at=datetime.now().astimezone().isoformat(),old=str(OLD),new=str(NEW),
        storage_CPU_passed=True,ring_passed=True,two_clocks_passed=True,original26_passed=True,
        FIFO_reset_passed=True,IR_passed=True,lease_passed=True,actual_roles26=str(out/'actual_roles26.json'),
        source=dict(RTL=63,changed_RTL=source['changed_RTL'],all104_assets_exact=True,FW_BIN_bytes=53572),
        storage=dict(nonring_leaves=memory['nonring_leaves'],every_nonring_name_kind_effective_parameter_INIT_equal=True,
            BSRAM=269,counts=memory['all_actual_memory_counts_after'],CPU=memory['CPU']),
        ring=dict(actual_frame_RAM16SDP4=78,all_are_frame_mem=True,allocated_words=208,allocated_bits=4992,
            before_frame_RAM16SDP4=78,before_allocated_bits=4992,allocated_delta_bits=0,
            cursor_FFs=11,address_MULT12X12=0,all13banks_read_write_and_24bit_prefetch_checked=True,
            checks=ring['checks'],core_input_RAM_write_port_miters=396,source_producer_unchanged=True,
            extra18_numerator_high_FF_sign_aliases='Actual two-level INIT/base+nextstate induction certificate, then rechecked DI miter',
            whole_actual_core_equivalence_proved=False),
        clocks=dict(nonconstant_clockpins=clock['checked_clock_pins'],constant_clockpins=clock['constant_clock_pins'],
            roots=clock['root_pin_counts'],every_clockpin_individually_traced=True,original26_group_counts=original['group_counts'],
            all78_role_kind_INIT_parameters_preserved=True,FIFO_FFs=fifo['actual_FIFO_FFs']),
        stop=dict(IR_FFs=154,IR_checks=182,lease_FFs=200,lease_checks=len(lease['all_actual_next_states_and_outputs_equal'])),
        evidence=paths,proof_bindings=[meta(alias),meta(ring_dir/'result.json'),meta(IR/'result.json')],
        detailed_metadata_clarification='The first reusable ring report retained SYN43->44 legacy old_allocated_bits=4224/delta768. For this SYN44->45 comparison both actual frame inventories are78 leaves/4992bits, so delta0. The corrected reviewer derives these fields from actual leaf counts; this metadata clarification does not rerun or alter proofs.',
        preserved_initial_failures=[str(FIRST/'result.json'),str(BASE/'SYN45_ring_IR_binding_review2/result.json')],
        resolution='First attempt used instance-name-only state pairing and exact auto port sets. New evidence proves semantic age FF transitions, renamed IR input and every live IR transition/output, plus actual high/sign state invariants before comparing ring RAM DI.',
        physical_exploration_audited_gates_pass=True,automatic_PNR=False,deployment_accepted=False,no_EDA_started=True,
        not_covered=['Action product/DSP mapping and NAV/PID mappings have separate root-assigned reviewers',
            'Complete mapped core equivalence and universal nonring data/control-port equivalence',
            'Routed timing, physical BSRAM splitting, actual52 Actived status/maxdelay/MTBF',
            'Physical reset recovery/removal/min pulsewidth, CPU busy-stop and vehicle stopping'])
    result['reproducible_entrypoints']=dict(
        initial_review='D:/Desktop/SOC FPGA/nav/integration/g2/review_measured_timing_syn45_storage_clock_ring.py',
        ring_IR_binding_supplement='D:/Desktop/SOC FPGA/nav/integration/g2/review_measured_timing_syn45_ring_IR_supplement.py',
        finalizer=str(Path(__file__).resolve()),
        ring_command='D:/Anaconda/python.exe -B -X utf8 D:/Desktop/SOC FPGA/nav/integration/g2/review_measured_timing_syn45_ring_IR_supplement.py --out <new-empty-out> --parts actual_ring --certified-aliases '+str(alias),
        IR_command='D:/Anaconda/python.exe -B -X utf8 D:/Desktop/SOC FPGA/nav/integration/g2/review_measured_timing_syn45_ring_IR_supplement.py --out <different-new-empty-out> --parts actual_IR_stop_partition')
    save(out/'result.json',result);print(json.dumps(dict(passed=True,result=str(out/'result.json'),actual_roles26=str(out/'actual_roles26.json')),ensure_ascii=False))
    return 0


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--out',type=Path,required=True);p.add_argument('--ring',type=Path,required=True)
    p.add_argument('--alias-proof',type=Path,default=DEFAULT_ALIAS);a=p.parse_args()
    raise SystemExit(run(a.out,a.ring,a.alias_proof))
