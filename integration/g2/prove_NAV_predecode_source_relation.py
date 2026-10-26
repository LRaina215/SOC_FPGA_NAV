"""Source-bound one-step proof for the frozen six-command predecode.

No Verilog frontend/EDA or new simulation. Exact-source reconstruction binds
the handwritten transition relation to the actual candidate. Z3 proves the
binary bus step; an abstract comparison-result four-state proof avoids making
up vector== semantics and covers X/Z payloads and nonaccept cycles.
"""
import argparse
from datetime import datetime
import itertools
import json
from pathlib import Path
import re

from prepare_baseline_fusion_retime import metadata,same_bytes
from prepare_nav_command_predecode import derive,PULSES,ACCEPT,DEFAULTS,REPLACEMENTS
from prove_soc_console_remap import z3

BASE=Path('F:/nav_integration_g2_20261023')
SOURCE=BASE/'SLF128_storage_safeFW_syn43'
CANDIDATE=BASE/'SLF128_NAV_predecode_prepared1'
PROBE=BASE/'NAV_command_predecode_bus_probe2'
NAV='nav/rtl/top/nav_subsystem.v'
WINDOWS=('COSTMAP','TRAJ_RES','TRAJ_CAND','MATCH_RES','MATCH_CAND','LRAW_TX','LRAW_RX')
COMMANDS=(('MATCH_CMD',0),('MATCH_CMD',1),('MATCH_CMD',2),('LIDAR_POSE_COMMIT',None),('MAP_CMD',0),('MAP_CMD',1))


def load(p):return json.loads(p.read_text(encoding='utf8'))


def check(label,mismatch,assumptions=()):
    s=z3.Solver();s.set(timeout=30000);s.add(*assumptions);s.add(mismatch)
    answer=s.check()
    assert answer==z3.unsat,(label,str(answer),str(s.model())if answer==z3.sat else s.reason_unknown())
    return dict(label=label,result='UNSAT',assumptions=[str(a)for a in assumptions])


def scalar_truth_tables():
    # Codes0/1 are known bits;2=X;3=Z. Logical/bitwise scalar AND normalize
    # either unknown value to X unless a known0 or both-known1 decides it.
    def and_(a,b):return 0 if a==0 or b==0 else 1 if a==b==1 else 2
    rows=[]
    for accepted,eq,payload,old_eq,old_payload in itertools.product((False,True),(0,1,2),range(4),(0,1,2),range(4)):
        wr=1 if accepted else 0
        eq_after=eq if accepted else old_eq
        payload_after=payload if accepted else old_payload
        original=and_(and_(wr,eq_after),payload_after)
        proposed=and_(eq,payload)if accepted else 0
        assert original==proposed
        original_pose=and_(wr,eq_after)
        proposed_pose=eq if accepted else 0
        assert original_pose==proposed_pose
        rows.append((accepted,eq,payload,old_eq,old_payload,original,proposed))
    assert len(rows)==288
    return dict(exhaustive_scalar_case_count=len(rows),all_equal=True,
        equality_result_domain='0/1/X only; the exact same Verilog vector== expression and accepted aligned operands appear before/after capture',
        payload_and_old_payload_domain='0/1/X/Z',
        bad_procedural_if_comparison_example=dict(accepted=True,comparison='X',payload='1',
            original_and_actual_candidate='X',incorrect_if_eq_then_set_else_clear='0'))


def binary_bus(header):
    addresses={}
    for name in tuple('RA_'+n for n,_ in COMMANDS)+('RA_MBX_RX_DATA',):
        m=re.search(r'localparam\s+\[15:0\]\s+'+name+r"\s*=\s*16'h([0-9A-Fa-f]+);",header)
        assert m,name;addresses[name]=int(m[1],16)
    ranges=[]
    for name in WINDOWS:
        values=[]
        for suffix in('BASE','SIZE'):
            key='RW_'+name+'_'+suffix
            m=re.search(r'localparam\s+\[15:0\]\s+'+key+r"\s*=\s*16'h([0-9A-Fa-f]+);",header)
            assert m,key;values.append(int(m[1],16))
        base,size=values;assert base+size<65536
        ranges.append(dict(name=name,base=base,size=size))
    addr=z3.BitVec('bus_addr',16);data=z3.BitVec('bus_wdata',32)
    old_a=z3.BitVec('arbitrary_old_a_q',16);old_d=z3.BitVec('arbitrary_old_wd_q',32)
    state=z3.BitVec('arbitrary_old_bs',4)
    rst,req,we=z3.Bools('rst bus_req bus_we')
    aligned=addr&z3.BitVecVal(0xfffc,16)
    capture=z3.And(z3.Not(rst),state==0,req)
    window_tests=[z3.And(z3.UGE(addr,r['base']),z3.ULT(addr,r['base']+r['size']))for r in ranges]
    mbx_read=z3.And(z3.Not(we),aligned==addresses['RA_MBX_RX_DATA'])
    # Exact final branch is reached only after each preceding if/else-if falls
    # through. This covers arbitrary pending window/stall states as state!=0.
    fallback=z3.And(capture,*[z3.Not(w)for w in window_tests],z3.Not(mbx_read))
    original_wr_after=z3.And(fallback,we)
    a_after=z3.If(capture,aligned,old_a);d_after=z3.If(capture,data,old_d)
    queries=[];new=[]
    for name,(command,bit)in zip(PULSES,COMMANDS):
        code=addresses['RA_'+command]
        old=z3.And(original_wr_after,a_after==code)
        d=z3.BoolVal(True)if bit is None else z3.Extract(bit,bit,data)==1
        if bit is not None:old=z3.And(old,z3.Extract(bit,bit,d_after)==1)
        proposed=z3.And(fallback,we,aligned==code,d)
        new.append(proposed)
        queries.append(check(name+': one-step original pulse expression == registered proposed pulse',old!=proposed))
    any_new=z3.Or(*new)
    queries += [check('any special window suppresses every six-command pulse',z3.And(z3.Or(*window_tests),any_new)),
                check('read/reset/nonIDLE/no-request suppress every pulse',z3.And(z3.Or(z3.Not(we),rst,state!=0,z3.Not(req)),any_new))]
    return dict(all_16bit_addresses_all32bit_data_and_all_old_bus_registers=True,
        all4bit_BS_states=True,queries=queries,window_priority=[r['name']for r in ranges]+['MBX_RX_DATA read','ordinary RF'],
        register_addresses=addresses,actual_window_ranges=ranges,
        no_memory_latency_or_arbitration_stability_assumption_needed_for_local_pulse_identity=True)


def four_state_step():
    # The branch itself is identical text outside the pulse assignments. Its
    # result is a known action-taken Boolean even if Verilog if/case inputs
    # contain X/Z. Source equality establishes this shared action selection.
    accept=z3.Bool('same_actual_RF_write_branch_taken')
    eq=z3.Int('comparison_of_current_aligned_address')
    oldeq=z3.Int('arbitrary_comparison_if_no_write')
    payload=z3.Int('current_payload_bit');oldpayload=z3.Int('arbitrary_payload_if_no_write')
    constraints=(eq>=0,eq<=2,oldeq>=0,oldeq<=2,payload>=0,payload<=3,oldpayload>=0,oldpayload<=3)
    def and_(a,b):return z3.If(z3.Or(a==0,b==0),0,z3.If(z3.And(a==1,b==1),1,2))
    wr=z3.If(accept,1,0);posteq=z3.If(accept,eq,oldeq);postpayload=z3.If(accept,payload,oldpayload)
    olddata=and_(and_(wr,posteq),postpayload)
    newdata=z3.If(accept,and_(eq,payload),0)
    oldpose=and_(wr,posteq);newpose=z3.If(accept,eq,0)
    return dict(queries=[check('four-state generic data-gated pulse, including all no-write oldX/Z cases',olddata!=newdata,constraints),
                         check('four-state POSE pulse, including all no-write oldX/Z cases',oldpose!=newpose,constraints)],
        table=scalar_truth_tables(),
        unknown_request_reset_state_and_window_scope='The unchanged procedural if/case spine selects the same branch, including normal Verilog only-known1-if and exact-case behavior; acceptance is then0/1, not a 4-state combinational authorization gate.',
        initial_state_scope='Pulse relation holds after the first shared positive edge from arbitrary old bus/pulse values. Arbitrary pre-first-edge combinational outputs are not claimed equivalent; mapped cold-start INIT/reset remains an actual-netlist gate.')


def main():
    ap=argparse.ArgumentParser(description=__doc__);ap.add_argument('--out',type=Path,required=True)
    a=ap.parse_args();assert not a.out.exists()
    original=(SOURCE/NAV).read_text(encoding='utf8');candidate=(CANDIDATE/NAV).read_text(encoding='utf8')
    expected,consumers=derive(original);assert candidate==expected
    manifest=load(SOURCE/'manifest.json');rels=[Path(p).relative_to(SOURCE).as_posix()for p in manifest['source_files']]
    assert len(rels)==63 and all(same_bytes(SOURCE/r,CANDIDATE/r)for r in rels if r!=NAV)
    sourcefiles=[SOURCE/r for r in rels]+[CANDIDATE/r for r in rels]
    regmap=SOURCE/'nav/rtl/regs/nav_regs_map.vh';rf=SOURCE/'nav/rtl/regs/nav_regfile.v';rfh=SOURCE/'nav/rtl/regs/nav_subsystem_rf.vh'
    for p in(regmap,rf,rfh):assert same_bytes(p,CANDIDATE/p.relative_to(SOURCE))
    include=rfh.read_text(encoding='utf8');regs=rf.read_text(encoding='utf8');header=regmap.read_text(encoding='utf8')
    assert '.wr(rf_wr), .rd(rf_rd), .addr(a_q), .wdata(wd_q)'in include
    for n,_ in COMMANDS:
        key='RA_'+n;m=re.search(r'localparam\s+\[15:0\]\s+'+key+r"\s*=\s*(16'h[0-9A-Fa-f]+);",header);assert m
        assert f'assign p_{n} = wr && (addr == {m[1]});'in regs
    # These source facts connect the mathematical step to the actual accepted
    # live bus operands and exclude hidden writers/extra pulse stages.
    assert re.findall(r"\brf_wr\s*<=\s*1'b([01])",original)==['0','1']
    assert len(re.findall(r'\ba_q\s*<=',original))==len(re.findall(r'\bwd_q\s*<=',original))==1
    assert 'a_q <= {bus_addr[15:2], 2\'b00}; wd_q <= bus_wdata;'in original
    for name in PULSES:assert len(re.findall(r'\b'+name+r'\s*<=',candidate))==2
    idle=original[original.index('            B_IDLE:'):original.index('            B_RF:')]
    order=['if (in_cm)','else if (in_res)','else if (in_cand)','else if (in_mres)','else if (in_mcand)','else if (in_lrtx)','else if (in_lrrx)',
           "else if (!bus_we && {bus_addr[15:2], 2'b00} == RA_MBX_RX_DATA)","if (bus_we) rf_wr <= 1'b1; else rf_rd <= 1'b1;"]
    locations=[idle.index(x)for x in order];assert locations==sorted(locations)
    assert original.count('B_END: bs <= B_IDLE;')==candidate.count('B_END: bs <= B_IDLE;')==1
    probe=load(PROBE/'result.json');assert probe['passed']and probe['inputs_unchanged']
    assert Path(probe['source']).resolve()==CANDIDATE.resolve()
    for b in probe['bindings']:assert metadata(Path(b['path']))==b,('finite probe input changed',b['path'])
    for label,text in(('reference',original),('candidate',candidate)):
        fixture=(PROBE/('nav_'+label+'.v')).read_text(encoding='utf8')
        normalized=fixture.replace('module nav_command_'+label+' #(','module nav_subsystem #(',1)
        assert normalized.count('`undef NAV_REGS_MAP_VH\n')==1
        normalized=normalized.replace('`undef NAV_REGS_MAP_VH\n','',1)
        assert normalized==text,('finite miter changed hardware logic',label)
    log=(PROBE/'xsim.log').read_text(encoding='utf8');tb=(PROBE/'tb_NAV_command_predecode.sv').read_text(encoding='utf8')
    match=re.search(r'TEST PASSED NAV_COMMAND_BUS checks=(\d+) transactions=(\d+) resets=(\d+) arbitration=(\d+) xz=(\d+) unknown_read_words=(\d+)',log);assert match
    assert 'Fatal:'not in log and 'for(integer b=0;b<6;b++)if(pulses[b]==0)$fatal'in tb
    assert not re.search(r'\bforce\b',tb)
    bindings=[metadata(p)for p in sourcefiles+[regmap,rf,rfh,PROBE/'result.json',PROBE/'xsim.log',PROBE/'tb_NAV_command_predecode.sv',PROBE/'bus_wrappers.sv',Path(__file__).resolve()]]
    binary=binary_bus(header);four=four_state_step()
    assert bindings==[metadata(Path(b['path']))for b in bindings]
    result=dict(passed=True,recorded_at=datetime.now().astimezone().isoformat(),source=str(SOURCE),candidate=str(CANDIDATE),
        method='Exact frozen source reconstruction plus all-input one-step transition relation and 4-state scalar equality; not an RTL-frontend whole-G2 formal run',
        source_unchanged_except_one_NAV_patch=True,all62_other_RTL_byte_identical=True,consumer_replacements=consumers,
        source_spine_checks=dict(original_only2_rf_wr_assignments=True,single_same_edge_a_q_wd_q_capture=True,
            original_window_if_else_priority=order,old_ref_regfile_inputs='rf_wr/a_q/wd_q',new_pulse_assignment_sites_per_bit=2,
            original_B_RF_ACK_and_B_END_bubble_unchanged=True,all_RAW_ownership_frame_and_stop_code_unchanged=True),
        binary_transition_proof=binary,four_state_relation_proof=four,solver='Z3 '+z3.get_version_string(),
        finite_existing_probe=dict(source=metadata(PROBE/'result.json'),clock_comparisons=int(match[1]),transactions=int(match[2]),
            resets=int(match[3]),arbitration_overlap_samples=int(match[4]),XZ_cases=int(match[5]),unknown_read_words=int(match[6]),
            all6_command_bits_exercised_by_actual_TB=True,renamed_old_new_sources_only_and_local_include_guard_reset_verified=True,
            actual_RX_idle=True,new_simulation_started=False),
        compositional_trace_argument=[
            'After each shared edge the6 registers equal the untouched rf_wr/latched-address/data expressions, for accepting and nonaccepting actions. No prior extra-register relation is required.',
            'Between shared edges both expressions depend only on their registered state. Unselected/master/live payload changes cannot alter already accepted commands.',
            'Every old shared state and every child module/consumer is unchanged except substituting these equal wires. From an equal initialized shared-state/memory boundary AND aligned six-pulse relation, identical external inputs give equal subsequent next states/outputs by induction. This is not a claim from arbitrary inconsistent added-register initial values.',
            'E0 acceptance does not execute consumers early; E1 consumers and ACK see old pulse/address/data; E2 B_END and arbiter hold are original. A held req may be reaccepted later exactly as before, not a new exactly-once promise.',
            'This substitution argument is independent of arbitrary RAW/FIFO/MBX ready/results at the unchanged boundaries. It preserves existing operational assumptions; it does not newly certify their original safety.'],
        sufficient_source_functional_evidence_for_next_joint_SYN=True,
        reason='Six local same-edge pulse substitutions are source-bound equal independently of idle-network stimulus; existing finite actual NAV+arb miter supports integration. Mapping/physical gates remain pending.',
        functional_trace_base_condition='Original known-startup/reset contract must establish the common initialized boundary and pulse relation before transactions. Probe2 holds common reset for5 clocks. New actual FF INIT/clear must be checked against original rf_wr0; local one-step pulse identity alone does not establish every old consumer state after an inconsistent unreset start.',
        root_new_combined_source_gate_required=True,
        remaining_before_physical_or_deployment_acceptance=[
            'New actual mapped6 FF/copies/INIT/clock/clear and D/CE cones equal the accepted pulse expressions; no residual old address decode in the targeted consuming CE cones',
            'Actual mapped no additional bus/point/command clocks and original all RAM/INIT/CPU/26CDC/IR/SAFE/lease preservation',
            'Existing finite miter holds RX idle: no direct exercised RAW/SOF+command/last point drain, variable MBX_RX wait or continuous console/injection traffic in that TB',
            'Whole-G2 frontend formal/physical reset/IO/CDC/stop not proved by this local handwritten transition partition',
            'Physical setup/hold for new bus→pulse FF and unchanged reverse-ready suffix; prefix removal alone cannot promise50MHz'],
        no_new_RAW_FIFO_MBX_TB_needed_before_exploratory_SYN=True,
        no_RTL_file_written=True,existing_preparer_or_runner_modified=False,EDA_started=False,simulator_started=False,
        deployment_accepted=False,input_bindings=bindings,no_SHA=True)
    a.out.mkdir(parents=True)
    (a.out/'result.json').write_text(json.dumps(result,ensure_ascii=False,indent=2)+'\n',encoding='utf8')
    print(json.dumps(dict(passed=True,sufficient_for_next_joint_SYN=True,UNSAT_queries=len(binary['queries'])+len(four['queries']),
        four_state_scalar_cases=four['table']['exhaustive_scalar_case_count'],out=str(a.out/'result.json'),new_EDA_or_simulation=False),ensure_ascii=False))


if __name__=='__main__':main()
