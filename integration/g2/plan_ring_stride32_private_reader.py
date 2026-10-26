"""Create only a design plan for the measured PNR53 ring read-address path.

There is deliberately no RTL materialization, simulator, synthesis or PNR
mode. The current frozen source and both physical jobs remain untouched.
"""
import argparse
from datetime import datetime
import json
from pathlib import Path

from synthesis_connectivity import CELL


BASE=Path('F:/nav_integration_g2_20261023')


def meta(p):
    s=p.stat();return dict(path=str(p),bytes=s.st_size,mtime_ns=s.st_mtime_ns)


def main():
    ap=argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--source',type=Path,default=BASE/'SLF128_storage_safeFW_syn43')
    ap.add_argument('--actual-observation',type=Path,default=BASE/'SYN43_sensor_age_snapshot_path_readonly1/rl_observation_v5.v')
    ap.add_argument('--timing',type=Path,default=BASE/'PNR53_all_failed_endpoints_classification1/result.json')
    ap.add_argument('--out',type=Path,required=True)
    a=ap.parse_args();observer=a.source/'control/rl_observation_v5.v';policy=a.source/'control/rl_policy_tick_v5.v';owner=a.source/'control/mem_policy.v'
    ot=observer.read_text();pt=policy.read_text();actual=a.actual_observation.read_text()
    for anchor in ('reg signed [23:0] frame_mem [0:167];','work_slot*8\'d28+lane','frame_read_index-segment*8\'d28','read_slot*8\'d28+read_lane',"segment==0 ? clip_current(raw_word) : raw_word"):
        assert anchor in ot,anchor
    assert 'frame_read_index=state==SNAPSHOT ? 8\'d0 : load_index+8\'d1' in pt
    assert 'if(load_index!=167) snapshot_word<=frame_read_data;' in pt
    ram=[dict(kind=k,name=n.lstrip('\\')) for k,n,b in CELL.findall(actual) if 'g_ring.frame_mem' in n and k.startswith('RAM')]
    dsp=[n.lstrip('\\') for k,n,b in CELL.findall(actual) if k=='MULT12X12']
    assert len(ram)==66 and {r['kind'] for r in ram}=={'RAM16SDP4'}
    assert set(dsp)=={'n22065_s','n22100_s','n22165_s'}
    timing=json.loads(a.timing.read_text());worst=timing['worst_path']
    assert 'load_index' in worst['summary']['From'] and 'snapshot_word' in worst['summary']['To']
    rows=worst['actual_data_rows']
    travel=[r for r in rows if r['type']=='tNET' and ('n22100_s/A[0]' in r['node'] or 'read_address_0_s91/I1' in r['node'])]
    assert len(travel)==2
    physical_relation=[dict(logical_segment=s,logical_indices=[s*28,s*28+27],slot='live4 at first capture, then frozen4' if s==0 else 'frozen'+str(s-1),clip_current_only=s==0)
                       for s in range(6)]
    result=dict(recorded_at=datetime.now().astimezone().isoformat(),phase='design_plan_only',
        RTL_generated=False,RTL_changed=False,EDA_started=False,automatic_SYN_PNR=False,deployment_accepted=False,
        current54_input_or_controller_changed=False,
        measured_basis=dict(physical_build='PNR53',worst=worst['summary'],vendor_data_statistics=worst['vendor_data_statistics'],
                            actual_segment_DSP_interface_routes=travel,interface_route_sum_ns=sum(r['delay_ns'] for r in travel),
                            current_ring_distributed_primitives=len(ram),current_ring_primitive_kind='RAM16SDP4',
                            observer_address_MULT12X12=dsp),
        design_goal='Remove logical load_index increment/bounds/segment*28/subtract and slot*28/add from the accepted ring RAM read-data cone without adding any externally visible clock',
        recommendation='After both frozen layouts naturally finish and root admission, validate the combined stride32 plus11bit private cursor as one structural candidate; use compact-cursor only if measured memory placement cost rejects padding',
        exact_data_contract=dict(frame_words=28,history_frames=5,logical_input_words=168,word_bits=24,
            six_physical_slots_retained=True,valid_RAM_bits=4032,slot_roles=physical_relation,
            clip_rule='Only logical segment0 is clipped to signed +/-409600. Segment5 uses the same newest physical slot as segment0 but remains raw history.',
            old_new_memory_relation='For0<=slot<6 and0<=lane<28: newRAM[(slot<<5)|lane] == oldRAM[slot*28+lane], including arbitrary24bit/X initial values. No relation is imposed on hole words.',
            holes='Each32word slot has lanes28..31 unused. All writes and accepted read requests use lane0..27; holes are never consumers, never initialized as a proof shortcut.'),
        minimum_source_differences=[
            dict(file=str(observer),changes=['Only enabled ring branch: frame_mem depth168->192; write address becomes concatenated work_slot and original5bit lane',
                'Add private read_segment3/read_lane5/read_slot3 registers. Original work/live/frozen33bits remain unchanged.',
                'First-word speculative read selects live4,lane0. Other reads select private cursor. Direct concatenation is the physical RAM address.',
                'Original lane values/divider/write clocks, live COMMIT order and qualified frozen-pointer writes are unchanged; no memory reset/INIT.',
                'Clip decision comes from first-word phase or read_segment==0, never physical slot equality. Segment6 is an invalid sentinel producing0 after the final prefetch.']),
            dict(file=str(policy),changes=['Add two narrow wires to observer: peek_first=(state==SNAPSHOT), deliberately not health/deadline-qualified; advance only on the same accepted LOAD0..166 edge that captures snapshot_word.',
                'Keep original fully-qualified frame_freeze, snapshot_word capture enables, core write enable/address/history, all fiveFSM states, load_index8, tick_count, accepted_id, STOP/fault/deadline ordering and publication cycles.',
                'Existing frame_read_index may remain for default/reference branch, but the new cursor-enabled ring data address must have no combinational dependency on it.']),
            dict(file=str(owner),changes=['Connect two narrow phase/advance wires only. Keep sensor_age/ready/enable/core reset/action/IR/SAFE/lease contracts unchanged.'])],
        edge_schedule=[
            dict(edge='E_commit',before='All28 words in work_slot are already written',after='Original live pointer update and frame_valid=1. No new externally visible transaction. Next first-word read uses already registered live4.'),
            dict(edge='E_freeze',before='Policy SNAPSHOT and qualified frame_freeze. peek_first reads old live4,lane0 combinationally; no new reader FF is required to supply this first word.',after='Original snapshot_word captures word0; frozen0..4 copy old live0..4. Private cursor becomes segment0/lane1/slot old live4. Policy entersLOAD index0.'),
            dict(edge='E_LOAD_j, j0..165',before='Core writes OLD snapshot_word as logical wordj. Cursor describes logical wordj+1 before the edge.',after='snapshot_word captures wordj+1; private cursor advances to wordj+2 using its own small state, independently of load_index data decode.'),
            dict(edge='E_LOAD26,54,82,110,138',before='Prefetch consumes lane27 of the current segment',after='Set lane0 and next segment. On these boundaries choose respectively frozen0,frozen1,frozen2,frozen3,frozen4 for the following cycle.'),
            dict(edge='E_LOAD166',before='Core writes word166; cursor segment5/lane27 reads word167',after='snapshot_word captures word167; cursor becomes segment6 invalid sentinel, without addressing a hole.'),
            dict(edge='E_LOAD167',before='Core writes OLD word167; original frame_read_index would be168',after='snapshot_word held; no advance/read consumer; original START follows on its original edge.'),
            dict(edge='reset, disable, stale sensor, policy fault or deadline on any edge',before='Use original common reset and original capture/advance permission',after='Original abort/core-reset/actions-zero priority dominates; no private counter completion can publish or rearm. A later legal E_freeze overwrites all cursor bits before any acceptedLOAD.')],
        first_word_and_qualification=dict(
            true_commit='Keep original frame_freeze=SNAPSHOT&&frame_valid&&run_ok&&!deadline_missed for frozen pointer capture and existing snapshot capture',
            speculative_selector='peek_first is only state==SNAPSHOT; it may expose an unaccepted old/X RAM word while waiting or aborted. It must not write pointers, advance state, authorize motion or bypass a capture enable.',
            exact_public_relation='On every accepted E_freeze, both original and new read live4,lane0. On acceptedLOAD, cursor and frozen pointer invariant gives the same word. Outside accepted consumption only this private observer/policy read wire may differ.',
            scope='Paired observer+policy instance only. Preserve default flat branch and do not claim generic arbitrary external frame_read_index/frame_freeze module equivalence.',
            not_the_old_H2_only='H2 alone changes the health/freeze selector but leaves load_index->segment multiply/subtract, which PNR53 actually made worst. The cursor is essential to this proposal.'),
        private_cursor_reset_and_priority=dict(
            reset='Use the original observer asynchronous reset for the11new cursor bits: segment6 invalid sentinel, lane0, slot0. Do not reset/initialize the RAM or change existing pointer/reset roots.',
            priority='Original observer reset > qualified frame_freeze&&initialized initializes all cursor fields > qualified accepted LOAD0..166 advances > hold',
            no_extra_valid_FF='segment6 represents inactive/past-end; no twelfth cursor-valid FF is required',
            source_qualified_advance='state==LOAD && load_index!=167 && run_ok && !deadline_missed; this wire terminates at cursor FF control, not a live address arithmetic input',
            accepted_output_scope='Any reset/idle speculative read-wire differences are internal to the paired provider/consumer; all original accepted core writes, faults, actions and public publication edges must match.'),
        source_resource_estimates_not_mapping=[
            dict(variant='Current measured SYN43',new_explicit_FF=0,ring_RAM16SDP4=66,allocated_bits=4224,logical_words=168,address_DSPs=3,meaning='Actual current primitive inventory, not new candidate measurement'),
            dict(variant='A: private segment/lane/slot, compact28word slots',new_explicit_FF=11,FF_breakdown={'segment':3,'lane':5,'slot':3},
                ring_RAM16SDP4_estimate=66,allocated_bits_estimate=4224,new_BSRAM_target=0,
                remaining_address_math='slot*28+lane remains, including first word; segment*28 and logical-index subtraction disappear from RAM path',
                address_DSP_target=2,not_guaranteed='Tool may infer different arithmetic or share existing FFs; no measured gain yet'),
            dict(variant='B: stride32 plus private segment/lane/slot, recommended combined candidate',new_explicit_FF=11,
                FF_breakdown={'segment':3,'lane':5,'slot':3},new_control_wires_not_FFs=2,
                ring_RAM16SDP4_estimate=72,allocated_bits_estimate=4608,delta_RAM16SDP4_estimate=6,delta_allocated_bits_estimate=384,
                declared_array_extra_words=24,declared_array_extra_bits=576,new_BSRAM_target=0,
                address_DSP_target=0,read_address_ALU_target=0,
                remaining_read_math='RAM bank output selection and existing current clipping remain; small counter increment logic terminates at private FF D, not RAM RAD.',
                reset_load='At most11 new cursor reset loads if using original observer reset. No reset-load removal saving is credited.',
                not_guaranteed='72cells assumes full192x24 distributed shape is retained; optimizer hole pruning/packing and totalLUT/FF/DSP/CLS require actual mapping.'),
            dict(variant='Stride32 alone is insufficient',new_explicit_FF=0,ring_RAM16SDP4_estimate=72,
                remaining_address_math='Logical frame_read_index still needs /28 segment and index-segment*28. In particular the measured n22100_s may remain.',
                address_DSP_target=1,not_selected_as_standalone_candidate=True),
            dict(variant='Optional compact fully registered address cursor, only if padding proves costly',new_explicit_FF_estimate=[16,24],ring_RAM16SDP4_estimate=66,
                breakdown='segment3+lane5+linear-address8=16; optional precomputed first live-base8 adds8. Constant base selection/update occurs before use.',
                remaining_address_math='Original write address slot*28 remains; first-word base must be precomputed at COMMIT or supplied by a constant lookup bypass.',
                not_selected_as_parallel_variant=True)],
        concrete_rejected_shortcuts=[
            dict(shortcut='Physical lane=frame_read_index[4:0], segment=frame_read_index[7:5]',counterexample='Logical index28 must be history0/lane0. Low5bits28 point to a hole; index32 should be lane4 but low5bits0 reads another word.'),
            dict(shortcut='Capture a new read address only at E_freeze and use it for word0 on that same edge',counterexample='Nonblocking FF Q changes after the edge. snapshot_word sees the OLD address/word, causing a stale first word or requiring a forbidden extra clock.'),
            dict(shortcut='Initialize prefetch lane0 at E_freeze',counterexample='LOAD0 writes word0 then captures word0 again; LOAD1 incorrectly writes a duplicate0 instead of1. Initial private prefetch is lane1.'),
            dict(shortcut='Switch segment after LOAD27',counterexample='The source prefetch is one word ahead. The boundary must be after LOAD26 consumes word27 for preload; waiting until LOAD27 reads a hole or repeats the wrong word.'),
            dict(shortcut='Use physical newest-slot equality to enable clip',counterexample='First28 current words and last28 history words refer to the same slot. A raw value500000 must be current409600 but history500000. First-history repetition makes all five history pointers equal and must still remain raw.'),
            dict(shortcut='Copy old frozen pointers into a second private table on E_freeze',counterexample='Same-edge nonblocking updates read the previous frozen generation. Subsequent history can select the prior snapshot; use original live->frozen commit and read the updated frozen table at later boundaries.')],
        proof_obligations_before_any_new_mapping=[
            'Memory bijection over all6slots x28valid lanes with arbitrary24bit initial/data values; holes never consumed; write clock/lane_value unchanged',
            'Induction on accepted E_freeze/LOAD: cursor describes logical j+1, slot follows frozen4/0/1/2/3/4 order, lane0..27, segment6 only after final prefetch',
            'First COMMIT with !initialized repeats one frame in all5history roles; subsequent6slot wrap and repeated pointers retain exact history and current-only clipping',
            'Reset/disable/stale/fault/deadline exactly at COMMIT, E_freeze, LOAD26/27,54/55,138/139,166/167; no old cursor rearm or publicX acceptance',
            'Original real4500 comparison:168accepted input words,425trace values,6actions, frameID, capture/load/start/publish edges, including interrupted/rearmed and clipping extrema. No inference substitute or forced/zeroed private RAM',
            'Actual mapping: direct FF/phase-to-RAD concatenation; no segment/slot multiply or logical-index subtract in accepted read-address cone; prove relevant write/read/CE and all 24bit lane wiring',
            'Report actual total FF/LUT/SSRAM/BSRAM/DSP/CLS. Only actual subsequent routed paths determine improvement; do not claim either variant fixes NAV/PID/CPU negative paths'],
        mapping_and_physical_gate='No RTL is generated while54runs. After root source-contract admission use a unique new source; preserve4500weights/125Hz/clock targets/52constraints and all unrelated firmware. Measure one selected implementation before adding another.',
        input_bindings=[meta(p) for p in (observer,policy,owner,a.actual_observation,a.timing)],
        artifacts_generated=['result.json design plan only'])
    a.out.mkdir(parents=True,exist_ok=False)
    (a.out/'result.json').write_text(json.dumps(result,ensure_ascii=False,indent=2)+'\n',encoding='utf8')
    print(json.dumps(dict(plan_only=True,RTL_generated=False,EDA_started=False,new_FF_estimate=11,distributed_RAM_delta_estimate=6,BSRAM_delta_target=0)))


if __name__=='__main__':main()
