"""Read existing SYN40/43 timing and source; propose no RTL changes or EDA run."""
import argparse
from html.parser import HTMLParser
import json
from pathlib import Path
import re

from synthesis_connectivity import CELL, PORT


class Rows(HTMLParser):
    def __init__(self):
        super().__init__(); self.rows=[]; self.row=None; self.cell=None
    def handle_starttag(self,tag,attrs):
        if tag=='tr': self.row=[]
        if tag in ('td','th') and self.row is not None: self.cell=[]
    def handle_endtag(self,tag):
        if tag in ('td','th') and self.cell is not None:
            self.row.append(''.join(self.cell).strip()); self.cell=None
        if tag=='tr' and self.row is not None:
            self.rows.append(self.row); self.row=None
    def handle_data(self,data):
        if self.cell is not None: self.cell.append(data)


def meta(p):
    s=p.stat(); return dict(path=str(p),bytes=s.st_size,mtime_ns=s.st_mtime_ns)


def first_path(path):
    parser=Rows(); parser.feed(path.read_text(encoding='utf8'))
    rows=parser.rows
    start=next(i for i,r in enumerate(rows) if r and r[0]=='Slack')
    header=next(i for i in range(start,len(rows)) if rows[i][:3]==['AT','DELAY','TYPE'])
    end=next(i for i in range(header+1,len(rows)) if rows[i][:3]==['AT','DELAY','TYPE'])
    summary=dict(rows[start:header])
    data=[dict(zip(('arrival_ns','delay_ns','type','transition','fanout','node'),r))
          for r in rows[header+1:end] if len(r)==6]
    comb=[r for r in data if r['type']=='tINS' and r['node']!='clk50_ibuf/O']
    clock=next(r for r in rows if len(r)==6 and r[:2]==['1','clk50'] and 'MHz' in r[2])
    return dict(summary=summary, clock_summary=clock, data=data, combinational_output_rows=comb)


def extract_modules(vg,names,out):
    found={}; active=None; buffer=[]
    with vg.open(encoding='utf8') as f:
        for line in f:
            m=re.match(r'module\s+(\w+)\s*\(',line)
            if m and m.group(1) in names: active=m.group(1);buffer=[line]
            elif active: buffer.append(line)
            if active and re.match(r'\s*endmodule\b',line):
                text=''.join(buffer); p=out/(active+'.v')
                if p.exists(): assert p.read_text(encoding='utf8')==text
                else: p.write_text(text,encoding='utf8')
                found[active]=text; active=None;buffer=[]
            if len(found)==len(names): break
    assert set(found)==set(names)
    return found


def main():
    ap=argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--base',type=Path,default=Path('F:/nav_integration_g2_20261023'))
    ap.add_argument('--out',type=Path,required=True)
    a=ap.parse_args(); a.out.mkdir(parents=True,exist_ok=True)
    b=a.base; prepared=b/'stage3_SLF128_storage_RTL_prepared8'
    report43=b/'SLF128_storage_safeFW_syn43/impl/gwsynthesis/g2_top_syn.rpt.html'
    report40=b/'baseline_fusion50_ring_MAX_syn40/impl/gwsynthesis/g2_top_syn.rpt.html'
    vg=b/'SLF128_storage_safeFW_syn43/impl/gwsynthesis/g2_top.vg'
    path43=first_path(report43);path40=first_path(report40)
    assert path43['summary']['Data Arrival Time']=='27.028'
    assert path43['summary']['From']=='u_control/model_thread/sensor_age_6_s1'
    assert path43['summary']['To']=='u_control/model_thread/policy/snapshot_word_14_s0'
    assert path43['summary']==path40['summary']
    assert path43['combinational_output_rows']==path40['combinational_output_rows']
    modules=extract_modules(vg,{'mem_policy','rl_observation_v5','rl_policy_tick_v5'},a.out)
    cells={}
    for module,body in modules.items():
        param_by_cell={}
        for name,parameter in re.findall(r'^defparam\s+(\\\S+|\w+)\s*\.(.*);\s*$',body,re.M):
            param_by_cell.setdefault(name.lstrip('\\'),[]).append(parameter)
        for kind,name,ports in CELL.findall(body):
            name=name.lstrip('\\')
            parameters=param_by_cell.get(name,[])
            cells[(module,name)]=dict(kind=kind,name=name,ports=dict(PORT.findall(ports)),parameters=parameters)
    chain=[]
    for row in path43['combinational_output_rows']:
        relative=row['node'].removeprefix('u_control/model_thread/')
        owner='mem_policy'
        if relative.startswith('observation/'):
            owner='rl_observation_v5';relative=relative.removeprefix('observation/')
        name=relative.rsplit('/',1)[0]
        cell=cells[(owner,name)]
        chain.append(dict(**row,owner=owner,actual_cell=cell))
    source_files=['control/mem_policy.v','control/rl_observation_v5.v',
                  'control/rl_policy_tick_v5.v','control4500_core.v']
    identical=[]
    for name in source_files:
        lhs=prepared/name;rhs=b/'baseline_fusion50_ring_MAX_syn40'/name
        assert lhs.read_bytes()==rhs.read_bytes()
        identical.append(dict(path=name,actual43=meta(lhs),SYN40=meta(rhs),byte_identical=True))
    stages=[
        ('launch_clock_and_Q',0,.715,'source sensor_age[6] DFFCE'),
        ('32bit_age_threshold',.715,5.743,'six LUTs model_status_flags_3_s7/s6/s5/s2/s0/s'),
        ('health_ready_qualification',5.743,7.399,'n414_s2 and model_ready_s'),
        ('freeze_transaction_qualification',7.399,8.281,'observation/n22350_s1'),
        ('live_frozen_and_segment_slot_select',8.281,12.678,'five read_slot LUTs'),
        ('slot_times28',12.678,14.893,'MULT12X12 n22165_s, all registers BYPASS'),
        ('lane_offset_addition',14.893,17.046,'two ALU cells in read_address[2] cone'),
        ('distributed_RAM_read',17.046,17.874,'RAM16SDP4 asynchronous RAD->DO'),
        ('RAM_bank_output_selection',17.874,21.314,'four observation_read_data_0 LUTs'),
        ('current_clip_and_public_word_select',21.314,26.657,'six observation_read_data_14 LUTs'),
        ('capture_D_net',26.657,27.028,'snapshot_word[14] data pin')]
    result=dict(
        complete=True,readonly=True,RTL_changed=False,EDA_started=False,
        physical_timing_accepted=False,
        confirmed=dict(synthesis_arrival_ns=27.028,reported_logic_level=29,
            listed_combinational_primitives=len(chain),
            chain_is_not_29_LUTs=True,
            actual_domains='This selected source is the 50MHz control/NAV base, not model25',
            source_sensor_timeout_cycles=2500000,source_period_cycles=400000,
            identical_to_SYN40_reported_path=True,
            meaning='The same control read path predates the SLF128 storage migration; neither timing closure nor physical cause follows from this synthesis estimate'),
        path_stages=[dict(stage=n,from_ns=f,to_ns=t,increment_ns=round(t-f,3),actual_structure=s) for n,f,t,s in stages],
        source_references=[
            dict(file=str(prepared/'control/mem_policy.v'),lines='40-61',purpose='32bit saturating age, sensor_ok, stop_latched and ready'),
            dict(file=str(prepared/'control/rl_policy_tick_v5.v'),lines='61-65,99-127',purpose='Qualified freeze, current read index, capture/abort priority and old-word LOAD'),
            dict(file=str(prepared/'control/rl_observation_v5.v'),lines='59-65,153-200',purpose='Current-only clip, six-slot RAM, live/frozen pointer selection and simultaneous pointer freeze')],
        unnecessary_cross_boundary_dependency=dict(
            confirmed_source_relation='frame_freeze = SNAPSHOT && frame_valid && run_ok && !deadline_missed; each rN selects liveN versus frozenN with frame_freeze',
            consequence='The eligibility check controls a RAM read address and subsequent clipping before the same eligibility also permits snapshot_word capture',
            judgment='This is a concrete avoidable source dependency under the paired observer/policy consumer contract, not a proven false timing path or permission for an STA exception',
            unchanged_needed='Keep immediate health/stop qualification on capture/LOAD/core reset/action outputs and keep the qualified pointer-freeze event'),
        original_edge_semantics=[
            dict(edge='E_capture',before='Policy capture was asserted on the preceding edge; observer IDLE samples current sensors, commands and executed actions',after='Original numerators/divider work are frozen; sensor_age concurrently uses old sample_valid for reset/increment'),
            dict(edge='E_commit',before='Observer has completed all28 raw words in work_slot',after='live pointers and valid/initialized update together; policy on this same edge still sees old frame_valid/live values'),
            dict(edge='E_freeze',before='Policy SNAPSHOT sees frame_valid=1 and current run_ok=1 without deadline; read index0 uses old live4, raw RAM value is clipped',after='snapshot_word captures word0 and frozen0..4 capture the same old live pointers; load_index becomes0, state becomesLOAD'),
            dict(edge='E_LOAD_j for j0..166',before='Core consumes old snapshot_word as wordj; read indexj+1 uses frozen pointers',after='snapshot_word captures wordj+1; index advances'),
            dict(edge='E_LOAD_167',before='Core consumes old word167; computed read index168 is invalid',after='snapshot_word is held; START follows on the original edge'),
            dict(edge='threshold crossing',before='age=T-1, sample_valid=0; current sensor_ok is still fresh',after='age becomesT and combinational freshness immediately falls; next stop/policy edge must see stale, without a one-clock delay'),
            dict(edge='new sample at stale age',before='sample_valid=1 with age=T; old policy/stop logic still evaluates old stale state',after='age becomes0 and freshness becomes true; sticky stop/fault recovery priorities are unchanged')],
        hypotheses=[
            dict(id='H1_exact_redundant_freshness_register',status='proposed_only',priority=2,
                transformation='Retain the full32bit sensor_age counter and add a redundant stale FF whose next value equals the predicate on the counter next-state, rather than registering current sensor_ok',
                exact_invariant='stale(t) == (sensor_age(t)>=T). next_age = sample_valid ?0 : age<T ?uint32(age+1) : age; next_stale=(next_age>=T)',
                production_simplification='For positive T=2500000, reset age=T/stale=1; sample_valid gives age0/stale0; otherwise stale_next=stale||(age==T-1). T0 and generic signed/width corners need explicit original-expression fallback',
                preserved_sampling='Both new FFs describe the same post-edge age; stop_latched and policy consume the old stale value on the current edge exactly as before. Naive stale<=age>=T or sensor_ok<=old_sensor_ok is one edge late and rejected',
                intended_check='Existing actual chain six-stage age comparison no longer lies between a counter FF and the snapshot read address; its new cone terminates at redundant stale FF. Other immediate health inputs remain combinational',
                validation=['One-step32bit invariant for all age values under the stale/age relation; reset, T0/T1/power boundaries and production T',
                    'Cycle/four-state comparison at T-2/T-1/T with sample_valid coincident with capture/commit/freeze, sensor error and enable/stop transitions',
                    'Only after source proof, actual mapping must show preserved32bit age and exact FF boundary; actual PNR decides timing benefit']),
            dict(id='H2_speculative_read_selector_separate_from_commit',status='proposed_only',priority=1,
                transformation='Provide a narrow read-live selector from policy state==SNAPSHOT, independent of health/deadline qualification. Use it only for live/frozen read-address selection. Retain original fully qualified frame_freeze for frozen-pointer writes and original snapshot_word/LOAD enables',
                exact_valid_relation='Whenever the original policy actually captures its first word, qualified frame_freeze=1 and state==SNAPSHOT=1, so old/new read pointers and word0 are identical. In LOAD, both selectors choose frozen pointers. Other differences are speculative unread data, never a captured public word',
                preserved_sampling='E_freeze still captures old live pointers and word0 simultaneously; E_LOAD0 still writes old word0; no extra pipeline clock, no delayed abort, no changed reset or recovery. Uninitialized speculative data may beX while capture is disabled; valid accepted output must still rejectX',
                intended_check='The current sensor_age/health chain is absent from RAM RAD and clip data cones; health remains on commit/capture/core reset. State decode and ring read/clip path still need actual mapping and physical timing',
                scope='Paired mem_policy observer/policy contract only. Do not replace a generic observer interface behavior for arbitrary external frame_read_index/frame_freeze callers without a separate contract',
                validation=['Boolean/transition miter: every original accepted snapshot_word or core write sees the same word and ID; pointer writes and fault priority unchanged',
                    'Actual full4500 ring comparison including first history replication, all168 inputs, all425 trace neurons and6 actions; reset at E_commit/E_freeze/LOAD and stale/deadline on the same edges',
                    'Actual mapping source-cone audit before one bounded new synthesis candidate; no false-path/multicycle constraint, no current PNR input changes'])],
        recommended_order=['Let current PNR53/54 finish naturally and use their full routed setup/hold paths to determine whether this synthesis cone remains critical',
            'If the same read path is critical, try H2 alone first because it preserves all original registers and eliminates control eligibility from speculative data selection',
            'H1 is a separate single-factor fallback or follow-up for timer/stop/ready paths; do not stack both before measuring actual mapping',
            'Do not add a generic extra read pipeline stage or sample sensor_ok one cycle late; either changes the original load/stop contract'],
        evidence=dict(report43=meta(report43),report40=meta(report40),VG43=meta(vg),source_identity=identical,
            actual_local_modules=[meta(a.out/(n+'.v')) for n in sorted(modules)]),
        detail_path=str(a.out/'actual_first_path_cells.json'))
    (a.out/'actual_first_path_cells.json').write_text(json.dumps(dict(path43=path43,path40=path40,actual_chain_cells=chain),ensure_ascii=False,indent=2)+'\n',encoding='utf8')
    (a.out/'result.json').write_text(json.dumps(result,ensure_ascii=False,indent=2)+'\n',encoding='utf8')
    print(json.dumps(dict(complete=True,readonly=True,arrival_ns=27.028,listed_combinational_primitives=len(chain),identical_SYN40=True,RTL_changed=False,EDA_started=False)))


if __name__=='__main__':
    main()
