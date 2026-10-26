"""Read-only review of the frozen parser-offload plan and proposed strings.

This does not materialize RTL, execute the parser/C service, or run EDA.  It
distinguishes a counterexample in the retained guard from unimplemented future
call sites and assumptions that the stated exclusive FW owner already excludes.
"""
import argparse
import json
from pathlib import Path


REPO=Path(__file__).resolve().parents[2]


def meta(p):
    s=p.stat();return dict(path=str(p),bytes=s.st_size,mtime_ns=s.st_mtime_ns)


def ref(path,needle):
    text=path.read_text(encoding='utf-8-sig')
    assert needle in text,(path,needle)
    return dict(file=str(path),line=text[:text.index(needle)].count('\n')+1,text=needle)


def main():
    ap=argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--base',type=Path,default=Path('F:/nav_integration_g2_20261023'))
    ap.add_argument('--out',type=Path,required=True)
    a=ap.parse_args();base=a.base
    planpath=base/'P11_parser_offload_plan2/result.json'
    generator=REPO/'integration/g2/prepare_P11_parser_offload.py'
    guard=REPO/'integration/g2/P11_shared_FW_candidate/p11_command_guard.c'
    source=base/'SLF128_storage_safeFW_SYN43_prepared1'
    uart=source/'nav/rtl/io/imu_uart.v'
    busy=REPO/'docs/4_design/evidence/g2_20261023/CPU250_frontier_radius_CPU_final_20261026.json'
    plan=json.loads(planpath.read_text(encoding='utf8'))
    text=generator.read_text(encoding='utf8');gt=guard.read_text(encoding='utf8')
    assert plan['phase']=='plan_only_no_RTL_written' and not plan['RTL_materialized']
    assert 'if(a!=b)return P11_SNAPSHOT_UNSTABLE;' in text
    assert 'if(b==0xffffffffu||(uint32_t)(b-previous)!=1u)return P11_SNAPSHOT_LOST;' in text
    assert '(int32_t)(g->burst_us-g->authorized_us)<0' in gt
    busytext=busy.read_text(encoding='utf8');assert '3955.36' in busytext
    refs=dict(
        snapshot_four_reads=ref(generator,'uint32_t a=p11_snapshot_hal_seq();'),
        snapshot_strict_sequence=ref(generator,'if(b==0xffffffffu||(uint32_t)(b-previous)!=1u)return P11_SNAPSHOT_LOST;'),
        parser_reset=ref(generator,'if(rst||flush||!enable)begin'),
        parser_burst=ref(generator,'else if(have_burst&&!burst_used)begin'),
        owner_enable_authorization=ref(generator,"'sample authorization time AFTER enable ACK; previous_seq0; WAIT_ZERO'"),
        auth_signed_compare=ref(guard,'(int32_t)(g->burst_us-g->authorized_us)<0'),
        previous_arrival_compare=ref(guard,'if(g->have_arrival&&(int32_t)(g->burst_us-g->arrival_us)<=0)'),
        UART_first_start=ref(uart,'t_start <= time_us;'),
        UART_FIRST_latch=ref(uart,'first_pend <= (idle >= cfg_idle_us);'),
        UART_FIFO_producer=ref(uart,"push <= 1'b1; push_d <= {first_pend, sh};"),
        original_final_R0=ref(source/'fw/drv/hal_fpga.c','host_link_guard_filter(&g_host_link_guard,hal_time_us(),&guarded);'),
        original_motion=ref(source/'fw/nav/nav_task.c','nt->motion_result = motion_check(nt->map, &nt->mi, p, &nt->lp_cfg, &nt->cmd);'),
        original_support=ref(source/'fw/nav/nav_task.c','int ground=checked?SUPPORT_PASS:support_check(p,&nt->lp_cfg,&nt->mi,&nt->cmd);'),
        original_lease=ref(source/'g2_top.v','command_lease_guard #(.CLK_HZ(CLK_HZ)) u_command_lease('))
    result=dict(
        review_complete=True,mode='read_only_no_execution',RTL_generated=False,simulation_started=False,
        current_PNR_inputs_changed=False,implementation_or_deployment_accepted=False,
        plan_is_directionally_sound=True,
        overall='A bounded read-only snapshot adapter is reasonable. Retained long-session time ordering needs correction, and FW ownership/zero/filter integration plus source-switch drain remain future obligations. The sampled checkpoint timing alone does not justify predicting frequent normal50Hz locks.',
        findings=[
            dict(id='F1',classification='Confirmed retained-guard counterexample, conditional on reusing this comparison in the new snapshot guard',severity='availability_correctness',
                 title='Authorization-relative signed time order expires after half a32bit timer interval',
                 references=[refs['auth_signed_compare'],refs['previous_arrival_compare']],
                 trace=['Authorize at0x10000000; remain ARMED with fresh feedback/R0 and valid frames every20000us',
                        'At start_us0x90000000, start_us-authorized_us=0x80000000 interpreted signed negative, despite a fresh packet',
                        'All later fresh frames are rejected; accepted-arrival lease expires within200000us and locks'],
                 boundary_us=2147483648,boundary_time='35min47.483648sec since authorization; no full timer wrap is required',
                 scope='The new C_SERVICE only reads CSR and does not yet implement authorization checks; this is not a demonstrated unsafe command in the unmaterialized parser',
                 minimal_direction='Use authorization-relative order only while WAIT_ZERO, whose existing200ms deadline makes the modular interval unambiguous; once armed, compare current fresh arrival against the last accepted arrival. Keep owner/session reset and fresh zero requirements. Do not weaken current age limits.'),
            dict(id='F2',classification='Confirmed mailbox protocol tradeoff; global normal-workload failure rate is unverified',severity='availability_contract',
                 title='Strict consecutive SEQ imposes a new per-frame CPU service deadline',
                 references=[refs['snapshot_strict_sequence']],
                 trace=['CPU has consumed sequence10',
                        'Valid idle-separated50Hz packets11 and12 complete while service has a40ms gap',
                        'Four coherent reads obtain12; CRC/range/actual timestamp may all be fresh, but delta2 returns LOST and the planned FW locks'],
                 measured_frontier_checkpoint_gap_us=3955.36,proposed_sender_period_us=20000,
                 measured_scope='Existing finite frontier/Pico/guard/checkpoint sample only; not fullAPP or new snapshot service scheduling',
                 judgment='3955.36us is below20000us, so these measured checkpoints do not themselves predict skipped normal50Hz frames. Entire-APP worst service gap and actual hook remain unproved.',
                 required_decision='If velocity is latest-value semantics, consider monotonic skipped sequence as a diagnostic with unchanged freshness/zero/session filters. If every frame must be consumed, retain strictness and prove the service deadline or provide a bounded mailbox queue; do not call overwrite a transport corruption without this contract.'),
            dict(id='F3',classification='Confirmed receiver/policy boundary; safe under the stated timestamp and zero checks',severity='required_validation',
                 title='CTRL flush does not cancel an already started UART character',
                 references=[refs['parser_reset'],refs['UART_first_start'],refs['UART_FIRST_latch'],refs['owner_enable_authorization']],
                 trace=['FIRST character starts before authorization while imu_uart.busy_r=1',
                        'FW disables/flushes parser, changes baud, reenables, then captures authorization time',
                        'Old in-flight FIRST may later reach the parser and seed sequence1; its t_start remains pre-authorization',
                        'Post-authorization TS validation must reject that snapshot even if CRC completes later; SEQ1 alone is not a new-session proof'],
                 additional='Flush and FIRST on the same edge discard the character by reset priority; the parser may ignore the rest of that burst until another>=500us idle. This is conservative loss, not an unsafe replay.',
                 minimal_direction='Retain the actual start timestamp, WAIT_ZERO and accepted-sequence consumption on coherent but rejected snapshots. Add precise partial-character/flush/FIRST schedules to the future test; no need to reset the whole UART or existing raw queues to fake safety.'),
            dict(id='F4',classification='Unimplemented source-switch cleanup obligation, not an actual PNR/source defect',severity='integration_gap',
                 title='A P11 tap does not drain the old raw DATA/TS FIFOs before WIT return',
                 references=[refs['UART_FIFO_producer']],
                 fact='Original receiver continues pushing the raw DATA and FIRST timestamp queues; the offload tap neither pops nor backpressures them.',
                 condition='If the future P11 branch stops raw draining, these queues can fill/overflow while hardware snapshot operation remains healthy.',
                 gap='Plan2 says retained queues and baud switch, but does not specify the bounded paired drain/quiet/WIT decoder-reset protocol already required by the earlier owner plan.',
                 minimal_direction='Define a bounded count/overflow-aware DATA-FIRST/TS drain and quiet boundary before returning to WIT, and handle a physically in-flight character. Preserve original FIFO interfaces and keep movement zero throughout source switch.'),
            dict(id='F5',classification='Planned safety route is correct but future FW call sites are absent',severity='integration_gate',
                 title='R0/feedback/SAFE/lease protection cannot be inferred from a four-read adapter alone',
                 references=[refs['original_final_R0'],refs['original_motion'],refs['original_support'],refs['original_lease']],
                 fact='Proposed RTL produces only CSR command/TS/SEQ. No P11 net feeds SAFE, command_lease_guard or control UART targets. The selected image and selected firmware are untouched.',
                 gap='C_SERVICE returns a snapshot and has no R0/feedback/freshness/WAIT_ZERO/LOST zero handling; these belong in the unimplemented caller.',
                 required='Preserve guard polling even on NONE/UNSTABLE/LOST; consume coherent rejected sequence without renewing motion; stage proposals before common motion/support; retain final host_link_guard_filter, P11 zero filter, SAFE commit and original lease. Existing IDLE/ESTOP priority and the manual movement-enabled state must be explicit.' )],
        checked_without_found_counterexample=[
            dict(item='Four-read atomicity',conclusion='Under same-clock atomic CMD/TS/SEQ writes, no-IRQ single CTRL writer and saturating sequence, equal a/b proves no publication between the twoSEQ samples. No wrap ABA; mismatch yields no renewal. Actual HAL must issue four volatile MMIO reads.',
                 excluded='External/debug concurrent CTRL flush can reset SEQ and permit ABA, but the plan explicitly does not promise safety for that writer; this is not an in-contract defect.'),
            dict(item='Burst first-only',conclusion='Already present in the retained raw guard, rather than a new offload regression. A normal10byte/230400 frame takesabout434us;20ms spacing leaves plenty of>=500us idle. Continuous no-idle traffic intentionally expires rather than gaining a fabricated fresh timestamp.',
                 unverified='Actual RK serialized write cadence/USB fragmentation and source-switch first-character behavior are not measured by the frontier checkpoint sample.'),
            dict(item='Timer wrap',conclusion='Unsigned now-stamp age checks remain valid for these bounded200/250ms intervals across normal32bit wrap. snapshotSEQ provides validity, so timestamp0 is not intrinsically ambiguous here; retaining its conservative rejection loses at most that arrival if previousSEQ is still consumed.',
                 must_keep='Do not carry an unbounded signed authorization-time comparison into a long ARMED session.'),
            dict(item='SEQ saturation',conclusion='Code updates CMD/TS when changing FFFFFFFE to FFFFFFFF, then freezes all three. The adapter rejects FFFFFFFF. This is fail-closed and avoids sequence wrap ABA; it does not silently keep renewing a stale command.')],
        next_validation_order=[
            'First specify/fix the long-session time rule, source-switch drain and strict-sequence availability semantics in the plan; keep currentPNR53/54 inputs frozen',
            'After explicit root admission, test the materialized byte parser and old raw/WIT receiver with CTRL at every partial-character/FIRST/CRC boundary and FIFO-full/overflow conditions',
            'Exercise publication at each of the fourMMIO cuts; include coherent rejected nonzero WAIT_ZERO, sequence jumps, saturation and reset under the stated exclusive writer',
            'Use long-time virtual timestamps near2^31 since auth and32bit wrap without running35minutes of simulation',
            'Bind actual FW proposal/final publication hooks and prove R0 dead with continuous P11, stale feedback, IDLE/ESTOP, LOST/UNSTABLE plus busy checkpoints all retain zero/latched rearm',
            'Measure actual whole-APP service gaps and CPU publication with the selected sender cadence before deciding whether strict consecutive SEQ is deployable; retain350ms submission and500ms physical stop gates'],
        all_input_bindings=[meta(p) for p in (planpath,generator,guard,uart,busy,source/'fw/drv/hal_fpga.c',source/'fw/nav/nav_task.c',source/'g2_top.v')])
    a.out.mkdir(parents=True,exist_ok=False)
    (a.out/'result.json').write_text(json.dumps(result,ensure_ascii=False,indent=2)+'\n',encoding='utf8')
    print(json.dumps(dict(review_complete=True,RTL_generated=False,simulation_started=False,findings=len(result['findings']))))


if __name__=='__main__':main()
