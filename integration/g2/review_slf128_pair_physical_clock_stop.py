"""Prepare/run an explicit-source SLF128 physical clock/26/stop review.

Default is preparation only and never parses VG/VO. --review is a separate
read-only action after natural PNR completion and actual2/4GiB admission. It
reuses selected_preservation through an isolated function globals dictionary;
no historical module path binding or script is changed.
"""
import argparse
from collections import Counter
from datetime import datetime
import inspect
import json
from pathlib import Path
import re
import time
import traceback
from types import FunctionType

import psutil
import review_model50_ring_pnr52_final as legacy
from audit_cdc_first_stages import sdc_fragment
from prepare_baseline_fusion_retime import metadata,same_bytes
from retry_frozen_synthesis import memory_status

GIB=1024**3
CLOCK=re.compile(r'CLK\d*|CLKA|CLKB|CLOCK|WCLK|RCLK')
REPO=Path(__file__).resolve().parents[2]


def load(path):return json.loads(path.read_text(encoding='utf-8-sig'))


def save(path,value):
    assert not path.exists(),('Evidence already exists',str(path))
    path.write_text(json.dumps(value,ensure_ascii=False,indent=2)+'\n',encoding='utf8')


def configuration(source,physical,out):
    reused=inspect.getsource(legacy.selected_preservation)
    assert 'SOURCE' in legacy.selected_preservation.__code__.co_names
    assert 'PHYSICAL' in legacy.selected_preservation.__code__.co_names
    assert 'memories(' not in reused and 'resource_report(' not in reused
    assert 'make_context(' in reused and 'first-Q fanout' in reused
    assert 'clockpins' in reused and 'verify_extra_guard_replicas' in reused
    dependencies=[Path(__file__),Path(legacy.__file__),
                  REPO/'integration/g2/related_clock_physical_review_common.py',
                  REPO/'integration/g2/review_model25_ring_system_pnr51_final.py',
                  REPO/'integration/g2/review_lease_tree_pnr50_final.py']
    return dict(prepared=True,review_executed=False,full_graph_parsed=False,
        source=str(source),physical=str(physical),out=str(out),
        source_VG=str(source/'impl/gwsynthesis/g2_top.vg'),physical_VO=str(physical/'impl/pnr/g2_top.vo'),
        natural_runtime_required=str(physical/'tool_runtime.json'),reference=str(physical/'cdc_reference_audit.json'),
        starts_EDA=False,writes_frozen_inputs=False,modifies_legacy_script_or_bindings=False,
        no_old_BSRAM_or_split_count_assumption=True,
        reused_function='review_model50_ring_pnr52_final.selected_preservation',
        binding_method='FunctionType with a copied globals dictionary and explicit SOURCE/PHYSICAL; original function globals untouched',
        checks=['Actual command_lease_guard/safety_guard hierarchy discovered from supplied source, plus IR remote boundary',
                'All selected primitive effective parameters/INIT and pins; only already-proved buffer, exact registered-copy, ADD commutation normalizations',
                'Original26 source/first/second primitive connection and firstQ sole actual secondD fanout',
                'Every nonconstant physical primitive clock pin root/inversion, only clk50 and rgmii_rxc, noPLL',
                'Additional check: every original nonconstant source clock-pin key still exists and has the same canonical root/inversion',
                'New physical clock pins inventoried explicitly; wide RAM split behavior remains the separate memory-only audit'],
        memory_review_separate='integration/g2/review_slf128_pair_physical_memories.py',
        timing_coverage_separate='integration/g2/review_pnr_timing_coverage.py',
        limitations=['No CDC payload route-delay/second-stage resolution-time/MTBF acceptance',
                     'No external reset assertion pulse/recovery-removal/reset release physical acceptance',
                     'No source-synchronous external IO budget or vehicle-stop acceptance',
                     'No ring/LF/weights actual physical functional reproof in this selected partition'],
        implementation_files=[metadata(p) for p in dependencies],deployment_accepted=False)


def finished(physical):
    runtime=load(physical/'tool_runtime.json')
    assert runtime.get('ended_at') and runtime.get('tool_exit') is not None,'PNR not naturally final; VG/VO not opened'
    assert runtime['cancelled'] is False,'Cancelled result is not natural completion'
    assert runtime['max_runtime_s']==runtime['max_no_observable_progress_s']==0
    pid=runtime['pid'];created=datetime.fromisoformat(runtime['started_at']).timestamp()
    current=psutil.Process(pid).create_time() if psutil.pid_exists(pid) else None
    assert current is None or abs(current-created)>2,('Original PNR tool identity remains alive',pid,current,created)
    return runtime,dict(pid=pid,original_created_epoch=created,current_created_epoch=current,original_identity_exited=True)


def review(source,physical,out,wait_memory=False):
    assert not out.exists();out.mkdir(parents=True)
    process=psutil.Process();state=dict(pid=process.pid,created_epoch=process.create_time(),phase='preflight',
                                      source=str(source),physical=str(physical),EDA_started=False)
    def phase(name,**kw):
        state.update(phase=name,at=datetime.now().astimezone().isoformat(),**kw)
        (out/'job.json').write_text(json.dumps(state,ensure_ascii=False,indent=2)+'\n',encoding='utf8')
    try:
        runtime,identity=finished(physical)
        source_job=load(source/'job.json')
        assert source_job['phase']=='synthesis_complete' and source_job['tool_exit']==0
        reference=load(physical/'cdc_reference_audit.json')
        assert reference['passed'] and len(reference['paths'])==26
        required=[x for x in sdc_fragment(reference).splitlines() if x.startswith('set_')]
        assert len(required)==52
        sdc=[x.strip() for x in (physical/'g2.sdc').read_text().splitlines() if x.strip() and not x.strip().startswith('#')]
        assert [x for x in sdc if x.startswith('set_')]==required
        assert len([x for x in sdc if x.startswith('create_clock ')])==2
        assert not any(x.startswith(('create_generated_clock','set_clock_groups','set_multicycle_path')) for x in sdc)
        assert 'create_clock -name clk50 -period 20 [get_ports {clk50}]' in sdc
        assert 'create_clock -name rgmii_rxc -period 10 [get_ports {rgmii_rxc}]' in sdc
        input_paths=[source/'job.json',source/'impl/gwsynthesis/g2_top.vg',physical/'g2_top.vg',
                     physical/'impl/pnr/g2_top.vo',physical/'g2.sdc',physical/'g2.cst',physical/'cdc_reference_audit.json',
                     physical/'tool_runtime.json',Path(legacy.__file__),REPO/'integration/g2/related_clock_physical_review_common.py']
        before=[metadata(p) for p in input_paths]
        assert same_bytes(source/'impl/gwsynthesis/g2_top.vg',physical/'g2_top.vg'),'Physical input is not supplied completed source'
        assert same_bytes(source/'g2.cst',physical/'g2.cst')
        while True:
            memory=memory_status()
            if memory.avail_phys>=2*GIB and memory.avail_page>=4*GIB:break
            phase('waiting_actual_review_memory',physical_bytes=memory.avail_phys,commit_bytes=memory.avail_page)
            assert wait_memory,'2GiB physical/4GiB commit admission not met; no graph parsed'
            time.sleep(15)
        phase('parse_explicit_source_and_physical',physical_bytes=memory.avail_phys,commit_bytes=memory.avail_page)
        held={}
        def context(*args,**kwargs):
            ctx=legacy.make_context(*args,**kwargs);held['context']=ctx;return ctx
        namespace=dict(legacy.selected_preservation.__globals__)
        namespace.update(SOURCE=source,PHYSICAL=physical,make_context=context)
        bound=FunctionType(legacy.selected_preservation.__code__,namespace,
                           'explicit_source_selected_preservation',legacy.selected_preservation.__defaults__)
        selected=bound(out,reference)
        ctx=held.pop('context');source_cells=ctx['source'];physical_pins=ctx['clockpins']
        checked=0;counts=Counter();missing=[];different=[];source_keys=set()
        for name,cell in source_cells.items():
            for pin,nets in cell['ports'].items():
                if not CLOCK.fullmatch(pin):continue
                for bit,net in enumerate(nets):
                    if net in ('GND','VCC'):continue
                    key=(name,pin,bit);source_keys.add(key)
                    if key not in physical_pins:
                        missing.append(dict(instance=name,pin=pin,bit=bit,source_net=net));continue
                    old=ctx['canonical'](source_cells,ctx['source_buffer_drivers'],net)
                    new=ctx['canonical'](ctx['physical'],ctx['physical_buffer_drivers'],physical_pins[key])
                    counts[old]+=1;checked+=1
                    if old!=new:different.append(dict(instance=name,pin=pin,bit=bit,source_root=old,physical_root=new))
        extra=[dict(instance=k[0],pin=k[1],bit=k[2],net=v) for k,v in physical_pins.items() if k not in source_keys]
        clocks=dict(passed=not missing and not different,original_source_nonconstant_clock_pin_count=len(source_keys),
                    matched_original_pin_count=checked,missing=missing,different_root_or_inversion=different,
                    original_root_counts=[dict(root=r,inverted=bool(p),pins=n) for (r,p),n in sorted(counts.items())],
                    additional_physical_clock_pin_count=len(extra),additional_physical_clock_pins=extra,
                    additional_pins_all_in_legacy_actual_root_inventory=True,
                    extra_RAM_split_and_nonselected_replica_semantics_separate=True)
        save(out/'all_original_clock_pin_role_comparison.json',clocks)
        assert clocks['passed'],('Original clock pin roles changed',missing[:8],different[:8])
        del ctx,source_cells,physical_pins
        unchanged=before==[metadata(p) for p in input_paths];assert unchanged,'Frozen input changed during read'
        result=dict(passed=True,preservation_review_completed=True,source=str(source),physical=str(physical),
            runtime=runtime,original_process_identity=identity,read_only=True,EDA_started=False,
            original26_source_first_second_and_firstQ_unique_load_checked=True,
            selected_stop_IR_clocks=selected,all_original_clock_pin_role_comparison=clocks,
            configured_52_requests_match_actual_reference=True,effective_Actived_status_requires_TR_review=True,
            no_BSRAM_or_split_count_assumption=True,memory_INIT_resources_review_separate=True,
            input_bindings=before,inputs_unchanged_after_read_only_review=unchanged,
            physical_CDC_payload_delay_resolution_MTBF_reset_pulse_IO_unproved=True,
            whole_design_functional_or_deployment_accepted=False)
        save(out/'result.json',result);phase('complete',passed=True)
        return result
    except BaseException as exc:
        phase('failed',error=f'{type(exc).__name__}: {exc}')
        save(out/'review_failed.json',dict(passed=False,error=str(exc),traceback=traceback.format_exc(),
                                         source=str(source),physical=str(physical),EDA_started=False))
        raise


def main():
    ap=argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--source',type=Path,required=True);ap.add_argument('--physical',type=Path,required=True)
    ap.add_argument('--out',type=Path,required=True);ap.add_argument('--review',action='store_true')
    ap.add_argument('--wait-for-memory',action='store_true');ap.add_argument('--prepare-out',type=Path)
    a=ap.parse_args();source=a.source.resolve();physical=a.physical.resolve()
    if not a.review:
        result=configuration(source,physical,a.out)
        if a.prepare_out:save(a.prepare_out,result)
        print(json.dumps(result,ensure_ascii=False));return
    assert a.prepare_out is None
    r=review(source,physical,a.out,a.wait_for_memory)
    print(json.dumps(dict(passed=r['passed'],result=str(a.out/'result.json'),deployment_accepted=False)))


if __name__=='__main__':main()
