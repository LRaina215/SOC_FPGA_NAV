"""Prepare the measured ring read-address redesign, never run EDA.

The default --plan mode renders three changed RTL strings in memory only. It
does not create a Verilog file. --materialize is a separate, explicit action
and is refused until the two frozen PNR53/54 runtimes ended naturally.
Source equivalence and actual mapping remain subsequent, independent gates.
"""
import argparse
from datetime import datetime
import difflib
import json
from pathlib import Path
import re
import shutil

from prepare_baseline_fusion_retime import metadata, same_bytes


BASE = Path('F:/nav_integration_g2_20261023')
SOURCE = BASE/'SLF128_storage_safeFW_syn43'
JOBS = (BASE/'SLF128_storage_place1_pnr53',
        BASE/'SLF128_storage_place2_pnr54')
CHANGED = ('control/rl_observation_v5.v', 'control/rl_policy_tick_v5.v',
           'control/mem_policy.v')
ASSETS = {'.hex', '.mem', '.bin', '.mif'}
AUTHORIZATION = 'materialize-only-after-natural-PNR53-PNR54-completion'


# Templates remain inside this Python file until the separate materialization
# action. No tool is launched by any mode of this script.
READER = '''        // Six32word slots: only lanes0..27 have consumers or writes.
        // A private one-word-ahead cursor replaces logical-index arithmetic.
        reg [2:0] read_segment;
        reg [4:0] read_lane;
        reg [2:0] read_slot;
        wire [7:0] write_address={work_slot,lane};
        // First word is read before its same-edge cursor initialization.
        // Speculation changes no capture, pointer, STOP or publication enable.
        wire [7:0] read_address=frame_read_peek_first ? {live4,5'd0} :
                                                       {read_slot,read_lane};
        wire signed [23:0] raw_word=frame_mem[read_address];
        assign frame_read_data=frame_read_peek_first ? clip_current(raw_word) :
            read_segment<6 ? (read_segment==0 ? clip_current(raw_word) : raw_word) :
                             24'sd0;
        always @(posedge clk or negedge rst_n) begin
            if(!rst_n) begin
                read_segment<=3'd6;read_lane<=0;read_slot<=0;
            end else if(frame_freeze && initialized) begin
                // word0 is captured now; the next LOAD prefetch needs word1.
                read_segment<=0;read_lane<=1;read_slot<=live4;
            end else if(frame_read_advance) begin
                if(read_segment<6) begin
                    if(read_lane==27) begin
                        read_lane<=0;read_segment<=read_segment+1'b1;
                        case(read_segment)
                            0: read_slot<=frozen0;
                            1: read_slot<=frozen1;
                            2: read_slot<=frozen2;
                            3: read_slot<=frozen3;
                            4: read_slot<=frozen4;
                            // After word167 the sentinel has no consumer.
                            default: read_slot<=read_slot;
                        endcase
                    end else read_lane<=read_lane+1'b1;
                end
            end
        end
'''

ADVANCE = '''    // Only the original LOAD snapshot-word capture advances its reader.
    // Keep the same procedural priority/4-state if behavior as that always:
    // no rst/enable/health/deadline bypass and no index-only speculation.
    assign frame_read_peek_first=OBS_RING_STORAGE!=0 && state==SNAPSHOT;
    always @* begin
        frame_read_advance=1'b0;
        if(!rst_n) begin end
        else if(!enable) begin end
        else if(!sensor_ok || fault!=0) begin end
        else begin
            if(!(tick_count==PERIOD_CLKS-1 && state!=IDLE)) begin
                case(state)
                    LOAD: if(OBS_RING_STORAGE!=0) begin
                        if(load_index!=167) frame_read_advance=1'b1;
                    end
                endcase
            end
        end
    end
'''


def once(text, old, new):
    if text.count(old) != 1:
        raise ValueError(('frozen anchor is not unique', old, text.count(old)))
    return text.replace(old, new, 1)


def derive_observer(text):
    new = once(text, '    input wire frame_freeze,',
               '    input wire frame_freeze,\n'
               '    input wire frame_read_peek_first,frame_read_advance,')
    new = once(new, 'reg signed [23:0] frame_mem [0:167];',
               'reg signed [23:0] frame_mem [0:191];')
    start = new.index("        wire [7:0] write_address=work_slot*8'd28+lane;")
    end = new.index('        always @(posedge clk) begin', start)
    old_read = new[start:end]
    for anchor in ("frame_read_index-segment*8'd28", "read_slot*8'd28+read_lane",
                   'segment==0 ? clip_current(raw_word) : raw_word'):
        assert anchor in old_read, anchor
    new = new[:start] + READER + new[end:]
    # The default flat branch, producer and pointer state machines are untouched.
    for begin, endmark in (('    always @(posedge clk or negedge rst_n) begin',
                            '    generate if (OBS_RING_STORAGE != 0)'),
                           ('        always @(posedge clk) begin',
                            '    end else begin: g_flat_default')):
        assert text[text.index(begin):text.index(endmark)] == new[new.index(begin):new.index(endmark)]
    assert text[text.index('    end else begin: g_flat_default'):] == new[new.index('    end else begin: g_flat_default'):]
    assert 'frame_read_index' not in new[new.index('    generate if'):new.index('    end else begin: g_flat_default')]
    return new


def derive_policy(text):
    new = once(text, '    output wire frame_freeze,',
               '    output wire frame_freeze,\n'
               '    output wire frame_read_peek_first,\n'
               '    output reg frame_read_advance,')
    anchor = "    assign frame_read_index=state==SNAPSHOT ? 8'd0 : load_index+8'd1;\n"
    new = once(new, anchor, anchor + ADVANCE)
    # Both sequential always blocks, including accepted SNAPSHOT/LOAD and every
    # abort/STOP/diagnostic/publication condition, are byte-identical strings.
    start = '    always @(posedge clk or negedge rst_n) begin'
    assert text[text.index(start):] == new[new.index(start):]
    assert text.count('if(load_index!=167) snapshot_word<=frame_read_data;') == 1
    assert new.count('if(load_index!=167) snapshot_word<=frame_read_data;') == 1
    return new


def derive_owner(text):
    new = once(text, '    wire observation_freeze;',
               '    wire observation_freeze;\n'
               '    wire observation_peek_first,observation_read_advance;')
    anchor = '        .frame_freeze(observation_freeze),.frame_read_index(observation_read_index),.frame_read_data(observation_read_data),'
    assert new.count(anchor) == 2
    add = '\n        .frame_read_peek_first(observation_peek_first),.frame_read_advance(observation_read_advance),'
    new = new.replace(anchor, anchor + add)
    assert new.replace(add, '').replace('    wire observation_peek_first,observation_read_advance;\n', '') == text
    return new


def paths_in_manifest(source):
    manifest = json.loads((source/'manifest.json').read_text(encoding='utf8'))
    paths = [Path(p).resolve() for p in manifest['source_files']]
    assert len(paths) == len(set(paths)) == 63
    rels = [p.relative_to(source).as_posix() for p in paths]
    assert set(CHANGED) <= set(rels)
    return manifest, dict(zip(rels, paths))


def scan_consumers(inputs):
    rows = []
    for rel, p in inputs.items():
        for line, text in enumerate(p.read_text(encoding='utf8').splitlines(), 1):
            # Remove line comments; identifiers are not macro-generated here.
            code = text.split('//', 1)[0]
            if re.search(r'\b(frame_read_data|frame_read_index|frame_freeze|observation_read_data)\b', code):
                rows.append(dict(file=rel, line=line, text=code.strip()))
    assert {r['file'] for r in rows} == set(CHANGED)
    # Exactly one source-level policy/observer instance, both in mem_policy.
    for module in ('rl_observation_v5', 'rl_policy_tick_v5'):
        found = []
        for rel, p in inputs.items():
            t = re.sub(r'//[^\n]*|/\*.*?\*/', '', p.read_text(encoding='utf8'), flags=re.S)
            found += [rel] * len(re.findall(r'\b'+module+r'\s*#\s*\(', t))
        assert sorted(found) == ['control/mem_policy.v', 'control/'+module+'.v'], (module, found)
    return rows


def kept_files(source):
    # Runtime/artifacts are not frozen source inputs. All remaining source,
    # FW, ELF/map, 69 inherited assets and 35 packed files are preserved.
    excluded = {'impl', '__pycache__', '.git'}
    runtime = {'job.json', 'tool_runtime.json', 'system_telemetry.jsonl'}
    return {p.relative_to(source).as_posix(): p for p in source.rglob('*')
            if p.is_file() and not excluded.intersection(p.relative_to(source).parts)
            and p.name not in runtime and p.suffix.lower() != '.log'}


def render(source):
    manifest, inputs = paths_in_manifest(source)
    transforms = (derive_observer, derive_policy, derive_owner)
    rendered = {rel: fn(inputs[rel].read_text(encoding='utf8'))
                for rel, fn in zip(CHANGED, transforms)}
    consumers = scan_consumers(inputs)
    files = kept_files(source)
    assets = sorted(r for r in files if Path(r).suffix.lower() in ASSETS)
    packed = [r for r in assets if r.startswith('src/mem/weight9_')]
    assert len(assets) == 104 and len(packed) == 35 and len(assets)-len(packed) == 69
    assert (source/'firmware/nav_fw.bin').stat().st_size == 53572
    assert len(manifest['source_files']) == 63
    changes = []
    for rel, new in rendered.items():
        old = inputs[rel].read_text(encoding='utf8')
        assert old != new
        diff = list(difflib.unified_diff(old.splitlines(), new.splitlines()))
        changes.append(dict(file=rel, source_bytes=inputs[rel].stat().st_size,
            generated_utf8_LF_bytes=len(new.encode('utf8')),
            old_lines=len(old.splitlines()), new_lines=len(new.splitlines()),
            removed_diff_lines=sum(s.startswith('-') and not s.startswith('---') for s in diff),
            added_diff_lines=sum(s.startswith('+') and not s.startswith('+++') for s in diff)))
    return manifest, inputs, files, assets, packed, rendered, consumers, changes


def natural_pair_complete():
    import psutil
    result = []
    for build in JOBS:
        p = build/'tool_runtime.json'
        runtime = json.loads(p.read_text(encoding='utf8'))
        assert runtime.get('ended_at') and runtime.get('tool_exit') is not None, ('PNR still in progress', build)
        assert runtime.get('cancelled') is False, ('PNR not a natural completion', build)
        assert runtime.get('max_runtime_s') == runtime.get('max_no_observable_progress_s') == 0
        command = runtime.get('command', [])
        assert command and Path(command[0]).name.lower() == 'gw_sh.exe'
        start = datetime.fromisoformat(runtime['started_at']).timestamp()
        end = datetime.fromisoformat(runtime['ended_at']).timestamp()
        assert end >= start
        pid = runtime['pid']
        same_process_live = False
        reused_pid = None
        try:
            proc = psutil.Process(pid)
            created = proc.create_time()
            # bounded_tool writes started_at immediately before Popen; an old
            # identical PID must be gone. A later reused PID is not ours.
            same_process_live = abs(created-start) < 60
            reused_pid = dict(pid=pid, created_epoch=created,
                              same_original_process=same_process_live)
        except psutil.NoSuchProcess:
            pass
        assert not same_process_live, ('original PNR process still exists', pid)
        job = json.loads((build/'job.json').read_text(encoding='utf8'))
        assert str(Path(job.get('build', job.get('output', str(build)))).resolve()).lower() == str(build.resolve()).lower()
        result.append(dict(build=str(build), runtime=metadata(p), ended_at=runtime['ended_at'],
            tool_exit=runtime['tool_exit'], cancelled=False, pid=pid,
            original_tool_process_exited=True, currently_reused_PID=reused_pid))
    return result


def relocate_paths(value, source, out):
    if isinstance(value, str):
        return value.replace(source.as_posix(), out.as_posix()).replace(str(source), str(out))
    if isinstance(value, list):
        return [relocate_paths(v, source, out) for v in value]
    if isinstance(value, dict):
        return {k: relocate_paths(v, source, out) for k, v in value.items()}
    return value


def disabled_tcl(source, out):
    old = (source/'run.tcl').read_text(encoding='utf8')
    lines = [line for line in old.splitlines()
             if not re.match(r'^\s*(run|exit)\b', line)]
    assert sum(bool(re.match(r'^\s*run\s+syn\s*$', x)) for x in old.splitlines()) == 1
    t = '\n'.join(lines)+'\n'
    t = relocate_paths(t, source, out)
    # First command refuses EDA, even if a tool runs the file by accident.
    t = ('error {Unvalidated ring stride32 private reader; automatic EDA disabled}\n'
         '# The following are frozen setup commands only; no run command.\n' + t)
    assert not re.search(r'^\s*run\b', t, flags=re.M)
    return t


def write_json(path, value):
    path.write_text(json.dumps(value, ensure_ascii=False, indent=2)+'\n',
                    encoding='utf8', newline='\n')


def materialize(a, design, data):
    assert a.authorization == AUTHORIZATION, 'explicit materialize-only authorization token required'
    # Gate BEFORE mkdir, source copying, .v writes, or metadata modification.
    pair = natural_pair_complete()
    source = a.source.resolve()
    out = a.candidate.resolve()
    assert not out.exists(), ('unique output required', out)
    assert out.parent == BASE.resolve(), 'candidate must be one new direct child of the frozen F root'
    assert source != out and not source.is_relative_to(out) and not out.is_relative_to(source)
    manifest, inputs, files, assets, packed, rendered, consumers, changes = data
    # Read set is bound before writes. Use size/mtime and exact bytes, no SHA.
    bindings = {r: metadata(p) for r, p in files.items()}
    out.mkdir()
    for rel, p in files.items():
        q = out/rel
        q.parent.mkdir(parents=True, exist_ok=True)
        shutil.copy2(p, q)
    for rel, text in rendered.items():
        (out/rel).write_text(text, encoding='utf8', newline='\n')
    (out/'run.tcl').write_text(disabled_tcl(source, out), encoding='utf8', newline='\n')
    new_manifest = relocate_paths(manifest, source, out)
    # Historical pass receipts are not current-candidate receipts. Only the
    # manifest and a new explicit pending contract describe this candidate.
    new_manifest.update(candidate=str(out), frozen_source_build=str(source),
        stage='prepared_only_unvalidated', source_preparation_only=True,
        ring_stride32_private_reader=True, modified_rtl_inputs=list(CHANGED),
        EDA_authorized=False, synthesis_complete=False, automatic_SYN=False,
        automatic_PNR=False, automatic_pnr=False, programmed=False,
        deployment_accepted=False, inherited_proof_scope='Ancestor only; new3RTL and mapping unvalidated')
    write_json(out/'manifest.json', new_manifest)
    same, rebound = [], ['run.tcl', 'manifest.json']
    for rel, old in files.items():
        now = metadata(old)
        assert now == bindings[rel], ('ancestor changed during copy', rel)
        if rel not in set(CHANGED) | set(rebound):
            assert same_bytes(old, out/rel), ('unexpected byte change', rel)
            same.append(rel)
    assert all(same_bytes(source/p, out/p) for p in assets)
    assert all(same_bytes(source/p, out/p) for p in inputs if p not in CHANGED)
    result = dict(design, phase='source_materialized_unvalidated', RTL_generated=True,
        candidate=str(out), natural_PNR53_PNR54_completion=pair,
        actual_changed_RTL=list(CHANGED), inherited_metadata_kept_as_ancestor_only=True,
        rebound_or_disabled_metadata=rebound,
        final_byte_identical_hardware_assets_FW_and_ancestor_receipts=same,
        all104_assets_byte_identical=True, all69_inherited_assets_byte_identical=True,
        all35_packed_weight_files_byte_identical=True, firmware_BIN53572_byte_identical=True,
        source_input_bindings=list(bindings.values()), candidate_input_bindings=[metadata(out/p) for p in inputs],
        candidate_new_RAM_mapping_and_semantics_pending=True, root_source_gate_pending=True)
    write_json(out/'ring_stride32_preparation_review.json', result)
    return result


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    mode = ap.add_mutually_exclusive_group()
    mode.add_argument('--plan', action='store_true', help='default: memory-only render and JSON receipt')
    mode.add_argument('--materialize', action='store_true', help='explicit gated source materialization; never EDA')
    ap.add_argument('--source', type=Path, default=SOURCE)
    ap.add_argument('--candidate', type=Path)
    ap.add_argument('--authorization', default='')
    ap.add_argument('--receipt', type=Path, required=True, help='new JSON only; never overwritten')
    a = ap.parse_args()
    a.source = a.source.resolve()
    assert a.source == SOURCE.resolve(), 'bind this preparer to the selected SYN43 ancestor'
    assert a.receipt.suffix.lower() == '.json' and not a.receipt.exists()
    assert not a.receipt.resolve().is_relative_to(a.source), 'never write into the frozen ancestor'
    data = render(a.source)
    manifest, inputs, files, assets, packed, rendered, consumers, changes = data
    design = dict(recorded_at=datetime.now().astimezone().isoformat(),
        phase='generator_prepared_only', generator=metadata(Path(__file__).resolve()),
        source=str(a.source), source_inputs=[metadata(p) for p in inputs.values()],
        original_consumer_inventory=consumers, exact_source_change_scope=changes,
        expected_changed_RTL=list(CHANGED), source_files=63, unmodified_RTL=60,
        inherited_assets=69, packed_weight_files=35, all_assets=104,
        assets_preserved=assets, firmware_BIN_bytes=53572,
        RTL_generated=False, simulation_started=False, SYN_started=False, PNR_started=False,
        automatic_SYN=False, automatic_PNR=False, deployment_accepted=False,
        selected_branch='OBS_RING_STORAGE!=0; default flat branch remains byte-identical',
        private_cursor=dict(segment_bits=3, lane_bits=5, slot_bits=3, total_explicit_FF=11,
            freeze='original frame_freeze&&initialized; cursor=segment0,lane1,old live4',
            advance='Exact original procedural LOAD capture priority, only indices0..166; LOAD167 holds',
            first_word='state==SNAPSHOT speculative live4,lane0; original accepted capture/freeze unchanged',
            last_word='word167 captured at LOAD166; cursor becomes invalid segment6; original LOAD167 writes held word167',
            clipping='logical segment0 only; segment5 newest history remains raw even if physical slot equals current',
            RAM_relation='newRAM[{slot,lane}]=oldRAM[slot*28+lane] for each slot0..5 and lane0..27, including arbitrary uninitialized content',
            no_hole_consumers=True, no_RAM_reset_or_INIT=True),
        mapping_estimate_only=dict(old_logical_words=168, new_physical_words=192, valid_words_per_slot=28,
            unused_words_per_slot=4, old_actual_RAM16SDP4=66, target_RAM16SDP4=72,
            added_allocated_RAM_bits=384, new_explicit_cursor_FF=11,
            old_address_MULT12X12=3, target_address_MULT12X12=0,
            actual_new_mapping_unknown=True, no_physical_timing_or_routing_claim=True),
        required_future_validation=[
            'Known-reset four-state old/new paired observer+policy: rejectX on every accepted168word and public valid; no forced/initialized RAM',
            'All six-slot wraps, first-history repeated slot IDs, current clipping vs raw newest-history, accepted word0 and lane27 boundaries',
            'Fault/disable/stale sensor/deadline at SNAPSHOT and everyLOAD boundary; cursor must hold when original capture does not occur',
            'Reset and explicit rearm during observation/division/LOAD/core; original STOP priority and publication edge',
            'Original real4500/weights/core input_mem:168 words,425 traces,6 actions,frameID,publication cycles; no inference substitute',
            'OBS_RING_STORAGE0 source branch and paired whole wrapper behavior retained',
            'Actual new mapping:all RAM semantics/INIT/CPU,72 distributedRAM estimate,11cursor FF provenance,zero addressDSP target,clipping/CE and unique consumers',
            'Only after source/mapping gates, root may select a new unconstrained-duration PNR; all setup/hold/CDC/reset/IO/STOP still required'],
        materialization_gate='Both exact frozen PNR53/54 runtimes ended naturally, original tool PID exited, explicit token, unique new F child output; natural failure is also completion',
        future_metadata_scope='Only3RTL plus relocated manifest/disabled run.tcl/new preparation receipt change; copied old receipts describe ancestor, never newcandidate passes')
    if a.materialize:
        assert a.candidate is not None, '--candidate required'
        result = materialize(a, design, data)
    else:
        assert a.candidate is None and not a.authorization, 'plan cannot create a candidate'
        result = design
    a.receipt.parent.mkdir(parents=True, exist_ok=True)
    write_json(a.receipt, result)
    print(json.dumps({k: result[k] for k in ('phase', 'RTL_generated', 'source_files', 'expected_changed_RTL',
        'all_assets', 'firmware_BIN_bytes', 'simulation_started', 'SYN_started', 'PNR_started')}, ensure_ascii=False))


if __name__ == '__main__':
    main()
