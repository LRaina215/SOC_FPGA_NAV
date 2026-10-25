"""Audit an emitted VG preserved when host protection interrupted report generation.

This reads frozen artifacts only. It never resumes synthesis, invents a vendor
report, changes the original job, or accepts timing/function/deployment. A pass
means the existing structural netlist and its source provenance can be reviewed
independently before an exploratory implementation is considered.
"""
import argparse
from collections import Counter
from datetime import datetime
from functools import lru_cache
import json
from pathlib import Path
import re
import xml.etree.ElementTree as ET

import psutil

from audit_netlist_memories import BSRAM, INST, MODULE, PARAM
from revalidate_observation_routing import LIBRARY
from synthesis_connectivity import CELL, PORT

ASSETS = {'.hex', '.mem', '.mif', '.bin'}
DEFAULT = re.compile(r'^\s*parameter\s+(?:(?:integer|real)\s+)?(?:\[[^\]]+\]\s+)?(\w+)\s*=\s*([^;]+);', re.M)
LITERAL = re.compile(r"(\d+)'([bBdDhH])([0-9a-fA-F_]+)")


def load(path):
    return json.loads(Path(path).read_text(encoding='utf-8'))


def metadata(path):
    path = Path(path).resolve()
    stat = path.stat()
    return dict(path=str(path), size_bytes=stat.st_size, mtime_ns=stat.st_mtime_ns)


def rewrite(value, old, new):
    if isinstance(value, str):
        return value.replace(str(old), str(new)).replace(old.as_posix(), new.as_posix())
    if isinstance(value, list):
        return [rewrite(v, old, new) for v in value]
    if isinstance(value, dict):
        return {k: rewrite(v, old, new) for k, v in value.items()}
    return value


def resource_counts(primitive_counts, wide):
    c = primitive_counts
    source = sum(c[k] for k in BSRAM)
    return dict(counting_rule='expanded explicit VG leaf instances; no vendor T_* or DSP-internal register conversion',
        LUT2_3_4=sum(c[k] for k in ('LUT2', 'LUT3', 'LUT4')),
        DFF_primitives=sum(v for k, v in c.items() if k.startswith('DFF')),
        ALU=c['ALU'], RAM16SDP4=c['RAM16SDP4'], RAM16SDP1=c['RAM16SDP1'], ROM16=c['ROM16'],
        source_BSRAM=source, wide32_36_BSRAM=wide,
        assumed_exploratory_PNR_wide_split=1, predicted_physical_BSRAM_with_split=source+wide,
        physical_PNR_resources_measured=False)


def structural_inventory(path, include_memories=True):
    text = Path(path).read_text(encoding='utf-8')
    pairs = MODULE.findall(text)
    declarations = re.findall(r'^module\s+(\S+)', text, re.M)
    mods = dict(pairs)
    assert declarations == [name for name, body in pairs], 'unterminated/unsupported module declaration'
    assert len(pairs) == len(mods) == len(re.findall(r'^endmodule\b', text, re.M)), 'duplicate or incomplete module'
    assert 'g2_top' in mods and text.rstrip().endswith('endmodule /* g2_top */')
    library_text = LIBRARY.read_text(encoding='utf-8')
    library_modules = dict(MODULE.findall(library_text))
    primitive_names = set(re.findall(r'^module\s+(\w+)', library_text, re.M))
    defaults = {kind: {k: v.strip() for k, v in DEFAULT.findall(body)}
                for kind, body in library_modules.items()}
    children, parameters = {}, {}
    for kind, body in mods.items():
        assert not re.search(r'^\s*assign\s', body, re.M), 'unexpected structural assign'
        cells = CELL.findall(body)
        assert [(k, i) for k, i, ports in cells] == INST.findall(body), ('incomplete instance', kind)
        assert len(cells) == len({i for k, i, ports in cells}), ('duplicate instance', kind)
        children[kind] = [(k, i.lstrip('\\'), dict(PORT.findall(ports))) for k, i, ports in cells]
        pp = parameters[kind] = {}
        for inst, key, value in PARAM.findall(body):
            target = pp.setdefault(inst.strip().lstrip('\\'), {})
            assert key not in target, ('duplicate parameter', kind, inst, key)
            target[key] = value.strip()
    visiting, visited = set(), set()
    @lru_cache(None)
    def count(kind):
        assert kind not in visiting, ('hierarchy cycle', kind)
        visiting.add(kind)
        visited.add(kind)
        c, wide = Counter(), 0
        for child, inst, ports in children[kind]:
            if child in mods:
                cc, ww = count(child)
                c.update(cc)
                wide += ww
            else:
                assert child in primitive_names, ('unresolved non-vendor leaf type', child)
                c[child] += 1
                effective = dict(defaults.get(child, {}), **parameters[kind].get(inst, {}))
                if child in BSRAM and any(effective.get(k) in ('32', '36') for k in ('BIT_WIDTH', 'BIT_WIDTH_0', 'BIT_WIDTH_1')):
                    wide += 1
        visiting.remove(kind)
        return c, wide
    counts, wide = count('g2_top')
    assert len(visited) == len(mods), 'unreachable structural module definitions'
    memories = []
    def walk(kind, prefix):
        for child, inst, ports in children[kind]:
            path = prefix+'/'+inst if prefix else inst
            if child in mods:
                walk(child, path)
                continue
            if child not in BSRAM and not child.startswith('RAM') and child != 'ROM16':
                continue
            assert child in defaults, ('memory primitive defaults unavailable', child)
            explicit = parameters[kind].get(inst, {})
            effective = dict(defaults[child], **explicit)
            initial = {k: v for k, v in effective.items() if k.startswith('INIT')}
            assert initial, ('memory has no readable initialization contract', path)
            init_bits, nonzero = 0, []
            for key, value in initial.items():
                m = LITERAL.fullmatch(value)
                assert m, ('unknown/non-literal initialization', path, key, value)
                bits, base, digits = m.groups()
                number = int(digits.replace('_', ''), {'b':2, 'h':16, 'd':10}[base.lower()])
                assert number < 1 << int(bits), ('initialization overflows declared width', path, key)
                init_bits += int(bits)
                if number:
                    nonzero.append(key)
            geometry = {k: v for k, v in effective.items() if not k.startswith('INIT')}
            if child in {'RAM16SDP1', 'ROM16'}:
                geometry.update(fixed_depth_words=16, fixed_data_bits=1)
            elif child == 'RAM16SDP4':
                geometry.update(fixed_depth_words=16, fixed_data_bits=4)
            memories.append(dict(path=path, containing_module=kind, primitive=child,
                source_BSRAM=child in BSRAM, local_port_expressions=ports,
                explicit_parameters=explicit, effective_geometry_and_modes=geometry,
                effective_initialization=initial, initialization_declared_bits=init_bits,
                nonzero_initialization_parameters=nonzero,
                default_parameter_source=str(LIBRARY)))
    if include_memories:
        walk('g2_top', '')
    return dict(definition_integrity=dict(passed=True, module_declarations=len(declarations),
        complete_module_definitions=len(mods), endmodule_count=len(pairs), reachable_definitions=len(visited),
        top='g2_top', all_instances_parsed=True, hierarchy_acyclic=True,
        all_external_leaf_types_in_original_vendor_library=True, final_top_endmodule_present=True),
        primitive_counts=dict(sorted(counts.items())), resources=resource_counts(counts, wide),
        memory_primitive_count=len(memories), memory_leaves=memories)


def audit(source, prepared, runtime_path, reference):
    source, prepared, runtime_path, reference = [Path(p).resolve() for p in (source, prepared, runtime_path, reference)]
    job, manifest, runtime = load(source/'job.json'), load(source/'manifest.json'), load(runtime_path)
    assert Path(job['source']).resolve() == prepared and Path(manifest['frozen_source_build']).resolve() == prepared
    assert job['phase'] != 'synthesis_complete' and not job.get('tool_exit')
    assert not job['modified_rtl_inputs'] and not job['programmed']
    assert runtime['cancelled'] and runtime['protection_reason'] == 'new_child_only_host_reserve_protection'
    command = runtime['command']
    assert Path(command[command.index('--out')+1]).resolve() == source
    assert str(prepared) in command and '--start-now' in command
    active = []
    for pid, creation in ((runtime['pid'], runtime['created_epoch']), (job['tool_pid'], job['tool_created_epoch'])):
        try:
            p = psutil.Process(pid)
            if abs(p.create_time()-creation) < .1:
                active.append(dict(pid=pid, created_epoch=creation, command=p.cmdline()))
        except psutil.NoSuchProcess:
            pass
    assert not active, ('original synthesis is still active', active)
    original_manifest = load(prepared/'manifest.json')
    expected = rewrite(original_manifest, prepared, source)
    ignored = {'effective_options', 'frozen_source_build'}
    assert {k:v for k,v in manifest.items() if k not in ignored} == {k:v for k,v in expected.items() if k not in ignored}, 'unexpected frozen manifest change'
    inputs = {Path(name).resolve().relative_to(prepared).as_posix() for name in original_manifest['source_files']}
    assert len(inputs) == len(original_manifest['source_files']) == 68
    assets, headers, fw = set(), set(), set()
    for p in prepared.rglob('*'):
        if not p.is_file() or 'impl' in p.relative_to(prepared).parts or '__pycache__' in p.parts:
            continue
        name = p.relative_to(prepared).as_posix()
        if p.suffix.lower() in ASSETS:
            assets.add(name)
        elif p.suffix.lower() in {'.h', '.vh'}:
            headers.add(name)
        elif name.startswith('fw/') and p.suffix.lower() in {'.c', '.s', '.ld', '.py'}:
            fw.add(name)
    assert len(assets) == 104 and len(headers) == 31
    matched = inputs | assets | headers | fw | {'g2.cst', 'g2.sdc', 'ip/serdes.csr', 'trim_profile.json', 'prepared_source_review.json'}
    bindings = []
    for name in sorted(matched):
        a, b = prepared/name, source/name
        assert a.read_bytes() == b.read_bytes(), ('frozen input/proof changed', name)
        bindings.append(dict(relative_path=name, prepared=metadata(a), synthesis=metadata(b), bytes_identical=True))
    assert (source/'run.tcl').read_text(encoding='utf-8') == rewrite((prepared/'run.tcl').read_text(encoding='utf-8'), prepared, source)
    assert load(prepared/'prepared_source_review.json')['passed']
    vg = source/'impl/gwsynthesis/g2_top.vg'
    tracked = [source/'job.json', source/'manifest.json', source/'run.tcl', source/'effective_options.tcl',
        source/'gw_sh.log', source/'impl/gwsynthesis/g2_top.log', source/'impl/gwsynthesis/g2_top.prj', vg, runtime_path, LIBRARY]
    before = [metadata(p) for p in tracked]
    start, end = [datetime.fromisoformat(runtime[k]).timestamp() for k in ('started_at', 'ended_at')]
    assert start <= vg.stat().st_mtime <= end, 'netlist was not written by the recorded interrupted attempt'
    tool_log = (source/'gw_sh.log').read_text(encoding='utf-8', errors='replace')
    synthesis_log = (source/'impl/gwsynthesis/g2_top.log').read_text(encoding='utf-8', errors='replace')
    assert f'[95%] Generate netlist file "{vg}" completed' in tool_log
    assert not any(marker in tool_log for marker in ('[100%]', 'GowinSynthesis finish'))
    errors = re.findall(r'^.*\b(?:ERROR|FATAL)\b.*$', tool_log+'\n'+synthesis_log, re.M)
    assert not errors, errors[:6]
    reports = ['g2_top_syn_rsc.xml', 'g2_top_syn_resource.html', 'g2_top_syn.rpt.html']
    assert not any((source/'impl/gwsynthesis'/name).exists() for name in reports), 'this path only handles absent report-stage artifacts'
    project = ET.parse(source/'impl/gwsynthesis/g2_top.prj').getroot()
    assert project.tag == 'Project'
    inventory = structural_inventory(vg)
    reference_job = load(reference/'job.json')
    assert reference_job['phase'] == 'synthesis_complete' and reference_job['tool_exit'] == 0
    baseline = structural_inventory(reference/'impl/gwsynthesis/g2_top.vg', include_memories=False)
    keys = ('LUT2_3_4', 'DFF_primitives', 'ALU', 'RAM16SDP4', 'RAM16SDP1', 'ROM16', 'source_BSRAM', 'wide32_36_BSRAM')
    assert before == [metadata(p) for p in tracked], 'preserved artifact changed during read-only recovery audit'
    return dict(schema_version=1, passed=True, evidence_scope='structural emitted-netlist recovery and exact frozen-source binding only',
        source=str(source), prepared=str(prepared), recorded_at=datetime.now().astimezone().isoformat(),
        report_stage_interrupted=True, vendor_synthesis_completed=False, vendor_normal_exit_observed=False,
        original_job_unchanged=True, original_job_phase=job['phase'], original_tool_exit_available=job.get('tool_exit'),
        vendor_resource_report_available=False, missing_vendor_reports=reports,
        original_cancellation_runtime=runtime, original_cancellation_runtime_path=str(runtime_path),
        original_synthesis_processes_exited=True, tool_log_netlist_generation_completed=True,
        tool_and_synthesis_log_ERROR_FATAL_lines=errors, project_XML_complete=True,
        preserved_artifacts=before, source_bindings=bindings,
        frozen_hardware_inputs_identical=len(inputs), frozen_assets_identical=len(assets),
        frozen_headers_identical=len(headers), frozen_firmware_sources_identical=len(fw),
        all_source_bindings_bytes_identical=True, manifest_immutable_fields_identical=True,
        reference_completed_synthesis=str(reference), reference_VG_resources=baseline['resources'],
        resource_delta_from_reference={k:inventory['resources'][k]-baseline['resources'][k] for k in keys},
        **inventory, permits_independent_exploratory_review=True,
        actual_clocks_CDC_RAM_preservation_and_profile_scope_reviewed=False,
        exploratory_PNR_precheck_passed=False, synthesis_or_PNR_started=False,
        programmed=False, deployment_accepted=False,
        limitations=['No normal synthesis completion or vendor report is claimed.',
            'Counts are explicit VG leaves; DSP-internal/combined vendor resource conversions are excluded.',
            'RAM INIT evidence includes original vendor defaults; it is not a hardware initialization test.',
            'Source/INIT/clock/CDC/profile checks and PNR signoff still require the independent joint reviewer.'])


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--source', type=Path, required=True)
    parser.add_argument('--prepared', type=Path, required=True)
    parser.add_argument('--runtime', type=Path, required=True)
    parser.add_argument('--reference-completed', type=Path, required=True)
    parser.add_argument('--out', type=Path, required=True)
    args = parser.parse_args()
    assert not args.out.exists(), 'preserve earlier recovery evidence; choose a new output'
    me = psutil.Process()
    me.nice(psutil.IDLE_PRIORITY_CLASS)
    if 16 in me.cpu_affinity():
        me.cpu_affinity([16])
    result = audit(args.source, args.prepared, args.runtime, args.reference_completed)
    args.out.write_text(json.dumps(result, ensure_ascii=False, indent=2)+'\n', encoding='utf-8')
    print(json.dumps({k:result[k] for k in ('passed', 'source', 'vendor_synthesis_completed', 'resources', 'resource_delta_from_reference', 'memory_primitive_count')}, ensure_ascii=False))
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
