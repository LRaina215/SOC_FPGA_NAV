"""Produce reviewed complete SYN caches while an existing PNR keeps running.

Only the explicitly registered RTL delivery candidates are synthesized, one at
a time. No PNR or programming command exists in this producer. Completed SYNs
are reused, failed netlist checks are preserved, and --review-only rechecks an
existing SYN without rebuilding it. The consumer must use validate_ready_cache.
Metadata is byte length and modification time, never a content hash.
"""
import argparse
from datetime import datetime
import json
import os
from pathlib import Path
import shutil
import subprocess
import sys
import time
import traceback

import psutil

from retry_frozen_synthesis import memory_status


ROOT = Path(__file__).resolve().parents[2]
GIB = 1024**3
MIB = 1024**2
TERMINAL = {'ready_for_PNR', 'synthesis_failed', 'pre_pnr_review_failed',
            'prepared_source_review_failed', 'input_metadata_changed'}


def load(path):
    return json.loads(Path(path).read_text(encoding='utf-8'))


def write(path, data):
    path = Path(path)
    temporary = path.with_name(path.name+'.'+str(os.getpid())+'.tmp')
    temporary.write_text(json.dumps(data, ensure_ascii=False, indent=2)+'\n', encoding='utf-8')
    temporary.replace(path)


def now():
    return datetime.now().astimezone().isoformat()


def file_metadata(path):
    path = Path(path).resolve()
    stat = path.stat()
    assert path.is_file(), ('not a frozen file', str(path))
    return dict(path=str(path), size_bytes=stat.st_size, mtime_ns=stat.st_mtime_ns)


def _relative_files(root):
    """Inputs and completed artifacts used by the actual read-only reviewer."""
    manifest = load(root/'manifest.json')
    files = {root/'manifest.json'}
    for name in manifest.get('source_files', []):
        p = Path(name)
        p = p.resolve() if p.is_absolute() else (root/p).resolve()
        assert root in p.parents, ('input escapes its frozen directory', str(p))
        files.add(p)
    for p in root.rglob('*'):
        if not p.is_file() or '__pycache__' in p.parts or 'impl' in p.relative_to(root).parts:
            continue
        rel = p.relative_to(root)
        if p.suffix.lower() in {'.hex', '.mem', '.bin', '.mif', '.vh', '.h'}:
            files.add(p)
        elif rel.parts[0] == 'fw' and p.suffix.lower() in {'.c', '.s', '.ld', '.py'}:
            files.add(p)
    for name in ('run.tcl', 'g2.cst', 'g2.sdc', 'ip/serdes.csr',
                 'trim_profile.json', 'prepared_source_review.json', 'job.json',
                 'synthesis_summary.json', 'memory_inventory.json',
                 'impl/gwsynthesis/g2_top.vg', 'impl/gwsynthesis/g2_top.prj',
                 'effective_options.tcl'):
        p = root/name
        if p.is_file():
            files.add(p)
    profile = root/'trim_profile.json'
    if profile.is_file():
        data = load(profile)
        for proof in data.get('proofs', []):
            p = Path(proof.get('path', proof.get('result_file')))
            p = p.resolve() if p.is_absolute() else (root/p).resolve()
            files.add(p)
            result = load(p)
            bindings = proof.get('source_bindings') or result.get('source_bindings', [])
            for binding in bindings:
                q = Path(binding['proof_source'])
                files.add(q.resolve() if q.is_absolute() else (p.parent/q).resolve())
        provenance = data.get('firmware_provenance') or {}
        for name in ('compiled_elf', 'stack_analysis'):
            if provenance.get(name):
                p = Path(provenance[name])
                files.add(p.resolve() if p.is_absolute() else (root/p).resolve())
        for binding in provenance.get('output_bindings', []):
            p = Path(binding['compiled_output'])
            files.add(p.resolve() if p.is_absolute() else (root/p).resolve())
    return sorted(files, key=lambda p: str(p).casefold())


def frozen_input_metadata(build):
    """Stable, shared producer/consumer metadata contract; no hashes."""
    root = Path(build).resolve()
    return [file_metadata(p) for p in _relative_files(root)]


def validate_ready_cache(job_or_path, profile=None):
    """Return the complete ready cache job, or reject stale/incomplete evidence."""
    job = load(job_or_path) if isinstance(job_or_path, (str, Path)) else job_or_path
    assert job['schema_version'] == 1 and job['phase'] == 'ready_for_PNR', 'cache is not ready'
    prepared, synthesis = Path(job['prepared']).resolve(), Path(job['synthesis']).resolve()
    if profile is not None:
        assert job['profile_id'] == profile['id']
        assert prepared == Path(profile['prepared']).resolve()
        assert synthesis == Path(profile['synthesis']).resolve()
        assert Path(job['baseline']).resolve() == Path(profile['baseline']).resolve()
    check = Path(job['pre_pnr_checks']).resolve()
    result = load(check)
    assert result['passed'] and Path(result['source']).resolve() == synthesis
    assert Path(result['prepared']).resolve() == prepared
    assert result['exploratory_PNR_precheck_only'] and not result['deployment_accepted']
    expected = job['verified_input_metadata']
    assert expected['prepared'] == frozen_input_metadata(prepared), 'prepared sources/proofs changed after cache review'
    assert expected['synthesis'] == frozen_input_metadata(synthesis), 'SYN inputs/netlist/proofs changed after cache review'
    assert expected['pre_pnr_checks'] == file_metadata(check), 'cached review result changed'
    assert expected['reviewer_snapshot'] == file_metadata(job['reviewer_snapshot']), 'cached reviewer snapshot changed'
    assert load(synthesis/'job.json').get('phase') == 'synthesis_complete'
    return job


def identity(process):
    return dict(pid=process.pid, created_epoch=process.create_time(), command=process.cmdline())


def alive(record):
    try:
        p = psutil.Process(record['pid'])
        return abs(p.create_time()-record['created_epoch']) < .1
    except psutil.NoSuchProcess:
        return False


def existing_synthesis_processes(synthesis):
    """Observe only a wrapper with the exact output or its recorded real tool."""
    synthesis = Path(synthesis).resolve()
    found = {}
    for p in psutil.process_iter(['pid', 'cmdline']):
        try:
            cmd = p.info['cmdline'] or []
            if not any(Path(part).name == 'retry_frozen_synthesis.py' for part in cmd):
                continue
            if '--out' not in cmd:
                continue
            output = Path(cmd[cmd.index('--out')+1]).resolve()
            if output == synthesis:
                found[p.pid] = identity(p)
        except (psutil.NoSuchProcess, psutil.AccessDenied, ValueError, IndexError):
            pass
    file = synthesis/'job.json'
    if file.is_file():
        state = load(file)
        pid = state.get('tool_pid')
        if pid:
            try:
                p = psutil.Process(pid)
                created = datetime.fromisoformat(state['synthesis_started_at']).timestamp()
                if (abs(p.create_time()-created) < 10 and
                    p.name().lower() == 'gw_sh.exe' and Path(p.cwd()).resolve() == synthesis):
                    found[p.pid] = identity(p)
            except (psutil.NoSuchProcess, psutil.AccessDenied, KeyError):
                pass
    return list(found.values())


def stop_owned_tree(child):
    """Only the child launched by this producer and that child's descendants."""
    try:
        process = psutil.Process(child.pid)
        descendants = process.children(recursive=True)
        for p in reversed(descendants):
            try:
                p.kill()
            except psutil.NoSuchProcess:
                pass
        process.kill()
        child.wait(timeout=10)
    except psutil.NoSuchProcess:
        pass


def run_owned(command, directory, label, cpu_core, save,
              max_rss_mib=1536, max_private_mib=1536,
              minimum_phys_mib=1536, minimum_commit_mib=3072):
    log = directory/(label+'.log')
    started = now()
    with log.open('w', encoding='utf-8') as output:
        child = subprocess.Popen(command, cwd=ROOT, stdout=output, stderr=subprocess.STDOUT,
            creationflags=subprocess.CREATE_NO_WINDOW | subprocess.IDLE_PRIORITY_CLASS)
        process = psutil.Process(child.pid)
        process.nice(psutil.IDLE_PRIORITY_CLASS)
        process.cpu_affinity([cpu_core])
        owned = identity(process)
        save(active_child=owned, active_log=str(log), active_command=command)
        reason = None
        low = 0
        peak_rss = 0
        peak_private = 0
        while child.poll() is None:
            try:
                tree = [process]+process.children(recursive=True)
                infos = [p.memory_info() for p in tree if p.is_running()]
                rss = sum(m.rss for m in infos)
                private = sum(m.private for m in infos)
                peak_rss = max(peak_rss, rss)
                peak_private = max(peak_private, private)
            except psutil.NoSuchProcess:
                rss = private = 0
            memory = memory_status()
            danger = (memory.avail_phys < minimum_phys_mib*MIB or
                      memory.avail_page < minimum_commit_mib*MIB)
            low = low+1 if danger else 0
            if rss > max_rss_mib*MIB:
                reason = 'new_child_tree_RSS_limit'
            elif private > max_private_mib*MIB:
                reason = 'new_child_tree_private_commit_limit'
            elif low >= 2:
                reason = 'new_child_only_host_reserve_protection'
            if reason:
                stop_owned_tree(child)
                break
            time.sleep(1)
        code = child.wait()
    record = dict(command=command, pid=child.pid, created_epoch=owned['created_epoch'],
        started_at=started, ended_at=now(), tool_exit=code, cancelled=bool(reason),
        protection_reason=reason, peak_process_tree_RSS_bytes=peak_rss,
        peak_process_tree_private_commit_bytes=peak_private,
        maximum_process_tree_RSS_MiB=max_rss_mib,
        maximum_process_tree_private_commit_MiB=max_private_mib,
        minimum_runtime_host_physical_MiB=minimum_phys_mib,
        minimum_runtime_host_commit_reserve_MiB=minimum_commit_mib,
        priority='IDLE', cpu_core=cpu_core, log=str(log))
    write(directory/(label+'_runtime.json'), record)
    save(active_child=None, active_log=str(log), last_command_runtime=record)
    return record


def settings_only_resume_possible(synthesis, log, runtime=None):
    if runtime and runtime.get('protection_reason'):
        return False
    jobfile = synthesis/'job.json'
    if not jobfile.is_file():
        return False
    job = load(jobfile)
    text = log.read_text(encoding='utf-8', errors='replace') if log.is_file() else ''
    preflight = synthesis/'preflight.log'
    preflight_text = preflight.read_text(encoding='utf-8', errors='replace') if preflight.exists() else ''
    return (job.get('phase') == 'validating_frozen_settings' and 'tool_pid' not in job
        and 'TimeoutExpired' in text and 'ERROR' not in preflight_text
        and (synthesis/'effective_options.tcl').is_file()
        and not (synthesis/'impl').exists() and not (synthesis/'gw_sh.log').exists()
        and not (synthesis/'first_settings_timeout').exists())


def _source_binding_matches(synthesis, prepared):
    job = load(synthesis/'job.json')
    manifest = load(synthesis/'manifest.json')
    assert Path(job['source']).resolve() == prepared
    assert Path(manifest['frozen_source_build']).resolve() == prepared
    return job


def produce(profile, directory, cpu_core, review_only=False):
    prepared = Path(profile['prepared']).resolve()
    synthesis = Path(profile['synthesis']).resolve()
    baseline = Path(profile['baseline']).resolve()
    directory.mkdir(parents=True, exist_ok=True)
    jobfile = directory/'job.json'
    current = load(jobfile) if jobfile.is_file() else {}
    if current:
        assert current['profile_id'] == profile['id']
        assert Path(current['prepared']).resolve() == prepared
        assert Path(current['synthesis']).resolve() == synthesis
        if current['phase'] == 'ready_for_PNR' and not review_only:
            try:
                return validate_ready_cache(current, profile)
            except (AssertionError, FileNotFoundError, KeyError) as error:
                current.update(phase='input_metadata_changed', review_error=str(error), updated_at=now())
                write(jobfile, current)
                return current
        if current['phase'] in TERMINAL and not review_only:
            return current
    state = dict(schema_version=1, profile_id=profile['id'], prepared=str(prepared),
        synthesis=str(synthesis), baseline=str(baseline), source_frozen=True,
        controller_pid=os.getpid(), controller_created_epoch=psutil.Process().create_time(),
        programmed=False, deployment_accepted=False, PNR_started=False,
        review_attempt=int(current.get('review_attempt', 0)),
        phase='reserved', prior_phase=current.get('phase'))
    if current.get('prepared_input_metadata'):
        state['prepared_input_metadata'] = current['prepared_input_metadata']

    def save(phase=None, **extra):
        if phase:
            state['phase'] = phase
        state.update(updated_at=now(), **extra)
        write(jobfile, state)

    def memory_wait(phase):
        save(phase)
        while True:
            memory = memory_status()
            if memory.avail_phys >= 2*GIB and memory.avail_page >= 4*GIB:
                return memory
            time.sleep(15)

    save()
    write(directory/'profile_snapshot.json', profile)
    try:
        assert profile['kind'] == 'rtl_source' and profile['reviewer'] == 'review_delivery_trim_joint'
        assert profile.get('ready') and load(prepared/'prepared_source_review.json')['passed']
        admitted = frozen_input_metadata(prepared)
        if 'prepared_input_metadata' in state:
            assert state['prepared_input_metadata'] == admitted, 'prepared inputs changed since producer reserved this candidate'
        save('prepared_source_reviewing', prepared_input_metadata=admitted)
        memory_wait('waiting_for_prepared_review_memory')
        source_review = run_owned([sys.executable, '-X', 'utf8',
            str(ROOT/'integration/g2/review_delivery_trim_joint.py'), '--source', str(prepared),
            '--baseline', str(baseline), '--prepared-only', '--out', str(directory/'prepared_checks.json')],
            directory, 'prepared_source_review', cpu_core, save)
        if source_review['tool_exit'] != 0 or source_review['cancelled'] or not load(directory/'prepared_checks.json')['passed']:
            save('prepared_source_review_failed', review_error='prepared source check did not pass')
            return state
        assert admitted == frozen_input_metadata(prepared), 'prepared inputs changed during source review'
        if synthesis.exists():
            save('waiting_for_existing_unique_synthesis')
            while True:
                observed = existing_synthesis_processes(synthesis)
                if not observed:
                    break
                save(observed_synthesis_processes=observed)
                time.sleep(15)
            synjob = _source_binding_matches(synthesis, prepared)
            if synjob.get('phase') != 'synthesis_complete' or synjob.get('tool_exit') != 0:
                if not review_only and settings_only_resume_possible(synthesis, directory/'synthesis.log'):
                    resume = True
                else:
                    save('synthesis_failed', synthesis_job=synjob,
                         review_error='existing SYN incomplete/failed; refused duplicate actual synthesis')
                    return state
            else:
                resume = None
        else:
            assert not review_only, '--review-only requires an existing completed SYN'
            resume = False
        if resume is not None:
            memory = memory_wait('waiting_for_memory')
            command = [sys.executable, '-X', 'utf8', str(ROOT/'integration/g2/retry_frozen_synthesis.py'),
                str(prepared), '--out', str(synthesis), '--start-now', '--tool-priority', 'idle',
                '--tool-cpu-core', str(cpu_core), '--tool-max-rss-mib', '1536',
                '--tool-max-private-mib', '1536',
                '--minimum-runtime-physical-mib', '1536', '--minimum-runtime-commit-mib', '3072']
            if resume:
                command.append('--resume-settings-only')
            save('synthesis_running', start_free_bytes=memory.avail_phys,
                 start_commit_reserve_bytes=memory.avail_page)
            runtime = run_owned(command, directory, 'synthesis_settings_resume' if resume else 'synthesis', cpu_core, save)
            if (runtime['tool_exit'] != 0 and not resume and
                settings_only_resume_possible(synthesis, directory/'synthesis.log', runtime)):
                runtime = run_owned(command+['--resume-settings-only'], directory,
                    'synthesis_settings_resume', cpu_core, save)
            synjob = _source_binding_matches(synthesis, prepared)
            if runtime['tool_exit'] != 0 or runtime['cancelled'] or synjob.get('phase') != 'synthesis_complete' or synjob.get('tool_exit') != 0:
                save('synthesis_failed', synthesis_job=synjob, tool_runtime=runtime)
                return state
        assert admitted == frozen_input_metadata(prepared), 'prepared source/proofs changed during unique SYN'
        state['review_attempt'] += 1
        attempt = state['review_attempt']
        result_file = directory/f'pre_pnr_checks_attempt{attempt}.json'
        reviewer = ROOT/'integration/g2/review_delivery_trim_joint.py'
        snapshot = directory/'reviewer_snapshot.py'
        if snapshot.exists():
            shutil.copyfile(snapshot, directory/f'reviewer_before_attempt{attempt}.py')
        shutil.copyfile(reviewer, snapshot)
        before_review = frozen_input_metadata(synthesis)
        memory_wait('waiting_for_netlist_review_memory')
        save('synthesis_complete_reviewing', review_attempt=attempt,
             pre_pnr_checks=str(result_file), reviewer_snapshot=str(snapshot),
             synthesis_job=load(synthesis/'job.json'))
        runtime = run_owned([sys.executable, '-X', 'utf8', str(reviewer), '--source', str(synthesis),
            '--baseline', str(baseline), '--out', str(result_file)], directory,
            f'netlist_review_attempt{attempt}', cpu_core, save)
        if runtime['tool_exit'] != 0 or runtime['cancelled'] or not result_file.is_file() or not load(result_file).get('passed'):
            result = load(result_file) if result_file.is_file() else None
            save('pre_pnr_review_failed', review_error=result, review_runtime=runtime)
            return state
        assert before_review == frozen_input_metadata(synthesis), 'complete SYN changed while its netlist was reviewed'
        assert admitted == frozen_input_metadata(prepared), 'prepared source/proofs changed while reviewing complete SYN'
        verified = dict(prepared=admitted, synthesis=before_review,
            pre_pnr_checks=file_metadata(result_file), reviewer_snapshot=file_metadata(snapshot))
        save('ready_for_PNR', verified_input_metadata=verified, tool_runtime=load(synthesis/'job.json'),
             pre_pnr_checks=str(result_file), reviewer_snapshot=str(snapshot), review_error=None)
        validate_ready_cache(state, profile)
        return state
    except Exception as error:
        (directory/'validation_failure.log').write_text(traceback.format_exc(), encoding='utf-8')
        phase = 'input_metadata_changed' if 'changed' in str(error) else 'pre_pnr_review_failed'
        save(phase, review_error=str(error), review_error_type=type(error).__name__)
        return state


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--registry', type=Path, required=True)
    parser.add_argument('--state-dir', type=Path, required=True)
    parser.add_argument('--cpu-core', type=int, required=True)
    parser.add_argument('--review-only', metavar='PROFILE_ID')
    parser.add_argument('--validate-only', action='store_true')
    args = parser.parse_args()
    registry, folder = args.registry.resolve(), args.state_dir.resolve()
    specification = load(registry)
    profiles = [p for p in specification['profiles'] if p['kind'] == 'rtl_source']
    assert len(profiles) == 4 and len({p['id'] for p in profiles}) == 4
    for profile in profiles:
        assert profile['reviewer'] == 'review_delivery_trim_joint'
        assert Path(profile['parallel_synthesis_cache_job']).resolve() == folder/profile['id']/'job.json'
    assert args.cpu_core in psutil.Process().cpu_affinity(), 'selected spare CPU core is unavailable'
    if args.validate_only:
        print(json.dumps(dict(passed=True, profiles=[p['id'] for p in profiles], starts_tools=False,
            metadata_API='frozen_input_metadata(build); validate_ready_cache(job_or_path, profile=None)')))
        return 0
    folder.mkdir(parents=True, exist_ok=True)
    lock = folder/'producer.lock.json'
    if lock.exists():
        previous = load(lock)
        assert not alive(previous), ('a producer already owns this cache', previous)
        lock.rename(folder/('stale_producer_lock_'+str(time.time_ns())+'.json'))
    me = psutil.Process()
    me.nice(psutil.IDLE_PRIORITY_CLASS)
    me.cpu_affinity([args.cpu_core])
    descriptor = os.open(lock, os.O_CREAT | os.O_EXCL | os.O_WRONLY)
    with os.fdopen(descriptor, 'w', encoding='utf-8') as stream:
        json.dump(identity(me), stream, indent=2)
    state = dict(schema_version=1, controller_pid=me.pid, controller_created_epoch=me.create_time(),
        registry=str(registry), cpu_core=args.cpu_core, completed_profiles=[],
        single_serial_SYN_producer=True, maximum_simultaneous_new_SYN=1,
        PNR_started=False, programmed=False, deployment_accepted=False)
    try:
        if args.review_only:
            profiles = [p for p in profiles if p['id'] == args.review_only]
            assert len(profiles) == 1, 'unknown review-only profile'
        for profile in profiles:
            state.update(phase='producing_one_unique_SYN_cache', active_profile=profile['id'], updated_at=now())
            write(folder/'job.json', state)
            result = produce(profile, folder/profile['id'], args.cpu_core, bool(args.review_only))
            state['completed_profiles'].append(dict(profile_id=profile['id'], phase=result['phase'],
                cache_job=str(folder/profile['id']/'job.json')))
        state.update(phase='all_unique_SYN_caches_collected', active_profile=None, updated_at=now())
        write(folder/'job.json', state)
        return 0
    except Exception as error:
        state.update(phase='producer_stopped_without_PNR_or_duplicate_SYN', error=str(error), updated_at=now())
        (folder/'producer_failure.log').write_text(traceback.format_exc(), encoding='utf-8')
        write(folder/'job.json', state)
        return 1
    finally:
        if lock.exists() and load(lock)['pid'] == me.pid:
            lock.unlink()


if __name__ == '__main__':
    raise SystemExit(main())
