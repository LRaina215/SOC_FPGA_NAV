"""Keep invalid IMU MMIO payload deterministic without touching FIFO storage.

The real CPU diagnostic returned VALID=0 with X payload. GCC's signed validity
branch then propagated X in the four-state PicoRV32 model. Valid data, pop,
timestamp, firmware, capacities and clocks are unchanged by this candidate.
"""
from pathlib import Path
from datetime import datetime
import argparse
import copy
import json
import shutil

from prepare_baseline_fusion_retime import hardware_inputs, metadata, relocate, same_bytes, save

SOURCE = Path('F:/nav_integration_g2_20261023/baseline_fusion_preview_safety250_prepared2')
PROBE = Path('F:/nav_integration_g2_20261023/CPU250_startup_diagnostic1')


def prepare(out):
    assert not out.exists()
    diagnostic = json.loads((PROBE/'job.json').read_text())
    log = (PROBE/'3_vsim.log').read_text(errors='replace')
    assert diagnostic['phase'] == 'finite_40ms_diagnostic_complete'
    assert 'a=6004 we=0' in log and 'q=00000Xxx' in log
    assert 'DIAG_END_40MS loops=0' in log and 'stack40=40006004' in log
    assert 'DIAG_UNEXPECTED_IDLE_IMU' not in log
    original_review = json.loads((SOURCE/'prepared_source_review.json').read_text())
    assert original_review['passed']
    for row in original_review['bindings']:
        assert metadata(row['path']) == row
    shutil.copytree(SOURCE, out, ignore=shutil.ignore_patterns('__pycache__'))
    manifest = json.loads((SOURCE/'manifest.json').read_text())
    inputs = hardware_inputs(SOURCE, manifest)
    relative = 'nav/rtl/top/nav_subsystem.v'
    original = (out/relative).read_text()
    old = "    assign i_IMU_RX_DATA  = {iu_valid, 22'd0, iu_first, iu_data};"
    new = "    assign i_IMU_RX_DATA  = iu_valid ? {1'b1, 22'd0, iu_first, iu_data} : 32'd0;"
    assert original.count(old) == 1
    (out/relative).write_text(original.replace(old, new, 1), newline='\n')
    assert len(inputs) == 63
    changed = [n for n in inputs if not same_bytes(SOURCE/n, out/n)]
    assert changed == [relative]
    assert (out/relative).read_text().replace(new, old, 1) == original
    # No initialization, force, model replacement, CPU/FIFO change or clock
    # acceleration: this is a hardware-visible candidate for the MMIO response.
    manifest = relocate(copy.deepcopy(manifest), SOURCE, out)
    manifest.update(invalid_IMU_MMIO_returns_zero=True, source_preparation_only=True,
                    synthesis_complete=False, automatic_pnr=False,
                    programmed=False, deployment_accepted=False)
    save(out/'manifest.json', manifest)
    proof = relocate(copy.deepcopy(original_review), SOURCE, out)
    proof.update(recorded_at=datetime.now().astimezone().isoformat(), source=str(SOURCE), prepared=str(out),
                 hardware_RTL63_all_byte_identical=False, hardware_RTL62_all_byte_identical=True,
                 only_changed_hardware_input=relative, invalid_IMU_read_returns_zero=True,
                 old_expression=old.strip(), new_expression=new.strip(),
                 valid_IMU_payload_pop_timestamp_FIFO_and_firmware_preserved=True,
                 diagnostic=str(PROBE), diagnostic_application_contract_passed=False,
                 cause_scope='Four-state simulation X propagation on empty payload, not a demonstrated silicon CPU defect',
                 actual_CPU250_boot_timeout_and_stop_not_yet_passed=True)
    proof['bindings'] = [metadata(row['path']) for row in proof['bindings']]
    save(out/'prepared_source_review.json', proof)
    return proof


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('--out', type=Path, required=True)
    result = prepare(parser.parse_args().out)
    print(json.dumps(dict(source_prepared=True, RTL_changed=result['only_changed_hardware_input'],
                          CPU250_contract_passed=False)))
