"""Adopt the proved SFP1 Lane2 IP into an otherwise frozen joint candidate.

This imports no standalone reset, timing exceptions, echo logic or actuator
logic. The protected MAC is required to be byte-identical to the old source.
"""
import json
from pathlib import Path
import shutil
import tomllib


def lane2_sdc(original):
    assert original.count('/LANE0_PCS_RX_O_FABRIC_CLK') == 1
    assert original.count('/LANE0_PCS_TX_O_FABRIC_CLK') == 1
    assert 'set_false_path' not in original and 'set_clock_groups' not in original
    return original.replace('/LANE0_PCS_RX_O_FABRIC_CLK', '/LANE2_PCS_RX_O_FABRIC_CLK').replace(
        '/LANE0_PCS_TX_O_FABRIC_CLK', '/LANE2_PCS_TX_O_FABRIC_CLK')


def check_proof(proof):
    proof = Path(proof)
    result = json.loads((proof/'ip/generation_result.json').read_text())
    inspection = json.loads((proof/'inspection.json').read_text())
    assert result['generated'] and result['lane'] == 'Q0 Lane2'
    assert result['device'] == 'GW5AT-LV138PG484AC1/I0' and result['version'] == 'C'
    assert result['refclock'] == 'Q0 REFCLK1 125MHz' and result['loopback'] == 'OFF'
    assert inspection['constrained_timing_passed'] and inspection['serdes_lane'] == 'Q0 Lane2'
    assert inspection['csr_passed_to_pnr'] and inspection['resources']['GTR12_QUAD'] == 1
    config = tomllib.loads((proof/'ip/serdes_tmp.toml').read_text())
    assert [f'q{q}.ln{n}' for q in range(2) for n in range(4)
            if config[f'q{q}'][f'ln{n}']['enable']] == ['q0.ln2']
    assert config['q0']['ref_pad1_freq'] == '125M'
    assert config['q0']['ln2']['loopBack'] == 'OFF'
    return result, inspection


def adopt(out, original, proof):
    out, original, proof = map(Path, (out, original, proof))
    check_proof(proof)
    assert (original/'ip/giga_serial_ethernet.v').read_bytes() == (proof/'ip/giga_serial_ethernet.v').read_bytes(), 'MAC implementation changed'
    # Only a mapping comment changes in this outer MAC wrapper.
    old = (original/'sfp_host_mac.v').read_text()
    new = (proof/'sfp_host_mac.v').read_text()
    assert old.splitlines()[1:] == new.splitlines()[1:]
    for name in ('serdes.v', 'serdes.csr', 'serdes_tmp.toml', 'serdes.ipc', 'serdes.mod', 'generation_result.json'):
        shutil.copyfile(proof/'ip'/name, out/'ip'/name)
    shutil.copyfile(proof/'sfp_host_mac.v', out/'sfp_host_mac.v')
    (out/'g2.sdc').write_text(lane2_sdc((original/'g2.sdc').read_text()), encoding='utf-8', newline='\n')
    return ['sfp_host_mac.v', 'ip/serdes.v']


def verify_frozen(out, original, proof):
    out, original, proof = map(Path, (out, original, proof))
    check_proof(proof)
    for name in ('serdes.v', 'serdes.csr', 'serdes_tmp.toml', 'serdes.ipc', 'serdes.mod', 'giga_serial_ethernet.v'):
        assert (out/'ip'/name).read_bytes() == (proof/'ip'/name).read_bytes(), ('Lane2 input mismatch', name)
    assert (out/'sfp_host_mac.v').read_bytes() == (proof/'sfp_host_mac.v').read_bytes()
    assert (out/'g2.cst').read_bytes() == (original/'g2.cst').read_bytes(), 'IO mapping unexpectedly changed'
    assert (out/'g2.sdc').read_text() == lane2_sdc((original/'g2.sdc').read_text()), 'joint clocks/exceptions changed beyond Lane2 pins'
    return dict(lane='Q0 Lane2', proof_build=str(proof.resolve()), exact_generated_csr_and_wrapper=True,
                protected_mac_unchanged=True, outer_mac_function_unchanged=True,
                joint_clock_periods_unchanged=True, standalone_exceptions_not_imported=True,
                io_cst_unchanged=True, joint_hardware_accepted=False)


def verify_synthesized_clock_pins(cells):
    gtr = cells['u_sfp/u_mac/u_serdes/gtr12_quad_inst0']
    assert gtr['kind'] == 'GTR12_QUAD'
    actual = {}
    for direction, expected in (('RX', 'h_rx_clk'), ('TX', 'h_tx_clk')):
        port = f'LANE2_PCS_{direction}_O_FABRIC_CLK'
        assert gtr['ports'].get(port) == [expected], (port, gtr['ports'].get(port))
        assert gtr['ports'].get(f'LANE0_PCS_{direction}_O_FABRIC_CLK') != [expected]
        actual[port] = expected
    return actual
