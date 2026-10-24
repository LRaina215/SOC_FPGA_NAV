"""Actual full frozen NAV subsystem MMIO/initialization/fault composition.

Original networks, regfile, safety, coverage and datapaths remain instantiated.
No full G2 synthesis, PNR, external PHY, CPU program or actuator test.
"""
import argparse
from datetime import datetime
import json
from pathlib import Path
import re
import shutil
import subprocess
from derive_ddr_matcher import HERE,derive,derive_store,derive_row_engine,derive_fast_cache
from derive_ddr_elevation import derive as derive_em,derive_map,derive_traversability
from derive_ddr_shared import derive_store as share_store,derive_cache,derive_matcher
from derive_ddr_full_subsystem import subsystem,matcher,elevation,map_top
FROZEN=Path('F:/nav_integration_g2_20261023/sfp_lane2_private_snapshot_syn11_retry1')


def run(out,interleaved=False):
    assert not out.exists();out.mkdir()
    manifest=json.loads((FROZEN/'manifest.json').read_text());nav=FROZEN/'nav'
    names=[]
    for item in manifest['source_files']:
        p=Path(item)
        if not p.is_relative_to(nav):continue
        # Old modules are harmless references. The derived full NAV and its
        # map instances below have distinct names; they are the active DUT.
        assert p.suffix=='.v' and not (out/p.name).exists(),p
        shutil.copyfile(p,out/p.name)
        # nav_regs_map.vh has an include guard around module-local params.
        # Do not compile both the unused old top and its replacement in the
        # same compilation unit: the old include would hide those params.
        if p.name!='nav_subsystem.v':names.append(p.name)
    for p in nav.rglob('*.vh'):shutil.copyfile(p,out/p.name)
    shutil.copyfile(nav/'rtl/plan/sin_lut.hex',out/'sin_lut.hex')
    for i in range(4):shutil.copyfile(FROZEN/f'ram_b{i}.hex',out/f'ram_b{i}.hex')
    for name in ['nav_ddr_native.v','nav_ddr_row_stream.v','nav_ddr_dual_step.v','nav_ddr_health.v','nav_ddr_admission.v']:
        shutil.copyfile(HERE/name,out/name);names.append(name)
    def text(rel):return (nav/rel).read_text(encoding='utf-8-sig')
    cache_derivation=derive_fast_cache
    if interleaved:
        from interleave_struct_cache import derive as cache_derivation
    generated={
        'nav_subsystem_ddr_full.v':subsystem(text('rtl/top/nav_subsystem.v')),
        'slf_match_ddr_full.v':matcher(derive_matcher(derive(text('rtl/match/slf_match.v')))),
        'elev_map_ddr.v':elevation(derive_em(text('rtl/map/elev_map.v'))),
        'map_subsys_ddr.v':map_top(derive_map(text('rtl/map/map_subsys.v'))),
        'trav_v3_ce.v':derive_traversability(text('rtl/map/trav_v3.v')),
        'nav_ddr_structure_store_full.v':share_store(derive_store((HERE/'nav_ddr_structure_store.v').read_text())),
        'nav_slf_row_update_full.v':derive_row_engine((HERE/'nav_slf_row_update.v').read_text()),
        'nav_ddr_struct_cache_fast.v':cache_derivation((HERE/'nav_ddr_struct_cache.v').read_text()),
        'nav_ddr_em_cache.v':derive_cache((HERE/'nav_ddr_map_cache.v').read_text())}
    for n,s in generated.items():(out/n).write_text(s,encoding='utf8');names.append(n)
    row=Path('F:/nav_integration_g2_20261023/ddr_map_row_update_probe7');assert json.loads((row/'functional_result.json').read_text())['passed']
    shutil.copyfile(row/'slf_lf_ce.v',out/'slf_lf_ce.v');names.append('slf_lf_ce.v')
    model=(HERE/'tb_slf_row_update.sv').read_text().split('module tb_slf_row_update;')[0]
    for a,b in [('[0:8191]','[0:16383]'),('[12:0]held_addr','[13:0]held_addr'),('addr[27:16]','addr[27:17]'),('addr[15:3]','addr[16:3]')]:
        assert model.count(a)==1;model=model.replace(a,b)
    (out/'row_service_model.sv').write_text(model)
    shutil.copyfile(HERE/'tb_ddr_full_subsystem.sv',out/'tb_ddr_full_subsystem.sv')
    commands=[('xvlog',names),('xvlog',['-sv','row_service_model.sv','tb_ddr_full_subsystem.sv']),
        ('xelab',['-debug','off','-timescale','1ns/1ps','tb_ddr_full_subsystem','-s','snap']),('xsim',['snap','-runall'])]
    for i,(tool,args) in enumerate(commands):
        with (out/f'{i}_{tool}_stdout.log').open('w') as log:
            subprocess.run(['cmd','/c',str(Path('E:/2026.1/Vivado/bin')/(tool+'.bat')),*args],cwd=out,
                stdout=log,stderr=subprocess.STDOUT,timeout=600,check=True,creationflags=subprocess.CREATE_NO_WINDOW)
    s=(out/'3_xsim_stdout.log').read_text(errors='replace')
    assert 'TEST PASSED' in s and not re.search(r'FATAL|Fatal|ERROR:',s),s[-3500:]
    result=dict(tested_at=datetime.now().astimezone().isoformat(),passed=True,
        metrics={k:int(v) for k,v in re.findall(r'(\w+)=(\d+)',re.search(r'TEST PASSED[^\r\n]*',s)[0])},
        cold=[{k:float(v) if '.' in v else int(v) for k,v in re.findall(r'(\w+)=([\d.]+)',l)} for l in s.splitlines() if l.startswith('SUBSYSTEM_COLD')],
        mode=[{k:int(v) for k,v in re.findall(r'(\w+)=(\d+)',l)} for l in s.splitlines() if l.startswith('SUBSYSTEM_MODE')],
        scope='complete original NAV subsystem with shared DDR map adapters and modeled service; scripted MMIO master',
        CPU_HOLD=[{k:int(v) for k,v in re.findall(r'(\w+)=(\d+)',l)} for l in s.splitlines() if l.startswith('CPU_HOLD_PROOF')],
        actual_CPU_firmware_tested=False,actual_SFP_data_tested=False,actual_DDR_service_bound_verified=False,
        interleaved_cache_banks=interleaved,
        full_joint_control_actuator_path_verified=False,full_joint_implementation=False,programmed=False)
    (out/'result.json').write_text(json.dumps(result,indent=2));print(json.dumps(result,indent=2))


if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--out',type=Path,required=True)
    p.add_argument('--interleaved-cache',action='store_true')
    a=p.parse_args();run(a.out.resolve(),a.interleaved_cache)
