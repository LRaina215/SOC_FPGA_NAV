"""Freeze two independent lossless storage candidates from completed SYN40.

Preparation only. No RTL simulator, synthesis, PNR, firmware build or programmer
is invoked. Both implementation changes have old isolated evidence; this rebase
still requires the frozen RTL comparisons and stage3 actual combined mapping.
"""
from pathlib import Path
from datetime import datetime
import argparse, difflib, importlib.util, json, re, shutil
from prepare_baseline_fusion_retime import hardware_inputs, metadata, same_bytes
from prepare_slf_point_planes import MARKER, BRANCH

BASE = Path('F:/nav_integration_g2_20261023')
SOURCE = BASE/'baseline_fusion50_ring_MAX_syn40'
PREP = dict(weight9=BASE/'baseline_fusion50_weight9_prepared1',
            pointplanes=BASE/'baseline_fusion50_pointplanes_prepared1')
HERE = Path(__file__).parent


def load(p): return json.loads(p.read_text(encoding='utf-8-sig'))
def save(p, x): p.write_text(json.dumps(x, ensure_ascii=False, indent=2)+'\n', encoding='utf-8')


def point_transform(text):
    # The old transform additionally required row-banks that SYN40 does not use.
    # Reuse only the exact point branch, preserving every fallback source byte.
    assert text.count(MARKER) == 1 and 'g_slf_point_planes' not in text
    assert 'reg [BW-1:0] selected;' in text
    new = text.replace(MARKER, BRANCH, 1)
    assert new.replace(BRANCH, MARKER, 1) == text
    return new


def selected_banked(text):
    return text.split('begin:g_banked', 1)[1].split('end else', 1)[0]


def weight_words():
    words = [int(x, 16) for x in (SOURCE/'src/mem/weights.mem').read_text().split()]
    assert len(words) == 40704 and all(0 <= x <= 65535 for x in words)
    padded = words+[0]*256
    banks = []
    for i in range(40):
        banks += [int(x,16) for x in (SOURCE/f'src/mem/weights_bank{i:02d}.mem').read_text().split()]
    assert padded == banks
    assert (SOURCE/'control/mem/weights.mem').read_bytes() == (SOURCE/'src/mem/weights.mem').read_bytes()
    return words, padded


def contract(profile, out):
    common = dict(source=str(SOURCE), prepared=str(out), preparation_only=True,
        no_new_RTL_test_started=True, synthesis_started=False, PNR_started=False,
        programmed=False, automatic_PNR=False, production_RTL_modified=False,
        control_4500_weights_training_math_125Hz_unchanged=True,
        clocks='Original clk50 50MHz and rgmii_rxc 100MHz; no clock/reset/CDC change',
        no_SHA_used=True)
    if profile == 'weight9':
        return dict(**common, port='clk,en,addr[15:0] -> signed value[15:0]; no write port',
            read='On rising clk with en=1, addressed word and all selector metadata are captured together; external value available after that same edge, exactly one enabled synchronous read.',
            hold='en=0 holds selectors, sign exception, all bank q and external value; input addr may change.',
            initialization='All 40704 trained 16bit words plus 256 old zero-pad words are exactly reconstructed from35 new files. Original output/selector registers are uninitialized before the first enabled read, and the candidate does not add a defined startup output.',
            read_first='Not applicable: read-only ROM, no write or read/write collision.',
            legal_address_domain=[0,40959], production_weight_address_domain=[0,40703],
            out_of_range='Addresses40960..65535 are outside the old40bank ROM domain; no new defined-value guarantee is introduced. Validation scans every legal word, not arbitrary X states outside that contract.',
            consumer='Unchanged fc_v4_top: en=(state==MAC_REQ); MAC_REQ->MAC_WAIT->MAC_ACC schedule unchanged. Layer bases and extents reach40703, not a new precision/quantization profile.',
            old_proofs=[str(BASE/'weight9_equivalence/result.json'),str(BASE/'weight9_netlist_functional/result.json'),str(BASE/'weight9_synthesis/result.json'),str(BASE/'weight9_synthesis_packed_v2/result.json')],
            reuse='Exact old packed ROM RTL and35 initialization assets are byte-bound to weight9_equivalence. Old40960word/45056vendor-read/3745hold and3frame1275neuron results are historical isolated evidence, not a new SYN40 test or physical timing result.',
            expected_physical_bsram_delta=-5, physical_delta_is_prediction=True,
            pending=['New frozen RTL word/hold and real4500 comparison','Combined actual RAM INIT/decode/select/clock mapping','Combined resource and actual two-place-option routing/setup/hold acceptance'])
    return dict(**common, geometry=dict(words=4096,bits_per_word=36,total_bits=147456),
        port='Single clk; independent we/waddr12/wdata36 and re/raddr12; registered rdata36. No reset port.',
        read='At rising clk with re=1, sample old mem[raddr]; one clock read latency. Selected-bank state changes on the same edge, with no added consumer wait state.',
        hold='re=0 holds selected bank and every per-bank read q, hence public rdata even while writes occur or addresses change.',
        read_first='RTL uses nonblocking writes and a separate enabled read process, so same-address simultaneous read/write returns pre-write data, then the next enabled read sees the new word.',
        initialization='INIT_FILE empty branch only. Every memory bit comes from INIT_VAL (low18/high18); each qr and selector initialize0 exactly as old source. Production INIT_VAL=0; nonzero36bit initialization is also in the prepared test.',
        storage='Old8x512x36 banks -> four groups of1024x(low18,high18), eight1024x18 leaves; full address/data capacity unchanged.',
        caller='slf_extract writes monotonically to pb_waddr=n_pts[11:0] while n_pts<4096; slf_match reads j[11:0] via S_T0/S_T0W/S_T1. No explicit top-level no-overlap gate exists: mt_frame_en does not include !mt_busy, so this review does not assume read/write exclusion.',
        collision_scope='Same-clock read-first is exercised in the prepared RTL test, including all4096 exact collisions. Old vendor-model evidence supports the old branch, but stage3 must independently inspect actual SDP modes; electrical collision behavior is not proven by source alone.',
        old_proofs=[str(BASE/'slf_point_planes_probe1/result.json')],
        reuse='Exact old point-plane BRANCH reused; old source active banked body differs from SYN40 only by syn_maxfan attribute. Old RTL+vendor28682checks/4098collisions/nonzeroINIT and physical16->8 were isolated in a different joint context. Fallback row/ROM branches are not imported.',
        expected_physical_bsram_delta=-8, physical_delta_is_prediction=True,
        pending=['New frozen RTL all-address/read-first/hold/INIT comparison','Combined actual8narrow leaves and zeroINIT; no remaining point wide split','All nonpoint RAM and entire joint resource/clocks/route/timing acceptance'])


def validation_files(profile, out):
    val=out/'validation'; val.mkdir()
    if profile == 'pointplanes':
        for name, root in [('reference',SOURCE),('candidate',out)]:
            text=(root/'nav/rtl/common/nav_ram_sdp.v').read_text()
            assert text.count('module nav_ram_sdp #(')==1
            (val/(name+'.v')).write_text(text.replace('module nav_ram_sdp #(', 'module ram_'+name+' #(',1),encoding='utf8')
        shutil.copyfile(HERE/'tb_stage2b_pointplanes.sv',val/'tb.sv')
        files=['reference.v','candidate.v','tb.sv']; top='tb_stage2b_pointplanes'
    else:
        shutil.copytree(out/'src',val/'src')
        for name, root in [('reference',SOURCE),('candidate',out)]:
            text=(root/'fixed4500_weight_rom.v').read_text()
            if name=='reference':text=text.replace('fixed4500_weight_rom','fixed4500_weight_rom_ref')
            (val/('weight_'+name+'.v')).write_text(text,encoding='utf8')
        core=(SOURCE/'fc_v4_top.v').read_text()
        assert core==(out/'fc_v4_top.v').read_text()
        (val/'fc_reference.v').write_text(core.replace('module fc_v4_top #(','module fc_original #(',1).replace('fixed4500_weight_rom weights(','fixed4500_weight_rom_ref weights(',1),encoding='utf8')
        shutil.copyfile(out/'fc_v4_top.v',val/'fc_candidate.v')
        shutil.copyfile(out/'control/fc_v4_round_sat.v',val/'fc_v4_round_sat.v')
        _, words=weight_words(); (val/'weights_expected.mem').write_text('\n'.join(f'{x:04x}'for x in words)+'\n')
        import random
        rng=random.Random(4500); vectors=[0]*168+[rng.randint(-8000,8000)for _ in range(168)]+[rng.randint(-409600,409600)for _ in range(168)]
        (val/'vectors.mem').write_text('\n'.join(f'{x&0xffffff:06x}'for x in vectors)+'\n')
        base=(HERE/'tb_storage_equivalence.sv').read_text()
        base=base[base.index('module tb_storage_equivalence;'):]
        start,end=base.index(' reg ram_we=0'),base.index(' reg [23:0] vectors')
        base=base[:start]+(HERE/'tb_stage2b_weight9_scan.svh').read_text()+base[end:]
        base=base.replace('if(cycles>600000)', 'if(cycles>600000)')
        base=base.replace('   if({b0,d0,e0,s0,id0,c0,v0}', '   if($isunknown({b0,d0,e0,s0,id0,c0,v0,b1,d1,e1,s1,id1,c1,v1}))$fatal(1,"unknown core status");\n   if({b0,d0,e0,s0,id0,c0,v0}')
        base=base.replace('     if({l0,n0,z0}', '     if($isunknown({l0,n0,z0,l1,n1,z1}))$fatal(1,"unknown neuron");\n     if({l0,n0,z0}')
        base=base.replace('if(d0)begin if(a0!==a1)', 'if(d0)begin if($isunknown({a0,a1}))$fatal(1,"unknown action");if(a0!==a1)')
        base=base.replace('5 RAM geometries x196608 cycles','WEIGHT9 all40960 enabled reads and holds; real4500 no substitute')
        (val/'tb.sv').write_text('`timescale 1ns/1ps\n'+base,encoding='utf8')
        files=['weight_reference.v','weight_candidate.v','fc_reference.v','fc_candidate.v','fc_v4_round_sat.v','tb.sv'];top='tb_storage_equivalence'
    spec=dict(profile=profile,working_directory=str(val),files=files,top=top,minimum_physical_bytes=2*1024**3,minimum_commit_bytes=4*1024**3,
              memory_gate_checked_only_when_explicitly_run=True,RTL_simulation_started=False,
              entry=['D:/Anaconda/python.exe','-B','-X','utf8',str(HERE/'validate_stage2b_weight_point.py'),'--candidate',str(out),'--out','<new-independent-F-directory>','--run'],
              EDA_or_simulation_launched_during_preparation=False)
    save(val/'validation_request.json',spec);return spec


def prepare(profile):
    out=PREP[profile];assert not out.exists(),out
    source_m=load(SOURCE/'manifest.json');a=hardware_inputs(SOURCE,source_m);assert len(a)==63
    assert load(SOURCE/'job.json')['phase']=='synthesis_complete'
    shutil.copytree(SOURCE,out,ignore=shutil.ignore_patterns('impl','__pycache__','job.json','tool_runtime.json','*.log','system_telemetry.jsonl'))
    if profile=='pointplanes':
        rel='nav/rtl/common/nav_ram_sdp.v';old=(SOURCE/rel).read_text();new=point_transform(old);(out/rel).write_text(new,encoding='utf8')
        old_probe=(BASE/'slf_point_planes_probe1/old.v').read_text()
        assert selected_banked(old_probe).replace(' /* synthesis syn_maxfan=128 */','')==selected_banked(old)
        old_new=(BASE/'slf_point_planes_probe1/new.v').read_text()
        assert BRANCH.split('begin:g_slf_point_planes',1)[1].split('end else if',1)[0] in old_new
        additions=[]; numeric=None
    else:
        rel='fixed4500_weight_rom.v';old=(SOURCE/rel).read_text()
        spec=importlib.util.spec_from_file_location('stage2b_weight9_generator',HERE.parents[1]/'control/tools/g2_weight_rom9.py');gen=importlib.util.module_from_spec(spec);spec.loader.exec_module(gen)
        words,_=weight_words();signed=[x-65536 if x&32768 else x for x in words]
        numeric=gen.generate(out,signed);new=(out/rel).read_text()
        assert new==(BASE/'weight9_equivalence/fixed4500_weight_rom.v').read_text()
        additions=sorted(str(p.relative_to(out)).replace('\\','/')for p in(out/'src/mem').glob('weight9*.mem'))
        assert len(additions)==35
        for n in additions:assert same_bytes(out/n,BASE/'weight9_equivalence'/n),n
    (out/'storage_candidate.patch').write_text(''.join(difflib.unified_diff(old.splitlines(True),new.splitlines(True),fromfile='SYN40/'+rel,tofile=profile+'/'+rel)),encoding='utf8')
    m=source_m.copy();m['source_files']=[str(out/Path(p).relative_to(SOURCE))for p in source_m['source_files']]
    m.update(candidate='stage2b_'+profile,source_preparation_only=True,frozen_source_build=str(SOURCE),automatic_PNR=False,automatic_pnr=False,synthesis_complete=False,programmed=False,modified_rtl_inputs=[rel],stage3_combined_synthesis_only=True)
    save(out/'manifest.json',m)
    t=(SOURCE/'run.tcl').read_text().replace(str(SOURCE),str(out)).replace(SOURCE.as_posix(),out.as_posix())
    (out/'run.tcl').unlink();(out/'run.tcl.disabled_stage3_only').write_text(t,encoding='utf8')
    assert set(re.findall(r'^add_file -type verilog \{(.+)\}$',t,re.M))=={p.as_posix()for p in hardware_inputs(out,m).values()}
    changed=sorted(n for n,p in a.items()if not same_bytes(p,out/n));assert changed==[rel],changed
    original_assets=[p.relative_to(SOURCE)for p in SOURCE.rglob('*')if p.is_file()and'impl'not in p.relative_to(SOURCE).parts and p.suffix.lower()in{'.hex','.mem','.mif','.bin'}];assert len(original_assets)==69
    assert all(same_bytes(SOURCE/n,out/n)for n in original_assets)
    headers=[p.relative_to(SOURCE)for p in SOURCE.rglob('*')if p.is_file()and'impl'not in p.relative_to(SOURCE).parts and p.suffix.lower()in{'.h','.vh'}];assert len(headers)==31 and all(same_bytes(SOURCE/n,out/n)for n in headers)
    fw=[p.relative_to(SOURCE)for p in(SOURCE/'fw').rglob('*')if p.is_file()and'__pycache__'not in p.parts];assert all(same_bytes(SOURCE/n,out/n)for n in fw)
    for n in('g2.cst','g2.sdc','g2_top.v','control4500_core.v','control/mem_policy.v','control/rl_policy_tick_v5.v','control/policy_action_map.v'):
        assert same_bytes(SOURCE/n,out/n),n
    v=validation_files(profile,out);c=contract(profile,out);save(out/'storage_contract.json',c)
    validation_bindings=[metadata(p)for p in sorted((out/'validation').rglob('*'))if p.is_file()]
    r=dict(prepared=True,profile=profile,recorded_at=datetime.now().astimezone().isoformat(),source=str(SOURCE),prepared_directory=str(out),selected_RTL=63,changed_RTL=changed,other62_RTL_byte_identical=True,
           original69_assets_byte_identical=True,headers31_byte_identical=True,firmware_all_byte_identical=True,new_initialization_assets=additions,numeric_reconstruction=numeric,
           explicit_CST_SDC_top_clocks_control_stop_125Hz_source_unchanged=True,run_tcl_disabled=True,contract_file=str(out/'storage_contract.json'),validation_request=str(out/'validation/validation_request.json'),
           validation_not_run=True,synthesis_not_run=True,PNR_not_run=True,programmed=False,automatic_PNR=False,
           validation_bindings=validation_bindings,
           bindings=[metadata(out/n)for n in sorted(set(a)|set(additions)|{str(n).replace('\\','/')for n in original_assets+headers+fw})])
    save(out/'prepared_source_review.json',r);return r


if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--profile',choices=list(PREP),required=True);p.add_argument('--prepare',action='store_true');a=p.parse_args();assert a.prepare,'Explicit preparation only'
    x=prepare(a.profile);print(json.dumps({k:x[k]for k in ['prepared','profile','prepared_directory','changed_RTL','new_initialization_assets','validation_not_run','synthesis_not_run','PNR_not_run']},ensure_ascii=False))
