"""Freeze B storage composition only. There is deliberately no EDA run option.

Run preparation only after every RTL simulator has ended. Sources are merged
against their exact frozen bases; overlapping edits fail instead of overwriting.
The resulting synthesis recipe is text with an explicit disabled entry.
"""
from pathlib import Path
import argparse,difflib,json,re,shutil
from prepare_baseline_fusion_retime import hardware_inputs,metadata,relocate
from prepare_stage2b_weight_point import point_transform

BASE=Path('F:/nav_integration_g2_20261023')
SOURCE=BASE/'baseline_fusion50_ring_MAX_syn40'
STORES=BASE/'stage2B_bitmap_mailbox_slist_trig_console_bias_prepared1'
WEIGHT=BASE/'baseline_fusion50_weight9_prepared1'
OWNER=BASE/'stage2B_point_ownership_prepared2'
TX=BASE/'stage2B_TXshadow_RDEN_prepared2'
CONSOLE=BASE/'stage2B_console_posted_prepared1'


def load(p):return json.loads(p.read_text(encoding='utf-8-sig'))
def save(p,x):p.write_text(json.dumps(x,ensure_ascii=False,indent=2)+'\n',encoding='utf-8')


def merge_disjoint(base,variants):
    """Merge independent exact-base line deltas; duplicate identical edits dedup."""
    lines=base.splitlines(keepends=True);edits=[]
    for name,text in variants:
        new=text.splitlines(keepends=True)
        for tag,i,j,k,m in difflib.SequenceMatcher(None,lines,new,autojunk=False).get_opcodes():
            if tag=='equal':continue
            change=(i,j,tuple(new[k:m]))
            for a,b,repl,owners in edits:
                same=(i,j,change[2])==(a,b,repl)
                conflict=(max(i,a)<min(j,b)) or (i==j and a<=i<b) or (a==b and i<=a<j) or (i==j==a==b)
                assert same or not conflict,('overlapping source edits',name,owners,(i,j),(a,b))
            match=next((e for e in edits if (i,j,change[2])==e[:3]),None)
            if match:match[3].append(name)
            else:edits.append([*change,[name]])
    result=lines[:]
    for i,j,repl,_ in sorted(edits,key=lambda x:(x[0],x[1]),reverse=True):result[i:j]=repl
    return ''.join(result),[dict(old_first_line=i+1,old_stop_line=j,new_lines=len(r),contributors=o)for i,j,r,o in edits]


def prepare(out):
    # This explicit environmental fence is read-only. It is not an EDA launch.
    import psutil
    names={'xvlog.exe','xelab.exe','xsim.exe','xsimk.exe','vsim.exe','vsimk.exe','vlog.exe','vopt.exe'}
    live=[]
    for p in psutil.process_iter(['pid','name','cmdline']):
        if (p.info['name']or'').lower() in names:live.append(p.info)
    assert not live,('RTL tools still active: do not write candidate RTL',live)
    assert not out.exists(),out
    manifest=load(SOURCE/'manifest.json');original=hardware_inputs(SOURCE,manifest);assert len(original)==63
    for root in (SOURCE,STORES,WEIGHT,OWNER,TX,CONSOLE):assert root.is_dir(),root
    # Parent candidates have already disabled injection promotion. Require their
    # exact host deltas agree, rather than choosing whichever is copied last.
    host='nav/rtl/net/host_mbx.v'
    assert (TX/host).read_bytes()==(CONSOLE/host).read_bytes(),'parent host changes differ'
    host_text=(TX/host).read_text(encoding='utf-8')
    assert 'nav_ram_sdp #(.DW(32), .AW(TX_WAW)) u_injb (' in host_text
    assert 'nav_ram_sdp #(.DW(32), .AW(TX_WAW), .STAGE2B_BLOCK(1)) u_txs (' in host_text
    # Source provenance is checked before any directory is created.
    owner_review=load(OWNER/'prepared_source_review.json')
    for row in owner_review['bindings']+owner_review['validation_bindings']:assert metadata(Path(row['path']))==row,row
    weight_review=load(WEIGHT/'prepared_source_review.json')
    for row in weight_review['bindings']:assert metadata(Path(row['path']))==row,row
    for root in (TX,CONSOLE):
        for row in load(root/'prepared_source_review.json')['hardware_bindings']:
            assert metadata(Path(row['path']))==row,row
    assert len(weight_review['new_initialization_assets'])==35
    common='nav/rtl/common/nav_ram_sdp.v';match='nav/rtl/match/slf_match.v';top='nav/rtl/top/nav_subsystem.v'
    merged={};merge_details={}
    stage_common=(STORES/common).read_text(encoding='utf-8')
    merged[common]=point_transform(stage_common)
    assert merged[common].count('begin:g_slf_point_planes')==1
    assert merged[common].count('begin:g_stage2b_block')==1 and 'parameter integer STAGE2B_BLOCK=0' in merged[common]
    for rel,variants in (
        (match,[('point ownership',(OWNER/match).read_text(encoding='utf-8')),('slist/trig block opt-in',(STORES/match).read_text(encoding='utf-8'))]),
        (top,[('point raw ownership',(OWNER/top).read_text(encoding='utf-8')),('TXshadow read enable',(TX/top).read_text(encoding='utf-8'))]),
    ):
        merged[rel],merge_details[rel]=merge_disjoint((SOURCE/rel).read_text(encoding='utf-8'),variants)
    merged[host]=host_text
    merged['nav/rtl/lidar/xt_frame_rx.v']=(OWNER/'nav/rtl/lidar/xt_frame_rx.v').read_text(encoding='utf-8')
    merged['nav/rtl/soc/nav_soc.v']=(CONSOLE/'nav/rtl/soc/nav_soc.v').read_text(encoding='utf-8')
    merged['fixed4500_weight_rom.v']=(WEIGHT/'fixed4500_weight_rom.v').read_text(encoding='utf-8')
    for rel in ('nav/rtl/map/trav_v3.v','nav/rtl/plan/traj_eval.v','fc_v4_top.v'):
        merged[rel]=(STORES/rel).read_text(encoding='utf-8')
    expected=sorted(merged)
    # Copy untouched frozen SYN40 base first, then apply each reviewed final body
    # once. Old run Tcl is never left as an executable composition entry.
    shutil.copytree(SOURCE,out,ignore=shutil.ignore_patterns('impl','__pycache__','validation','job.json','tool_runtime.json','*.log','*.patch','*_review*.json','prepared_source_review.json','memory_inventory.json','storage_layout.json','synthesis_summary.json','system_telemetry.jsonl','run.tcl','preflight.tcl','effective_options.tcl'))
    patches=[]
    for rel,text in merged.items():
        (out/rel).write_text(text,encoding='utf-8')
        patches.extend(difflib.unified_diff((SOURCE/rel).read_text(encoding='utf-8').splitlines(),text.splitlines(),rel+' SYN40',rel+' Bstorage',n=3))
    for rel in weight_review['new_initialization_assets']:
        assert not (out/rel).exists(),rel
        (out/rel).parent.mkdir(parents=True,exist_ok=True);shutil.copyfile(WEIGHT/rel,out/rel)
    manifest=relocate(manifest,SOURCE,out)
    manifest.update(stage='prepared_only',storage_B_composition=True,EDA_authorized=False,automatic_synthesis=False,automatic_PNR=False,
        composition_sources=[str(x)for x in (SOURCE,STORES,WEIGHT,OWNER,TX,CONSOLE)],firmware_profile='Exact SYN40 frozen firmware; fairness/safety FW pending separate decision')
    save(out/'manifest.json',manifest)
    final=hardware_inputs(out,manifest);assert set(final)==set(original)
    changed=sorted(rel for rel in original if original[rel].read_bytes()!=final[rel].read_bytes())
    assert changed==expected,(changed,expected)
    assets=[];headers=[];firmware=[]
    for old in sorted(SOURCE.rglob('*')):
        if not old.is_file() or 'impl' in old.relative_to(SOURCE).parts:continue
        rel=old.relative_to(SOURCE);p=out/rel
        if old.suffix.lower()in{'.hex','.mem','.bin','.mif'}:
            assert p.exists() and old.read_bytes()==p.read_bytes(),('old asset altered',rel)
            assets.append(str(rel))
        if old.suffix.lower()in{'.vh','.svh'}:
            assert p.exists() and old.read_bytes()==p.read_bytes(),('header altered',rel);headers.append(str(rel))
        if rel.parts[0]=='fw':
            assert p.exists() and old.read_bytes()==p.read_bytes(),('FW altered',rel);firmware.append(str(rel))
    assert len(assets)==69 and len(headers)==31,(len(assets),len(headers))
    # Source memory/pin/clock/CPU architecture contracts stay explicit.
    for rel in ('g2_top.v','g2.cst','g2.sdc','control4500_core.v','command_lease_guard.v',
                'control/rl_observation_v5.v','control/rl_policy_tick_v5.v',
                'control/ir_remote_control.v','control/oled.v','control/oled_driver.v','control/oled_front.v'):
        assert (SOURCE/rel).read_bytes()==(out/rel).read_bytes(),rel
    disabled='''# PREPARATION ONLY. This file must not launch EDA.
error {B storage candidate is not authorized for synthesis or PNR; root review and combined RTL validation are pending}
'''
    (out/'run.tcl').write_text(disabled,encoding='utf-8')
    for name in ('run.tcl','preflight.tcl'):
        text=(SOURCE/name).read_text(encoding='utf-8')
        text=text.replace(SOURCE.as_posix(),out.as_posix()).replace(str(SOURCE),str(out))
        assert str(SOURCE)not in text and SOURCE.as_posix()not in text
        (out/(name+'.disabled.txt')).write_text(text,encoding='utf-8')
    (out/'combined_delta.patch').write_text('\n'.join(patches)+'\n',encoding='utf-8')
    policy=dict(source_prediction_only=True,physical_baseline=332,weight9=-5,pointplanes=-8,
        bitmap=1,TXshadow=2,injection=0,slist=1,trig_four=4,console_four=4,bias=2,column_scratch=0,
        predicted_source_BSRAM=321,predicted_wide_split_additions=12,predicted_physical_BSRAM=333,
        mainline_target_at_most=335,hard_policy_at_most=336,physical_device_blocks=340,
        actual_LUT_or_CLS_recovery='unknown until one authorized combined synthesis/physical implementation',
        no_ring_gain_double_count=True,no_injection_promotion=True)
    result=dict(prepared=True,synthesis_started=False,PNR_started=False,programmed=False,automatic_run_disabled=True,
        out=str(out),source=str(SOURCE),RTL_count=63,changed_RTL=changed,merge_details=merge_details,
        original_assets_byte_identical=len(assets),new_weight_INIT_assets=weight_review['new_initialization_assets'],
        headers_byte_identical=len(headers),FW_files_byte_identical=len(firmware),
        safe_fairness_firmware='Not merged; original SYN40 images/source preserved, separate authorization and physical/Pico contracts pending',
        geometry_and_timing=['Full4096x36 point buffer','Original50MHz/100MHz roots and original SDC52','Original128KiB CPU geometry','Original4500 weights numeric values/ring/IR/OLED'],
        policy_budget=policy,
        blockers=['Parent TXshadow/console final RTL evidence must be read and accepted','Root composition review','Combined RTL contracts including CEB/AD qualification strategy','Only then explicit stage3 synthesis; no current launcher'],
        physical_CE_and_collision='Unknown until actual mapped primitives; OCE-only hold is not proof of CEB disabled',
        bindings=[metadata(final[rel])for rel in sorted(final)]+[metadata(out/rel)for rel in assets]+[metadata(out/rel)for rel in weight_review['new_initialization_assets']]+[metadata(out/rel)for rel in headers])
    save(out/'composition_review.json',result)
    save(out/'job.json',dict(phase='prepared_EDA_disabled',synthesis_started=False,PNR_started=False,programmed=False,root_review_required=True))
    print(json.dumps(dict(out=str(out),changed_RTL=changed,new_INIT=35,BSRAM_prediction=333,EDA_disabled=True),ensure_ascii=False))
    return result


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--out',type=Path,required=True);p.add_argument('--prepare',action='store_true');a=p.parse_args()
    if a.prepare:prepare(a.out)
    else:print(json.dumps(dict(preparation_not_executed=True,out=str(a.out),EDA_run_option=False,wait_for_RTL_tools_to_end=True),ensure_ascii=False))
