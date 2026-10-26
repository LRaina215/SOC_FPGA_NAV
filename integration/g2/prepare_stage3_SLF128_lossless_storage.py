"""Strict storage composition onto an explicit frozen SLF128/profile base.

No EDA invocation exists. --plan writes JSON only; --prepare writes a new
candidate after all RTL tools have ended. run.tcl always errors: new FW RV build,
ELF/BIN/128KiB lane binding and root review are required before another explicit
entry may be created. The historical256/B preparation is never resumed here.
"""
from pathlib import Path
import argparse,difflib,json,shutil
from prepare_baseline_fusion_retime import hardware_inputs,metadata,relocate
from prepare_stage2b_Bstorage_combined import SOURCE,STORES,WEIGHT,OWNER,TX,CONSOLE,load,save,merge_disjoint,storage_payload


def payload(base128):
    manifest=load(base128/'manifest.json')
    assert manifest.get('SLF_LOG2N')==7,'Only final production SLF128/profile7 is permitted'
    contract=load(base128/'implementation_contract.json')
    assert contract['production_SLF_LOG2N']==7 and contract['SLF_cells']==16384
    # Explicit selected directory and its frozen metadata are required; never
    # silently replace a later parent preparation with an earlier hardcoded one.
    base_review=load(base128/'prepared_source_review.json')
    checked=[]
    for key in ('hardware_bindings','all_FW_bindings'):
        for row in base_review.get(key,[]):
            assert metadata(Path(row['path']))==row,('128 source changed',row)
            checked.append(row)
    assert len(base_review['hardware_bindings'])==63
    for root,keys in ((OWNER,('bindings','validation_bindings')),(WEIGHT,('bindings',)),(TX,('hardware_bindings',)),(CONSOLE,('hardware_bindings',))):
        r=load(root/'prepared_source_review.json')
        for key in keys:
            for row in r[key]:assert metadata(Path(row['path']))==row,('storage source changed',row)
    storage,old_details=storage_payload();final={};details={}
    for rel,text in storage.items():
        parent_text=(base128/rel).read_text(encoding='utf-8')
        lf_snapshot='                    if (cmd_update) begin lf_build_cx <= lf_cx; lf_build_cy <= lf_cy; end\n'
        explicit_snapshot_join=(rel=='nav/rtl/match/slf_match.v')
        if explicit_snapshot_join:
            # Both independent changes insert at old nsnap. Register updates
            # target disjoint lvalues on the same accepted-command edge.
            assert parent_text.count(lf_snapshot)==1
            parent_text=parent_text.replace(lf_snapshot,'',1)
        final[rel],details[rel]=merge_disjoint((SOURCE/rel).read_text(encoding='utf-8'),[
            ('selected SLF128/profile/FW base',parent_text),
            ('proved storage consumer deltas',text)])
        if explicit_snapshot_join:
            owner='                    point_owned <= 1\'b1; wait_release <= 1\'b0;\n'
            assert final[rel].count(owner)==1
            final[rel]=final[rel].replace(owner,lf_snapshot+owner,1)
            details[rel].append(dict(explicit_semantic_join='LF center snapshot plus point ownership at the same original command-admission edge; disjoint register targets, both retained',parent_line=lf_snapshot.strip(),storage_line=owner.strip()))
    rel='nav/rtl/match/slf_match.v'
    old='nav_ram_sdp #(.DW(1), .AW(AW)) u_touch ('
    assert final[rel].count(old)==1
    final[rel]=final[rel].replace(old,'nav_ram_sdp #(.DW(1), .AW(AW), .STAGE2B_BLOCK(1)) u_touch (',1)
    assert 'parameter integer LOG2N  = 7,' in final[rel]
    assert 'S_WAIT_POINTS' in final[rel] and '.STAGE2B_BLOCK(1)) u_slist' in final[rel]
    assert 'begin:g_slf_point_planes' in final['nav/rtl/common/nav_ram_sdp.v']
    assert 'begin:g_stage2b_block' in final['nav/rtl/common/nav_ram_sdp.v']
    assert '.lf_cx(r_MATCH_LF_CX), .lf_cy(r_MATCH_LF_CY)' in final['nav/rtl/top/nav_subsystem.v']
    assert 'nav_ram_sdp #(.DW(32), .AW(TX_WAW)) u_injb (' in final['nav/rtl/net/host_mbx.v']
    safety=base128/'safety_rebase_review.json'
    if safety.exists():
        sr=load(safety)
        assert sr['unchanged_all_FW'] and sr['public_nav_fifo_ob_unchanged']
        assert sr['FIFO_text_equals_proved_candidate3'] and sr['MAC_text_equals_proved_candidate2']
        assert "assign i_IMU_RX_DATA  = iu_valid ? {1'b1, 22'd0, iu_first, iu_data} : 32'd0;" in final['nav/rtl/top/nav_subsystem.v']
        assert (base128/'nav/rtl/common/nav_fifo_ob.v').read_bytes()==(SOURCE/'nav/rtl/common/nav_fifo_ob.v').read_bytes()
    return manifest,contract,base_review,checked,final,details,old_details


def budget():
    # All counts are source expectations derived from actual SYN40 leaf classes;
    # actual wide split/physical placement and <=300 acceptance remain unknown.
    rows=[
        dict(item='actual SYN40 base',source=314,wide=18,physical=332),
        dict(item='packed_weight9',source=-5,wide=0,physical=-5),
        dict(item='point planes',source=0,wide=-8,physical=-8),
        dict(item='SLF structmap64x1024x18 ->16x1024x18',source=-48,wide=0,physical=-48),
        dict(item='touch4x16384x1 ->1x16384x1; explicit class retention',source=-3,wide=0,physical=-3),
        dict(item='bitmap',source=1,wide=0,physical=1),
        dict(item='TXshadow512x32',source=1,wide=1,physical=2),
        dict(item='injection retained distributed',source=0,wide=0,physical=0),
        dict(item='slist1024x14',source=1,wide=0,physical=1),
        dict(item='four trig1024x16',source=4,wide=0,physical=4),
        dict(item='four console1024x8',source=4,wide=0,physical=4),
        dict(item='bias425x26',source=1,wide=1,physical=2),
        dict(item='column scratch not migrated',source=0,wide=0,physical=0),
    ]
    totals={k:sum(r[k]for r in rows)for k in ('source','wide','physical')}
    assert totals==dict(source=270,wide=12,physical=282)
    return dict(prediction_only=True,rows=rows,totals=totals,
        old_A_all_proposals_migration_physical=18,old_B_without_column_migration_physical=16,
        selected_migration_with_injection_retained_physical=14,
        touch_is_existing_BSRAM_class_retention_not_new_optimization=True,
        actual_non_camera_BSRAM_limit=300,camera_reserved_blocks_at_least=36,hard_total_policy=336,
        actual_BSRAM_LUT_FF_CLS='unknown until authorized combined mapping; no LUT/CLS saving credited for injection or ring a second time')


def plan(base128,out):
    assert not out.exists(),out
    m,c,r,checked,final,details,old_details=payload(base128)
    result=dict(plan_only=True,source128=str(base128),SLF_LOG2N=7,profile='production7',
        storage_changed_files=sorted(final),header_classification='C.h plus.vh/.svh; historical SYN40 count31 includes28 C and3 RTL headers',
        source128_hardware_and_FW_metadata_verified=len(checked),merge_details=details,storage_internal_merge=old_details,
        touch_BSRAM_optin_added=True,injection='distributed, GO/ACK/length/overwrite unchanged; no saving credited',
        source_budget=budget(),FW_source=c['selected_FW_source'],
        new_Firmware_image_bound=False,firmware_rebuild_pending=m.get('firmware_rebuild_pending',True),
        SYN_entry_disabled=True,synthesis_started=False,PNR_started=False,
        prerequisite='Use explicit final parent directory when preparing; this plan does not freeze a superseded parent version',
        injection_contract='F:/nav_integration_g2_20261023/stage3_injection_BSRAM_contract_review1/result.json')
    out.parent.mkdir(parents=True,exist_ok=True);save(out,result);return result


def prepare(base128,out):
    import psutil
    assert not out.exists(),out
    tools={'xvlog.exe','xelab.exe','xsim.exe','xsimk.exe','vsim.exe','vsimk.exe','vlog.exe','vopt.exe'}
    live=[p.info for p in psutil.process_iter(['pid','name'])if(p.info['name']or'').lower()in tools]
    assert not live,('RTL/TB must be frozen while tools run',live)
    manifest,contract,base_review,checked,final,details,old_details=payload(base128)
    # All merging and conflict rejection happens before touching the destination.
    shutil.copytree(base128,out,ignore=shutil.ignore_patterns('impl','validation*','__pycache__','*.log','*.patch','prepared_source_review.json','memory_inventory.json','storage_layout.json','synthesis_summary.json','job.json','tool_runtime.json','run.tcl','preflight.tcl','effective_options.tcl'))
    patch=[]
    for rel,text in final.items():
        (out/rel).write_text(text,encoding='utf-8')
        patch.extend(difflib.unified_diff((base128/rel).read_text(encoding='utf-8').splitlines(),text.splitlines(),rel+' SLF128 base',rel+' combined storage',n=3))
    additions=load(WEIGHT/'prepared_source_review.json')['new_initialization_assets'];assert len(additions)==35
    for rel in additions:
        p=out/rel
        if p.exists():assert p.read_bytes()==(WEIGHT/rel).read_bytes()
        else:p.parent.mkdir(parents=True,exist_ok=True);shutil.copyfile(WEIGHT/rel,p)
    manifest=relocate(manifest,base128,out)
    manifest.update(stage='prepared_only',SLF_LOG2N=7,lossless_storage_composed=True,
        final_SLF128_source=str(base128),firmware_rebuild_pending=True,inherited_CPU_INIT_not_new_FW=True,
        new_firmware_RV_ELF_BIN_128KiB_lanes_bound=False,automatic_SYN=False,automatic_PNR=False,synthesis_complete=False,EDA_authorized=False)
    save(out/'manifest.json',manifest)
    old_inputs=hardware_inputs(base128,load(base128/'manifest.json'));new_inputs=hardware_inputs(out,manifest)
    assert set(old_inputs)==set(new_inputs) and len(new_inputs)==63
    changed=sorted(k for k in old_inputs if old_inputs[k].read_bytes()!=new_inputs[k].read_bytes())
    assert changed==sorted(final)
    headers=[];assets=[];fw=[]
    for p in sorted(base128.rglob('*')):
        if not p.is_file() or any(x=='impl'or x.startswith('validation')for x in p.relative_to(base128).parts):continue
        rel=p.relative_to(base128);q=out/rel
        if p.suffix.lower()in{'.h','.vh','.svh'}:
            assert q.exists()and p.read_bytes()==q.read_bytes(),('128 header lost',rel);headers.append(q)
        if p.suffix.lower()in{'.hex','.mem','.bin','.mif'}:
            assert q.exists()and p.read_bytes()==q.read_bytes(),('128 source asset changed',rel);assets.append(q)
        if rel.parts[0]=='fw':assert q.exists()and p.read_bytes()==q.read_bytes(),('128 FW source changed',rel);fw.append(q)
    assert (out/'g2_top.v').read_bytes()==(base128/'g2_top.v').read_bytes()
    safety_preserved={}
    if (base128/'safety_rebase_review.json').exists():
        for rel in ('g2_top.v','nav/rtl/common/nav_async_fifo.v','nav/rtl/net/rgmii_mac100.v'):
            assert (out/rel).read_bytes()==(base128/rel).read_bytes(),('safety source lost',rel)
            safety_preserved[rel]='exact selected safe-base bytes'
        safety_preserved['nav/rtl/top/nav_subsystem.v']='invalid IMU read returns0 expression preserved alongside disjoint storage/point deltas'
    for rel in ('nav/rtl/match/slf_lf.v','nav/rtl/regs/nav_regfile.v'):
        assert (out/rel).read_bytes()==(base128/rel).read_bytes()
    for rel in ('g2.cst','g2.sdc','control4500_core.v','command_lease_guard.v','control/rl_observation_v5.v','control/rl_policy_tick_v5.v','control/ir_remote_control.v','control/oled.v','control/oled_driver.v','control/oled_front.v'):
        assert (SOURCE/rel).read_bytes()==(out/rel).read_bytes(),rel
    (out/'run.tcl').write_text('error {SYN DISABLED: final new FW RV build, ELF/BIN/128KiB lane binding, combined RTL and root review are pending}\n',encoding='utf-8')
    (out/'storage_delta.patch').write_text('\n'.join(patch)+'\n',encoding='utf-8')
    contracts=dict(injection=load(Path('F:/nav_integration_g2_20261023/stage3_injection_BSRAM_contract_review1/result.json')),
        point_ownership=load(OWNER/'ownership_contract.json'),SLF128=contract,
        TXshadow=load(TX/'consumer_contract.json'),console=load(CONSOLE/'consumer_contract.json'),
        touch='DW1/AW14 equals native depth: explicitSTAGE2B_BLOCK1 retains existing BSRAM class, capacity/ports/INIT/read schedule unchanged')
    save(out/'source_contracts.json',contracts)
    unique={str(p):p for p in list(new_inputs.values())+headers+assets+fw+[out/x for x in additions]+[out/x for x in ('g2.cst','g2.sdc','manifest.json','run.tcl','source_contracts.json')]}
    review=dict(prepared=True,final_for_EDA=False,base128=str(base128),SLF_LOG2N=7,source_files=63,
        changed_RTL_relative_to128=changed,unmodified128_headers=len(headers),unmodified128_FW_files=len(fw),
        inherited_assets_byte_identical=len(assets),weight9_INIT_added=35,merge_details=details,
        bindings=[metadata(unique[k])for k in sorted(unique)],source_budget=budget(),
        new_FW_RV_ELF_BIN_128KiB_lanes_bound=False,CPU_INIT='Inherited historical bytes only, not the new128/fairness FW',
        firmware_source=contract['selected_FW_source'],SYN_entry_disabled=True,synthesis_started=False,PNR_started=False,programmed=False,
        blockers=['Final rebuilt profile7/d/ZHI/safety FW RV compile and ELF/BIN/128KiB lanes byte binding','All frozen combined RTL contracts and root source review','Actual primitive CEB/OCE/AD and <=300 noncamera BSRAM/LUT/FF gates after one authorized combined synthesis'])
    review['safety_overlay_preserved']=safety_preserved
    review['changed_RTL_relative_to_SYN40']=sorted(rel for rel in new_inputs if (SOURCE/rel).read_bytes()!=new_inputs[rel].read_bytes())
    save(out/'composition_review.json',review);save(out/'job.json',dict(phase='prepared_SLF128_storage_EDA_disabled',automatic_SYN=False,automatic_PNR=False,new_firmware_bound=False))
    return review


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--base128',type=Path,required=True);p.add_argument('--out',type=Path,required=True);p.add_argument('--prepare',action='store_true');a=p.parse_args()
    r=prepare(a.base128,a.out)if a.prepare else plan(a.base128,a.out)
    print(json.dumps(dict(out=str(a.out),SLF_LOG2N=7,EDA_disabled=True,changed=r.get('changed_RTL_relative_to128',r.get('storage_changed_files')),source_budget=budget()['totals']),ensure_ascii=False))
