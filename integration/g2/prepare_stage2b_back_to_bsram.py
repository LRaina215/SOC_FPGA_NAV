"""Freeze the six requested stage2B storage migrations; never run tools.

The root reviewer owns packed_weight9 and point planes. This transform adds a
late, explicitly selected small-store branch and composes with their original
banked/point-plane branches. CLI only copies/freezes sources and RTL fixtures.
"""
import argparse
from datetime import datetime
import json
from pathlib import Path
import shutil
from prepare_bias_block_rom import transform as bias_transform

BASE=Path('F:/nav_integration_g2_20261023')
SOURCE=BASE/'baseline_fusion50_ring_MAX_syn40'
DEFAULT_OUT=BASE/'stage2B_bitmap_mailbox_slist_trig_console_bias_prepared1'
RAM='nav/rtl/common/nav_ram_sdp.v'
CHANGED={RAM,'nav/rtl/map/trav_v3.v','nav/rtl/net/host_mbx.v','nav/rtl/match/slf_match.v',
         'nav/rtl/plan/traj_eval.v','nav/rtl/soc/nav_soc.v','fc_v4_top.v'}

def save(p,obj):
    assert not p.exists(),('Do not overwrite frozen evidence',str(p))
    p.write_text(json.dumps(obj,ensure_ascii=False,indent=2)+'\n',encoding='utf-8')
def meta(p):
    s=p.stat();return dict(path=str(p),bytes=s.st_size,mtime_ns=s.st_mtime_ns)
def replace_once(text,old,new):
    assert text.count(old)==1,(old,text.count(old));return text.replace(old,new,1)

def transform_ram(text):
    """Default0 preserves every unselected call; selected procedural body exact."""
    header='parameter INIT_FILE="",parameter [DW-1:0] INIT_VAL={DW{1\'b0}}'
    marker=' end else begin:g_distributed\n'
    assert text.count(marker)==1 and 'STAGE2B_BLOCK' not in text
    body=text.split(marker,1)[1].rsplit(' end endgenerate',1)[0]
    old='(* syn_ramstyle="distributed_ram",syn_romstyle="distributed_rom" *)'
    new='(* syn_ramstyle="block_ram",syn_romstyle="block_rom" *)'
    assert body.count(old)==1
    branch=' end else if(STAGE2B_BLOCK)begin:g_stage2b_block\n'+body.replace(old,new,1)
    result=replace_once(text,header,header+',parameter integer STAGE2B_BLOCK=0')
    result=replace_once(result,marker,branch+marker)
    # This is a precise RTL-body identity, not a mapped-memory signoff.
    assert result.replace(branch,'',1).replace(',parameter integer STAGE2B_BLOCK=0','',1)==text
    assert branch.split('\n',1)[1].replace(new,old,1)==body
    return result

def transform_sources(texts):
    result=dict(texts);result[RAM]=transform_ram(result[RAM])
    selections={
      'nav/rtl/map/trav_v3.v':[('nav_ram_sdp #(.DW(1), .AW(AW)) u_bm','nav_ram_sdp #(.DW(1), .AW(AW), .STAGE2B_BLOCK(1)) u_bm')],
      'nav/rtl/net/host_mbx.v':[(f'nav_ram_sdp #(.DW(32), .AW(TX_WAW)) {name}',f'nav_ram_sdp #(.DW(32), .AW(TX_WAW), .STAGE2B_BLOCK(1)) {name}') for name in ('u_injb','u_txs')],
      'nav/rtl/match/slf_match.v':[('nav_ram_sdp #(.DW(AW), .AW(10)) u_slist','nav_ram_sdp #(.DW(AW), .AW(10), .STAGE2B_BLOCK(1)) u_slist')],
      'nav/rtl/soc/nav_soc.v':[('nav_ram_sdp #(.DW(8), .AW(10)) u_con','nav_ram_sdp #(.DW(8), .AW(10), .STAGE2B_BLOCK(1)) u_con')]}
    for rel in ('nav/rtl/match/slf_match.v','nav/rtl/plan/traj_eval.v'):
      selections.setdefault(rel,[]).extend((f'nav_ram_sdp #(.DW(16), .AW(10), .INIT_FILE(SIN_INIT)) {name}',f'nav_ram_sdp #(.DW(16), .AW(10), .INIT_FILE(SIN_INIT), .STAGE2B_BLOCK(1)) {name}') for name in ('u_sin','u_cos'))
    for rel,edits in selections.items():
      for old,new in edits:result[rel]=replace_once(result[rel],old,new)
      back=result[rel]
      for old,new in reversed(edits):back=replace_once(back,new,old)
      assert back==texts[rel],('Consumer procedural/ACK logic changed',rel)
    result['fc_v4_top.v']=bias_transform(result['fc_v4_top.v'])
    assert result['fc_v4_top.v'].replace('(* syn_ramstyle="block_ram",syn_romstyle="block_rom" *)reg signed [25:0] bias_mem [0:424];',
      '(* syn_ramstyle="distributed_ram",syn_romstyle="distributed_rom" *)reg signed [25:0] bias_mem [0:424];',1)==texts['fc_v4_top.v']
    return result

def contracts(source,out):
    common='One posedge enabled read: q(next)=mem(pre-edge)[raddr] when re, otherwise q holds; write updates mem at the edge. we&&re&&same address returns old data. q starts0; memory INIT_VAL or full INIT_FILE preserved. No reset port is added.'
    rows=[
      dict(name='bitmap',scope='u_nav/u_map/u_tv/u_bm',geometry='16384x1',source_blocks=1,extra_wide=0,physical=1,
        consumer='trav_v3.v sm/bm request and phased return/window pipeline; all procedural code unchanged',port='1W+1R independently enabled every clock',initial='INIT_VAL0; fixture also covers nonzero1'),
      dict(name='mailboxes',scope=['u_nav/u_mbx/u_injb','u_nav/u_mbx/u_txs'],geometry='two512x32',source_blocks=2,extra_wide=2,physical=4,
        consumer='injection iv1/iv2 and byte lanes; shadow bus address/data/ACK; no RX queue, timestamp, TX sequence or arbitration edits',port='Each separate1W+1R; can read and write the same clock/address',initial='INIT_VAL0; fixtures cover32-bit nonzero; u_txb is not selected and u_rxb alreadyBSRAM'),
      dict(name='slot_list',scope='u_nav/u_match/u_slist',geometry='1024x16, SLF256 retained',source_blocks=1,extra_wide=0,physical=1,
        consumer='UPD writes slot IDs; clear-list read states and touch clearing retain their exact wait states',port='1W+1R',initial='INIT_VAL0'),
      dict(name='trig',scope=['u_nav/u_match/u_sin','u_nav/u_match/u_cos','u_nav/u_tr/u_sin','u_nav/u_tr/u_cos'],geometry='four1024x16 tables',source_blocks=4,extra_wide=0,physical=4,
        consumer='SLF and trajectory retain two simultaneous independent addresses/enables each; no sharing of lookup ports',port='Four independent enabled read ports; actual we tied0',initial='Exact existing sin_lut.hex, q initially0; no quantization'),
      dict(name='console',scope='u_soc/g_con[0..3].u_con',geometry='four1024x8 lanes,4096 bytes total',source_blocks=4,extra_wide=0,physical=4,
        consumer='Only con_wp low2 matching lane writes; four same-word reads form32bit con_q. d_st address-register/read/capture/ACK states unchanged',port='Four independent byte writes (actual mutually exclusive), four common-word reads; preserve debug read/write collisions',initial='INIT_VAL0; con_wp reset/wrap unchanged'),
      dict(name='bias',scope='u_control/model_thread/policy/core/bias_mem',geometry='425x26 signed ->32bit',source_blocks=1,extra_wide=1,physical=2,
        consumer='BIAS_REQ updates bias_q, BIAS_CAPTURE consumes next edge; FSM, weights, cycle count and125Hz period unchanged',port='One enabled ROM read; no writes; collisions N/A',initial='Exact BIASES_FILE425 words; original bias_q initialX/no reset unchanged; addresses425..511 remain outside declared array, not newly zero-padded')]
    for row in rows:row.update(common_RAM_contract=common if row['name']!='bias' else 'Exact original signed read/hold statements, init file and declared depth; only attributes differ.',
      source_prediction_not_measurement=True,mapped_read_first_hold_INIT_and_ports_verified=False)
    return dict(phase='prepared_not_validated',rows=rows,SLF_LOG2N=8,column_scratch_selected=False,
      standalone_prediction=dict(SYN40_source=314,source_add=13,predicted_source=327,original_wide=18,wide_add=3,predicted_wide=21,predicted_physical=348,standalone_admitted=False),
      combined_B_route_prediction=dict(base=332,weight9=-5,point_planes=-8,bitmap_mailbox_slist_trig=10,console_bias=6,total=335,hard_limit=336,target_max=335,
        requires_parent_weight_and_point_candidate=True,actual_resource_report_pending=True),
      common_scope='No inferred6144logic saving or CLS saving is claimed; those are current storage accounting. New physical placement/timing remains unknown.',
      source=str(source),candidate=str(out),deployment_accepted=False)

def fixtures(source,out):
    folder=out/'validation';folder.mkdir()
    old=(source/RAM).read_text(encoding='utf-8-sig');new=(out/RAM).read_text(encoding='utf-8-sig')
    (folder/'old_ram.v').write_text(old.replace('module nav_ram_sdp #(','module old_ram #(',1),encoding='utf-8')
    (folder/'new_ram.v').write_text(new.replace('module nav_ram_sdp #(','module new_ram #(',1),encoding='utf-8')
    shutil.copy2(Path(__file__).with_name('tb_stage2b_storage_contract_observation2.sv'),folder/'tb_stage2b_storage_contract.sv')
    shutil.copy2(source/'sin_lut.hex',folder/'sin_lut.hex')
    # The bias wrapper uses declarations/read syntax extracted from the exact
    # old/new complete core; it does not substitute inference or run4500.
    for label,root in (('old',source),('new',out)):
      text=(root/'fc_v4_top.v').read_text(encoding='utf-8-sig')
      declaration=next(line.strip() for line in text.splitlines() if 'bias_mem [0:424];' in line)
      statement=next(line.strip() for line in text.splitlines() if 'if (state == BIAS_REQ) bias_q <= bias_mem[bias_address];' in line)
      wrapper='module bias_'+label+'(input clk,en,input[8:0]a,output reg signed[31:0]bias_q);\n'+declaration+'\nlocalparam BIAS_REQ=1;wire[0:0]state=en;wire[8:0]bias_address=a;\ninitial $readmemh("bias_narrow.mem",bias_mem);\nalways@(posedge clk)begin '+statement+' end\nendmodule\n'
      (folder/(label+'_bias.v')).write_text(wrapper,encoding='utf-8')
    shutil.copy2(source/'src/mem/bias_narrow.mem',folder/'bias_narrow.mem')
    commands=[['xvlog',['old_ram.v','new_ram.v','old_bias.v','new_bias.v']],['xvlog',['-sv','tb_stage2b_storage_contract.sv']],
      ['xelab',['-debug','off','tb_stage2b_storage_contract','-s','stage2b_contract_snap']],['xsim',['stage2b_contract_snap','-runall']]]
    save(folder/'entry.json',dict(phase='prepared_only_not_executed',commands=commands,cwd=str(folder),
      intended_tool_directory='E:/2026.1/Vivado/bin',Gowin_SYN_or_PNR_commands=[],
      requires_root_explicit_validation_authorization=True,admission_physical_bytes=2*1024**3,admission_commit_bytes=4*1024**3,
      scope='RTL operation contract only; no vendor mapped primitives before the one joint stage3 SYN. No real CPU/350ms/vehicle or whole4500 acceptance.',
      remaining_checks=['After stage3: inspect exact source/physical geometry and all INIT bits, replay these contract vectors through vendor mapped primitives.','Full consumer miter or existing bus tests: no ACK/handshake procedural edits plus identical RAM boundary sequences must compose; currently not simulated.','Original4500 frame/cycle regression belongs to the parent combined contract, not these isolated bias reads.']))

def prepare(source,out):
    source=Path(source).resolve();out=Path(out).resolve();assert not out.exists()
    manifest=json.loads((source/'manifest.json').read_text(encoding='utf-8-sig'))
    selected={Path(p).relative_to(source).as_posix():Path(p) for p in manifest['source_files']};assert len(selected)==63
    files=set(selected.values())
    for p in source.rglob('*'):
      if not p.is_file() or any(q in ('impl','__pycache__') for q in p.relative_to(source).parts):continue
      rel=p.relative_to(source)
      if rel.parts[0] in ('fw','firmware') or p.suffix.lower() in ('.mem','.hex','.bin','.mif','.h','.vh') or rel.as_posix() in ('g2.cst','g2.sdc'):files.add(p)
    bound=[meta(p) for p in sorted(files)];texts={rel:path.read_text(encoding='utf-8-sig') for rel,path in selected.items()}
    changed=transform_sources(texts);assert {r for r in selected if changed[r]!=texts[r]}==CHANGED
    out.mkdir(parents=True)
    for p in sorted(files):
      dest=out/p.relative_to(source);dest.parent.mkdir(parents=True,exist_ok=True);shutil.copy2(p,dest)
    for rel in sorted(CHANGED):(out/rel).write_text(changed[rel],encoding='utf-8',newline='\n')
    differences=[dict(path=p.relative_to(source).as_posix(),old_bytes=p.stat().st_size,new_bytes=(out/p.relative_to(source)).stat().st_size)
      for p in sorted(files) if p.read_bytes()!=(out/p.relative_to(source)).read_bytes()]
    assert {r['path'] for r in differences}==CHANGED
    new_manifest=dict(manifest);new_manifest.update(source_files=[str(out/r) for r in selected],frozen_source_build=str(source),
      stage2B_back_to_BSRAM_prepared_only=True,synthesis_complete=False,automatic_PNR=False,programmed=False,
      SLF_LOG2N_unchanged=8,column_scratch_migrated=False,stage2B_expected_combined_physical_BSRAM=335)
    save(out/'manifest.json',new_manifest);save(out/'contracts.json',contracts(source,out));save(out/'source_diff.json',differences)
    fixtures(source,out)
    unchanged=bound==[meta(Path(r['path'])) for r in bound];assert unchanged
    record=dict(phase='frozen_prepared_only',recorded_at=datetime.now().astimezone().isoformat(),source=str(source),candidate=str(out),
      hardware_inputs=63,changed_RTL=sorted(CHANGED),all_other_copied_inputs_byte_identical=True,copied_files=len(files),
      source_inputs_metadata_unchanged=unchanged,source_bindings=bound,
      static_operation_identity='Selected branch copies original initial/write/read/hold body exactly except attributes; consumer procedural code/ACK is exactly unchanged after removing the new opt-in parameter. Bias read/FSM/init statements are exact.',
      old_reusable_tools=['prepare_small_block_stores.py (bitmap/mailbox geometry and old mapped read-first tests)','prepare_trig_block_rom.py (same procedural initialized-read branch)','prepare_bias_block_rom.py (exact reused transform)'],
      old_proof_limits='Previous storage sources and full designs differ; current target mapping, source/physical split, INIT bits, routing, CLS and functional consumer regressions must be rechecked. Old runners start isolated SYN and must not be called during stage2B.',
      validation_entry=str(out/'validation/entry.json'),RTL_compiled=False,RTL_simulated=False,Gowin_started=False,
      blockers=['No simulator is authorized in this freeze stage.','One joint stage3 SYN is required to confirm predicted13 source/3 wide additions and native read/INIT semantics.','Candidate alone predicts348physical and must never be submitted standalone; parent weight/point planes reduce combined prediction to335.'],
      production_source_or_plan_modified=False,deployment_accepted=False)
    save(out/'preparation.json',record)
    shutil.copy2(Path(__file__),out/Path(__file__).name)
    print(json.dumps({k:record[k] for k in ('phase','candidate','hardware_inputs','changed_RTL','copied_files','RTL_compiled','RTL_simulated','Gowin_started')},ensure_ascii=False))

if __name__=='__main__':
    parser=argparse.ArgumentParser();parser.add_argument('--source',type=Path,default=SOURCE);parser.add_argument('--out',type=Path,default=DEFAULT_OUT)
    args=parser.parse_args();prepare(args.source,args.out)
