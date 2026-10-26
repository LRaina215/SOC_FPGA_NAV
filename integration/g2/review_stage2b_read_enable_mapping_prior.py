"""Read existing C netlists for read-port CE inference; do not launch EDA."""
from pathlib import Path
from datetime import datetime
import argparse,json,re

BASE=Path('F:/nav_integration_g2_20261023')
OLD=BASE/'small_block_stores_probe1'
CURRENT=BASE/'stage2B_TXshadow_RDEN_prepared2/nav/rtl/common/nav_ram_sdp.v'
DOC=BASE/'stage2B_BSRAM_mode_collision_review1/UG300.txt'
def load(p):return json.loads(p.read_text(encoding='utf-8-sig'))
def meta(p):
 s=p.stat();return dict(path=str(p),bytes=s.st_size,mtime_ns=s.st_mtime_ns)
def cells(p):
 text=p.read_text(encoding='utf-8-sig');rows=[]
 for m in re.finditer(r'(?m)^\s*(SDPB|SDPX9B)\s+(\\[^\s]+|\w+)\s*\((.*?)\n\s*\);',text,re.S):
  kind,name,body=m.groups();ports={pin:re.sub(r'\s+','',net) for pin,net in re.findall(r'\.(\w+)\(([^)]*)\)',body)}
  params={k:v.strip() for k,v in re.findall(r'\bdefparam\s+'+re.escape(name)+r'\s*\.(\w+)\s*=\s*([^;]+);',text)}
  rows.append(dict(kind=kind,name=name,line=text[:m.start()].count('\n')+1,ports={k:ports[k] for k in ['CLKA','CLKB','CEA','CEB','OCE','RESET','ADA','ADB','BLKSELA','BLKSELB']},parameters={k:v for k,v in params.items() if not k.startswith('INIT_RAM_')}))
 return rows

def main(out):
 syn=OLD/'physical/impl/gwsynthesis/g2_top.vg';pnr=OLD/'physical/impl/pnr/g2_top.vo';source=OLD/'physical/ram.v'
 rows=[dict(phase='synthesis',netlist=meta(syn),cells=cells(syn)),dict(phase='post_PNR',netlist=meta(pnr),cells=cells(pnr))]
 assert [len(x['cells']) for x in rows]==[2,3]
 for group in rows:
  for c in group['cells']:
   assert c['kind']=='SDPB' and c['ports']['CEA']=='we_d' and c['ports']['CEB']=='re_d' and c['ports']['OCE']=='GND'
   assert c['parameters']['READ_MODE']=="1'b0" and not any(k.startswith('WRITE_MODE') for k in c['parameters'])
 tcl=(OLD/'physical/run.tcl').read_text();assert 'set_device -device_version C GW5AT-LV138PG484AC1/I0' in tcl
 assert 'set_option -convert_sdp32_36_to_sdp16_18 1' in tcl
 old=source.read_text();new=CURRENT.read_text();manual=DOC.read_text(encoding='utf8')
 assert 'always @(posedge clk)if(re)qr<=bitmap[raddr];' in old
 assert 'always @(posedge clk)if(re)qr<={high_mem[raddr],low_mem[raddr]};' in old
 assert 'always @(posedge clk)if(re)qr<=mem[raddr];' in new
 assert 'Port B clock enable signal, active-high.' in manual
 pages=manual.split('\f');sdp=next((i+1,x) for i,x in enumerate(pages) if 'SDPB/SDPX9B Port A is for write operation' in x)
 following=' '.join((sdp[1]+pages[sdp[0]]).split())
 assert 'Pipeline mode' in following and 'Bypass mode' in following
 prior=load(OLD/'result.json');assert prior['physical_run']['runtime']['tool_exit']==0
 result=dict(recorded_at=datetime.now().astimezone().isoformat(),scope='Read-only prior actual C mapping evidence; new stage2B leaves not synthesized',current_frozen_common_RAM=meta(CURRENT),current_style='STAGE2B_BLOCK single array, two posedge processes, if(we) mem[waddr]<=wdata; if(re) qr<=mem[raddr]; output qr',prior_source=meta(source),same_clocked_read_enable_style=True,important_source_differences=['Prior bitmap has the same single-array conditional-read structure, width1/depth16384.','Prior mailbox uses two16bit arrays concatenated into qr; synthesis merges them to one32bit leaf, then PNR splits it into two16bit leaves.','Current STAGE2B_BLOCK is parameterized single array; console8bit and other new consumer context have not been mapped.'],prior_device='GW5AT-LV138PG484AC1/I0 C',prior_tool_version='V1.9.12.04 (64-bit)',wide_split=True,actual_prior_mapping=rows,prior_tool_exit=0,confirmed_facts=['This style does not inevitably map re only to OCE: both prior synthesis leaves and all3 physical leaves used CEB=re_d, CEA=we_d, OCE=GND, READ_MODE=0.','With READ_MODE=0 bypass, OCE is not the read-port enable. CEB is the active-high portB clock enable.','No unsupported WRITE_MODE parameter occurs on these SDPB leaves.'],manual=dict(file=str(DOC),section='3.3 SDP, table3-7/3-8 around printed23-25',source_pdf='D:/Gowin2/Gowin_V1.9.12.04_x64/IDE/doc/EN/UG300-1.3.6E_Arora V BSRAM & SSRAM User Guide.pdf'),new_stage2B_CEA_CEB_OCE_not_yet_verified=True,physical_collision_electrical_behavior_not_proven=True,minimum_optional_mapping_plan=dict(automatic_run=False,scope='Only frozen current nav_ram_sdp.v plus one wrapper; no G2 or assets/CPU/firmware',instances=[dict(name='bitmap',DW=1,AW=14,DEPTH=16384),dict(name='console_lane',DW=8,AW=10,DEPTH=1024),dict(name='slist',DW=16,AW=10,DEPTH=1024),dict(name='TXshadow',DW=32,AW=9,DEPTH=512)],independent_write_and_read_inputs=True,all_output_bits_kept_to_ports=True,target='Same C device/version/options, synth-only initially; no PNR requested',strict_acceptance=['Every actual writable leaf READ_MODE=0, CEA actualwe, CEB actualre, no output-only OCE gating; prove any CE aliases/logic.','Reject unsupported WRITE_MODE=2 on DP/SP families before PNR; no manual netlist mode edits.','If a physical split is introduced, inspect both16bit lanes; inspect actual14bit ADA/ADB and enables.','Isolated wrapper demonstrates inference for these geometries only; final combined SYN must still prove actual consumer CE/BLKSEL/AD exclusion and public latency/INIT.']),no_source_changes=True,no_new_EDA_or_simulation=True,no_SHA=True)
 out.write_text(json.dumps(result,ensure_ascii=False,indent=2)+'\n',encoding='utf8')
 print(json.dumps(dict(output=str(out),prior_synthesis_leaves=2,prior_physical_leaves=3,all_CEB_re_and_OCE_ground=True,current_actual_mapping_verified=False,new_EDA_started=False),ensure_ascii=False))

if __name__=='__main__':
 p=argparse.ArgumentParser();p.add_argument('--out',type=Path,required=True);a=p.parse_args();main(a.out)
