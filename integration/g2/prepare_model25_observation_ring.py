"""Explicit-only independent six-slot observation ring source candidate.

Four RTL inputs change; the original core/mathematics and default flat-storage
implementations remain intact. This script starts no EDA or SYN38 controller.
"""
from pathlib import Path
from datetime import datetime
import argparse,json,re,shutil,difflib
from prepare_baseline_fusion_retime import same_bytes,metadata
BASE=Path('F:/nav_integration_g2_20261023')
SOURCE=BASE/'baseline_fusion_model25_bounded_MAX_syn37'
PREP=BASE/'baseline_fusion_model25_observation_ring_prepared1'
FILES=['control/rl_observation_v5.v','control/rl_policy_tick_v5.v','control/mem_policy.v','control/mem_policy_related25.v']

RING=r'''
    generate if (OBS_RING_STORAGE != 0) begin: g_ring
        // No RAM reset/INIT: only a complete28word generation is published.
        (* syn_ramstyle="distributed_ram",syn_romstyle="distributed_rom" *)
        reg signed [23:0] frame_mem [0:167];
        reg [2:0] work_slot;
        reg [2:0] live0,live1,live2,live3,live4;
        reg [2:0] frozen0,frozen1,frozen2,frozen3,frozen4;
        wire lane_write=(state==PREPARE && denominators[lane]==1) || state==SAVE;
        wire signed [23:0] lane_value=state==SAVE ?
            sat24(negative ? -$signed(quotient) : $signed(quotient)) :
            sat24(numerators[lane]);
        wire [7:0] write_address=work_slot*8'd28+lane;
        // Pointer/segment selects are narrow; no4032bit frame bus is rebuilt.
        wire [2:0] segment=frame_read_index<28 ? 0 : frame_read_index<56 ? 1 :
            frame_read_index<84 ? 2 : frame_read_index<112 ? 3 :
            frame_read_index<140 ? 4 : 5;
        wire [7:0] read_lane=frame_read_index-segment*8'd28;
        wire [2:0] r0=frame_freeze ? live0 : frozen0;
        wire [2:0] r1=frame_freeze ? live1 : frozen1;
        wire [2:0] r2=frame_freeze ? live2 : frozen2;
        wire [2:0] r3=frame_freeze ? live3 : frozen3;
        wire [2:0] r4=frame_freeze ? live4 : frozen4;
        wire [2:0] read_slot=segment==0 ? r4 : segment==1 ? r0 :
            segment==2 ? r1 : segment==3 ? r2 : segment==4 ? r3 : r4;
        wire [7:0] read_address=read_slot*8'd28+read_lane;
        wire signed [23:0] raw_word=frame_mem[read_address];
        assign frame_read_data=frame_read_index>=168 ? 24'sd0 :
            segment==0 ? clip_current(raw_word) : raw_word;
        always @(posedge clk) begin
            if(rst_n && lane_write) frame_mem[write_address]<=lane_value;
        end
        always @(posedge clk or negedge rst_n) begin
            if(!rst_n) begin
                work_slot<=0;live0<=0;live1<=0;live2<=0;live3<=0;live4<=0;
                frozen0<=0;frozen1<=0;frozen2<=0;frozen3<=0;frozen4<=0;
            end else begin
                if(state==COMMIT) begin
                    if(!initialized) begin
                        live0<=work_slot;live1<=work_slot;live2<=work_slot;
                        live3<=work_slot;live4<=work_slot;
                    end else begin
                        live0<=live1;live1<=live2;live2<=live3;live3<=live4;live4<=work_slot;
                    end
                    work_slot<=work_slot==5 ? 0 : work_slot+1'b1;
                end
                if(frame_freeze && initialized) begin
                    frozen0<=live0;frozen1<=live1;frozen2<=live2;
                    frozen3<=live3;frozen4<=live4;
                end
            end
        end
    end else begin: g_flat_default
        assign frame_read_data=24'sd0;
    end endgenerate
'''

def once(text,old,new):
 assert text.count(old)==1,(old,text.count(old));return text.replace(old,new)
def derive_observation(text):
 text=once(text,'parameter integer ENCODER_CPR = 1320,','parameter integer OBS_RING_STORAGE = 0,\n    parameter integer ENCODER_CPR = 1320,')
 text=once(text,'input wire clk, rst_n, capture,','input wire clk, rst_n, capture,\n    input wire frame_freeze,\n    input wire [7:0] frame_read_index,\n    output wire signed [23:0] frame_read_data,')
 text=once(text,'words[lane]<=sat24(numerators[lane]);','if(OBS_RING_STORAGE==0) words[lane]<=sat24(numerators[lane]);')
 text=once(text,'words[lane]<=sat24(negative ? -$signed(quotient) : $signed(quotient));','if(OBS_RING_STORAGE==0) words[lane]<=sat24(negative ? -$signed(quotient) : $signed(quotient));')
 old='''                    for(j=0;j<28;j=j+1) current_flat[j*24+:24]<=clip_current(words[j]);
                    if(!initialized) begin
                        for(j=0;j<140;j=j+1) history_flat[j*24+:24]<=words[j%28];
                    end else begin
                        history_flat[2687:0]<=history_flat[3359:672];
                        for(j=0;j<28;j=j+1) history_flat[(112+j)*24+:24]<=words[j];
                    end'''
 text=once(text,old,'                    if(OBS_RING_STORAGE==0) begin\n'+old+'\n                    end')
 text=once(text,'\nendmodule','\n'+RING+'\nendmodule');return text
def derive_tick(text):
 text=once(text,'parameter integer WEIGHT_BITS = 16,','parameter integer OBS_RING_STORAGE = 0,\n    parameter integer WEIGHT_BITS = 16,')
 text=once(text,'input wire frame_valid,','input wire frame_valid,\n    output wire frame_freeze,\n    output wire [7:0] frame_read_index,\n    input wire signed [23:0] frame_read_data,')
 text=once(text,'reg [4031:0] snapshot;','reg [4031:0] snapshot;\n    reg signed [23:0] snapshot_word;')
 text=once(text,'.wr_data(snapshot[23:0]),','.wr_data(OBS_RING_STORAGE!=0 ? snapshot_word : snapshot[23:0]),')
 text=once(text,'wire deadline_missed = tick_count==PERIOD_CLKS-1 && state!=IDLE;','''wire deadline_missed = tick_count==PERIOD_CLKS-1 && state!=IDLE;
    assign frame_freeze=OBS_RING_STORAGE!=0 && state==SNAPSHOT && frame_valid && run_ok && !deadline_missed;
    // C680 freezes slot IDs and prefetches0. Each LOAD writes the old word FF
    // and prefetches j+1; final LOAD167 does not request an extra source word.
    assign frame_read_index=state==SNAPSHOT ? 8'd0 : load_index+8'd1;''')
 text=once(text,'load_index<=0; snapshot<=0;','load_index<=0; if(OBS_RING_STORAGE==0) snapshot<=0; snapshot_word<=0;')
 text=once(text,'snapshot<={history_flat,current_flat}; load_index<=0; state<=LOAD;','''if(OBS_RING_STORAGE!=0) snapshot_word<=frame_read_data;
                            else snapshot<={history_flat,current_flat};
                            load_index<=0; state<=LOAD;''')
 text=once(text,"snapshot<={24'd0,snapshot[4031:24]};",'''if(OBS_RING_STORAGE!=0) begin
                                if(load_index!=167) snapshot_word<=frame_read_data;
                            end else snapshot<={24'd0,snapshot[4031:24]};''');return text
def derive_policy(text):
 text=once(text,'parameter integer FUSION_TIMEBASE_HZ=50000000,','parameter integer OBS_RING_STORAGE=0,\n    parameter integer FUSION_TIMEBASE_HZ=50000000,')
 text=once(text,'wire [3359:0] history_flat;','''wire [3359:0] history_flat;
    wire observation_freeze;
    wire [7:0] observation_read_index;
    wire signed [23:0] observation_read_data;''')
 text=once(text,'rl_observation_v5 #(.LEFT_ENCODER_SIGN(-1),','rl_observation_v5 #(.OBS_RING_STORAGE(OBS_RING_STORAGE),.LEFT_ENCODER_SIGN(-1),')
 text=once(text,'.valid(frame_valid),.initialized(history_initialized),','''.valid(frame_valid),.initialized(history_initialized),
        .frame_freeze(observation_freeze),.frame_read_index(observation_read_index),.frame_read_data(observation_read_data),''')
 text=once(text,'rl_policy_tick_v5 #(.WEIGHT_BITS(WEIGHT_BITS),','rl_policy_tick_v5 #(.OBS_RING_STORAGE(OBS_RING_STORAGE),.WEIGHT_BITS(WEIGHT_BITS),')
 text=once(text,'.frame_valid(frame_valid),.current_flat(current_flat),.history_flat(history_flat),','''.frame_valid(frame_valid),.current_flat(current_flat),.history_flat(history_flat),
        .frame_freeze(observation_freeze),.frame_read_index(observation_read_index),.frame_read_data(observation_read_data),''');return text
def derive_wrapper(text):
 return once(text,'mem_policy #(.WEIGHT_BITS(WEIGHT_BITS),','mem_policy #(.OBS_RING_STORAGE(1),.WEIGHT_BITS(WEIGHT_BITS),')
def prepare():
 assert not PREP.exists();shutil.copytree(SOURCE,PREP,ignore=shutil.ignore_patterns('impl','__pycache__','job.json','tool_runtime.json','*.log'))
 for rel,derive in zip(FILES,(derive_observation,derive_tick,derive_policy,derive_wrapper)):
  old=(SOURCE/rel).read_text();new=derive(old);(PREP/rel).write_text(new,encoding='utf8')
  (PREP/(Path(rel).name+'.ring.patch')).write_text(''.join(difflib.unified_diff(old.splitlines(True),new.splitlines(True),fromfile='SYN37/'+rel,tofile='ring/'+rel)),encoding='utf8')
 m=json.loads((SOURCE/'manifest.json').read_text());m['source_files']=[str(PREP/Path(n).relative_to(SOURCE))for n in m['source_files']]
 m.update(candidate='explicit_observation_ring6_source_only',source_preparation_only=True,automatic_PNR=False,synthesis_complete=False,OBS_RING_STORAGE_selected=1,OBS_RING_STORAGE_module_default=0)
 (PREP/'manifest.json').write_text(json.dumps(m,indent=2)+'\n')
 t=(SOURCE/'run.tcl').read_text().replace(str(SOURCE),str(PREP)).replace(SOURCE.as_posix(),PREP.as_posix())
 (PREP/'run.tcl.disabled_pending_ring_protocol').write_text(t,encoding='utf8');(PREP/'run.tcl').unlink()
 names={Path(n).relative_to(PREP).as_posix()for n in m['source_files']};assert len(names)==66
 changed=sorted(n for n in names if not same_bytes(SOURCE/n,PREP/n));assert changed==sorted(FILES)
 for suffix,count in[({'.hex','.mem','.bin','.mif'},69),({'.h','.vh'},31)]:
  paths=[p.relative_to(SOURCE)for p in SOURCE.rglob('*')if p.is_file()and'impl'not in p.relative_to(SOURCE).parts and'__pycache__'not in p.parts and p.suffix.lower()in suffix]
  assert len(paths)==count and all(same_bytes(SOURCE/n,PREP/n)for n in paths)
 for folder in('fw','firmware'):
  assert all(same_bytes(p,PREP/p.relative_to(SOURCE))for p in(SOURCE/folder).rglob('*')if p.is_file()and'__pycache__'not in p.parts)
 assert all(same_bytes(SOURCE/n,PREP/n)for n in('g2.cst','g2.sdc','fc_v4_top.v','fixed4500_weight_rom.v','control/imu_fusion.v','control/policy_action_map.v'))
 selected=re.findall(r'^add_file -type verilog \{(.+)\}$',t,re.M);assert len(selected)==66 and{Path(x).resolve()for x in selected}=={(PREP/n).resolve()for n in names}
 for func in('sat24','clip_current'):
  body=lambda x:re.search(r'function signed \[23:0\] '+func+r';.*?endfunction',x,re.S)[0]
  assert body((SOURCE/FILES[0]).read_text())==body((PREP/FILES[0]).read_text())
 result=dict(prepared=True,recorded_at=datetime.now().astimezone().isoformat(),source=str(SOURCE),prepared_directory=str(PREP),changed_RTL=changed,other62_RTL_identical=True,actual_hardware_inputs=66,assets69_headers31_FW_weights_core_input_mem_math_clocks_CST_SDC_unchanged=True,default_modules_keep_legacy_flat_implementation=True,candidate_wrapper_explicitly_selects_ring1=True,new_bus_interface='freeze1/index8/read_data24, no rebuilt full frame bus',RAM_initialization_assumed=False,full_SYN_PNR_started=False,SYN38_started=False,PNR50_modified=False,protocol_validation_pending=True,full4500_reference_comparison_pending=True,bindings=[metadata(PREP/n)for n in sorted(names)])
 (PREP/'prepared_source_review.json').write_text(json.dumps(result,indent=2)+'\n');return result
if __name__=='__main__':
 a=argparse.ArgumentParser();a.add_argument('--ring-storage',action='store_true');args=a.parse_args();assert args.ring_storage,'Explicit --ring-storage required; default remains old implementation.'
 print(json.dumps(prepare(),indent=2))
