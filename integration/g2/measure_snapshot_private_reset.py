"""Check every valid core transfer and public output after private reset removal."""
import argparse
import json
from pathlib import Path
import re
import xml.etree.ElementTree as ET

from measure_slf_row_banks import TOOL,run_syn,simulate,inventory
from preflight import validate
from prepare_snapshot_private_reset import transform
from synthesis_connectivity import read

ROOT=Path(__file__).resolve().parents[2]


def bench():
    text=(ROOT/'integration/g2/tb_snapshot_fanout.sv').read_text()
    text=text.replace('integer k;','integer k;reg defined_frame=0;integer directed_resets=0;')
    text=text.replace('if(rst_n)begin\n   if({a.core_rst_n',
        'if(!rst_n)defined_frame<=0;\n  else if(a.state==1 && frame_valid && a.snapshot_update)defined_frame<=1;\n  if(rst_n)begin\n   if(a.core.wr_en && !defined_frame)$fatal(1,"read before full snapshot");\n   if({a.core_rst_n',1)
    # The core ignores its data input unless wr_en. It still receives identical
    # reset, enable, address, history flag, start and frame ID on every edge.
    text=text.replace('a.core.wr_addr,a.core.wr_data,a.core.start','a.core.wr_addr,a.core.start')
    text=text.replace('b.core.wr_addr,b.core.wr_data,b.core.start','b.core.wr_addr,b.core.start')
    text=text.replace('if(a.core.wr_en)writes=writes+1;',
        'if(a.core.wr_en)begin\n    if(a.core.wr_data!==b.core.wr_data)$fatal(1,"valid core word differs");\n    writes=writes+1;end')
    text=text.replace(' || a.snapshot !== b.snapshot)',
        ' || (defined_frame && a.snapshot !== b.snapshot))')
    text=text.replace('end else begin sensor_ok=1;enable=1;end',
        '''end else begin sensor_ok=1;enable=1;end
  // Reset between clock edges at early/middle/final LOAD words.
  if(cycle>=30000)begin
   if(cycle<35000 && directed_resets<3 && a.state==2 &&
       a.load_index==(directed_resets==0 ? 3 : directed_resets==1 ? 83 : 167))begin
    rst_n=0;resets=resets+1;directed_resets=directed_resets+1;
   end else rst_n=1;
  end''')
    text=text.replace('resets<4','resets<7 || directed_resets!=3')
    text=text.replace('SNAPSHOT_FANOUT_PASS','PRIVATE_RESET_PASS')
    return text.replace('tb_snapshot_fanout','tb')


def finish(out):
    result=json.loads((out/'result.json').read_text())
    assert len(result['checks'])==1 and 'TEST PASSED PRIVATE_RESET' in result['checks'][0]['result']
    assert [r['snapshot_variable_reset_loads'] for r in result['runs']]==[4032,24]
    assert all(r['inventory']['bsram']==0 for r in result['runs'])
    # The24 retained CLEAR pins implement zero fill of the top word during
    # LOAD; they are driven by shift/update controls, not the external reset.
    counts=[];data_clear_drivers=[]
    for name in ('original','private'):
        cells=read(out/name/'impl/gwsynthesis/g2_top.vg')
        root,=[v['ports']['O'][0] for v in cells.values() if v['kind']=='IBUF' and v['ports'].get('I')==['rst_n']]
        reset_nets={root}|{v['ports']['O'][0] for v in cells.values() if v['kind']=='INV' and v['ports'].get('I')==[root]}
        snapshot={n:c for n,c in cells.items() if c['kind'].startswith('DFF') and n.startswith('snapshot_')}
        pins=[net for c in snapshot.values() for p,v in c['ports'].items()
              if p in ('RESET','CLEAR','PRESET','SET') for net in v if net not in ('GND','VCC')]
        counts.append(sum(n in reset_nets for n in pins))
        if name=='private':
            for net in set(pins):
                driver,=[v for v in cells.values() if v['ports'].get('F')==[net]]
                assert driver['kind']=='LUT2'
                assert driver['ports']['I0'][0].startswith('snapshot_shift_select')
                assert driver['ports']['I1'][0].startswith('snapshot_update')
                data_clear_drivers.append(driver)
    assert counts==[4032,0]
    result.update(passed=True,actual_module_system_reset_loads_removed=4032,
                  remaining_data_zero_fill_clear_pins=24,remaining_clear_drivers=data_clear_drivers,
                  actual_module_variable_control_pins_removed=4008,joint_benefit_not_yet_measured=True,
                  first_resource_assertion='initial checker overcounted24 data zero-fill CLEAR pins as external reset; classified using actual drivers without rerunning SYN')
    (out/'result.json').write_text(json.dumps(result,indent=2)+'\n',encoding='utf-8')
    print(json.dumps({k:v for k,v in result.items() if k not in ('runs',)}),flush=True)


def main():
    ap=argparse.ArgumentParser();ap.add_argument('source',type=Path);ap.add_argument('--out',type=Path,required=True)
    ap.add_argument('--finish-existing',action='store_true',help='classify completed reset/data-clear measurements; never rerun a tool')
    a=ap.parse_args();source=a.source.resolve();out=a.out.resolve()
    if a.finish_existing:return finish(out)
    out.mkdir(parents=True,exist_ok=False)
    old=(source/'control/rl_policy_tick_v5.v').read_text();new=transform(old)
    (out/'old.v').write_text(old.replace('RL_POLICY_TICK_V5_V_INCLUDED','PRIVATE_OLD_INCLUDED').replace('module rl_policy_tick_v5 #(','module snapshot_original #('),encoding='utf-8')
    (out/'new.v').write_text(new,encoding='utf-8')
    text=bench()
    result=dict(source=str(source),build=str(out),changed_input='control/rl_policy_tick_v5.v',private_buffer_bits=4032,
                capacity_and_load_cycles_unchanged=True,algorithm_weights_and125hz_unchanged=True,
                programmed=False,deployment_accepted=False,checks=[],runs=[])
    def save(): (out/'result.json').write_text(json.dumps(result,indent=2)+'\n',encoding='utf-8')
    # The bench's core stub accumulates every valid word and injects faults.
    check=simulate(out,'rtl',[out/'old.v',out/'new.v'],text.replace('PRIVATE_RESET_PASS','TEST PASSED PRIVATE_RESET'))
    result['checks'].append(check);save()
    stub=text.split('module fc_v4_top',1)[1].split('endmodule',1)[0]
    stub=('module fc_v4_top'+stub+'endmodule\n').replace('tb.mode',"3'd0")
    for name,policy in [('original',old),('private',new)]:
        folder=out/name;folder.mkdir()
        # Keeping the public ports exposes all control/diagnostic behavior.
        top=policy.replace('module rl_policy_tick_v5 #(','module g2_top #(')
        (folder/'policy.v').write_text(top,encoding='utf-8');(folder/'core.v').write_text(stub,encoding='utf-8')
        (folder/'clock.sdc').write_text('create_clock -name clk -period 20 [get_ports {clk}]\n',encoding='utf-8')
        settings=['set_device -device_version C GW5AT-LV138PG484AC1/I0']
        settings += [f'add_file -type verilog {{{(folder/f).as_posix()}}}' for f in ('policy.v','core.v')]
        settings += [f'add_file -type sdc {{{(folder/"clock.sdc").as_posix()}}}','set_option -top_module g2_top',
                     'set_option -output_base_name g2_top','set_option -global_freq 50',
                     'set_option -convert_sdp32_36_to_sdp16_18 0','set_option -looplimit 100000']
        options=validate(TOOL,folder,settings,50,0)
        (folder/'run.tcl').write_text('\n'.join(settings+['run syn','exit'])+'\n',encoding='utf-8')
        runtime=run_syn(folder,'isolated private snapshot reset comparison with identical core stub')
        syn=folder/'impl/gwsynthesis'
        # The policy is the top module here, so snapshot names have no prefix.
        cells=read(syn/'g2_top.vg')
        snapshot={n:c for n,c in cells.items() if c['kind'].startswith('DFF') and n.startswith('snapshot_')}
        assert len(snapshot)==4032
        loads=sum(net not in ('GND','VCC') for c in snapshot.values() for p,v in c['ports'].items()
                  if p in ('RESET','CLEAR','PRESET','SET') for net in v)
        row=dict(variant=name,runtime=runtime,effective_options=options,
                 resources=ET.parse(syn/'g2_top_syn_rsc.xml').getroot().attrib,
                 snapshot_registers=len(snapshot),snapshot_variable_reset_loads=loads,inventory=inventory(syn/'g2_top.vg'))
        result['runs'].append(row);save();print(json.dumps(dict(variant=name,snapshot_reset_loads=loads,resources=row['resources'])),flush=True)
    finish(out)


if __name__=='__main__':main()
