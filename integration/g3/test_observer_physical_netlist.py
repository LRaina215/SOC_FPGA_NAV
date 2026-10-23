"""Replay observer bus vectors through the exact routed B1 netlist/pad models."""
import argparse,json,re,shutil,subprocess,sys
from pathlib import Path
ROOT=Path(__file__).resolve().parents[2]
sys.path.insert(0,str(ROOT/'integration/g2'))
from audit_pin_loads import CELLS,PORTS,NETS
from audit_netlist_memories import PARAM
WRAPPER='''module i2c_id_observer(input wire clk,reset_n,scl,sda,
 output wire[127:0]address_ack_count,address_nack_count,id_read_count,
 output wire[31:0]last_ids,output wire[3:0]id_seen);
 tri1 pad_scl,pad_sda;
 assign pad_scl=scl?1'bz:1'b0;assign pad_sda=sda?1'bz:1'b0;
 feedback_probe_top physical(.clk(clk),.reset_n(reset_n),.M1_ENCA(1'b0),.M1_ENCB(1'b0),.M2_ENCA(1'b0),.M2_ENCB(1'b0),
  .BMI088_SDA(pad_sda),.BMI088_SCL(pad_scl));
 assign address_ack_count=physical.address_acks;
 assign address_nack_count=physical.address_nacks;
 assign id_read_count=physical.id_reads;
 assign last_ids=physical.last_ids;
 assign id_seen=physical.id_seen_Z;
endmodule
'''


def observer_cone(text):
    # Cut only at the verified physical IOBUF outputs. Preserve all selected
    # primitive expressions, parameters and startup/reset logic verbatim.
    boundaries={'clk_d','reset_n_d','BMI088_SCL_in','BMI088_SDA_in'}
    def nets(expression):
        result=[]
        for name,hi,lo in NETS.findall(expression):
            name=name.replace('\\','')
            result += [name]if not hi else [name+'['+str(n)+']'for n in range(min(int(hi),int(lo or hi)),max(int(hi),int(lo or hi))+1)]
        return result
    items=[];drivers={}
    for match in CELLS.finditer(text):
        kind,name,body=match.groups();ports=dict(PORTS.findall(body))
        output_pins={'Q'}if kind.startswith('DFF')else {'F','O','OF','SUM','COUT','G','V'}
        outputs=[n for pin,expression in ports.items()if pin in output_pins for n in nets(expression)]
        inputs=[n for pin,expression in ports.items()if pin not in output_pins for n in nets(expression)]
        index=len(items);items.append(dict(kind=kind,name=name,text=match[0],inputs=inputs))
        for n in outputs:drivers[n]=index
    for match in re.finditer(r'^assign\s+(.+?)\s*=\s*(.+?);',text,re.M):
        index=len(items);items.append(dict(kind='assign',name='',text=match[0],inputs=nets(match[2])))
        for n in nets(match[1]):drivers[n]=index
    pending=[name+'['+str(i)+']'for name,width in [('address_acks',128),('address_nacks',128),('id_reads',128),('last_ids',32),('id_seen_Z',4)]for i in range(width)]
    selected={i for i,item in enumerate(items)if item['kind']=='GSR'}
    for i in selected:pending+=items[i]['inputs']
    visited=set();unknown=set()
    while pending:
        n=pending.pop()
        if n in visited or n in boundaries:continue
        visited.add(n)
        if n not in drivers:unknown.add(n);continue
        index=drivers[n]
        if index not in selected:selected.add(index);pending+=items[index]['inputs']
    if unknown:raise ValueError('unresolved observer-cone drivers: '+str(sorted(unknown)))
    names={items[i]['name'].replace('\\','').strip()for i in selected}
    parameters=[m[0]for m in re.finditer(r'^defparam\s+.*?;',text,re.M)if any(name.replace('\\','').strip()in names for name,key,value in PARAM.findall(m[0]))]
    init_names={name.replace('\\','').strip()for line in parameters for name,key,value in PARAM.findall(line)if key=='INIT'}
    missing_init=[items[i]['name']for i in selected if(items[i]['kind'].startswith('LUT')or items[i]['kind'].startswith('DFF'))and items[i]['name'].replace('\\','').strip()not in init_names]
    if missing_init:raise ValueError('missing exact primitive INIT parameters: '+str(missing_init))
    declarations='\n'.join(line for line in text.splitlines()if line.startswith('wire '))
    header='''module observer_physical_cone(input wire clk,reset_n,scl,sda,
 output wire[127:0]address_acks,address_nacks,id_reads,output wire[31:0]last_ids,output wire[3:0]id_seen_Z);\n'''
    declarations='\n'.join(line for line in declarations.splitlines()if not re.search(r'\b(clk|reset_n|scl|sda|address_acks|address_nacks|id_reads|last_ids|id_seen_Z)\s*;',line))
    result=header+declarations+'\nassign clk_d=clk;assign reset_n_d=reset_n;assign BMI088_SCL_in=scl;assign BMI088_SDA_in=sda;\n'
    result+='\n'.join(items[i]['text']for i in sorted(selected))+'\n'+'\n'.join(parameters)+'\nendmodule\n'
    return result,dict(selected_primitives=sum(items[i]['kind']!='assign'for i in selected),selected_parameter_statements=len(parameters),all_lut_dff_init_preserved=True,full_primitives=sum(item['kind']!='assign'for item in items),boundaries=sorted(boundaries),unresolved_drivers=[])


def main():
    p=argparse.ArgumentParser();p.add_argument('--build',type=Path,required=True);p.add_argument('--out',type=Path,required=True);p.add_argument('--observer-cone',action='store_true');p.add_argument('--trace',action='store_true');a=p.parse_args()
    out=a.out.resolve();out.mkdir(parents=True,exist_ok=False)
    source=a.build/'impl/pnr/feedback_probe.vo';shutil.copyfile(source,out/'physical.v')
    text=(ROOT/'integration/g3/tb_i2c_id_observer.sv').read_text()
    text=text.replace('#30;reset_n=1;idle();','#30;reset_n=1;#6000;idle();')
    text=text.replace('reset_n=0;#40;reset_n=1;stop();idle();','reset_n=0;#40;reset_n=1;#6000;stop();idle();')
    if a.trace:
        text=text.rsplit('endmodule',1)[0]+r'''
 always@(scl or sda)begin #60;
 $display("TRACE t=%0t startreg=%h rst=%b sc=%b sd=%b active=%b bit=%d byte=%d shift=%h known=%b",
 $time,dut.physical.startup,dut.physical.n858_7,dut.physical.\observer/scl_sync ,dut.physical.\observer/sda_sync ,dut.physical.\observer/active ,dut.physical.\observer/bit_index ,dut.physical.\observer/byte_index ,dut.physical.\observer/shift ,dut.physical.\observer/target_known );
 end
 endmodule
 '''
    wrapper=WRAPPER;netlist_file=out/'physical.v';cone=None
    if a.observer_cone:
        cone_text,cone=observer_cone(netlist_file.read_text());netlist_file=out/'observer_cone.v';netlist_file.write_text(cone_text)
        (out/'cone_manifest.json').write_text(json.dumps(cone,indent=2))
        wrapper=WRAPPER.replace('feedback_probe_top physical(.clk(clk),.reset_n(reset_n),.M1_ENCA(1\'b0),.M1_ENCB(1\'b0),.M2_ENCA(1\'b0),.M2_ENCB(1\'b0),\n  .BMI088_SDA(pad_sda),.BMI088_SCL(pad_scl));','observer_physical_cone physical(.clk(clk),.reset_n(reset_n),.scl(scl),.sda(sda));')
    (out/'tb.sv').write_text(wrapper+'\n'+text)
    viv=Path('E:/2026.1/Vivado/bin');library=Path('D:/Gowin2/Gowin_V1.9.12.04_x64/IDE/simlib/gw5a/prim_sim.v')
    def run(tool,args):
        with(out/(tool+'_stdout.log')).open('w')as log:
            subprocess.run(['cmd','/c',str(viv/(tool+'.bat')),*args],cwd=out,stdout=log,stderr=subprocess.STDOUT,check=True,timeout=120)
    run('xvlog',[str(library),str(netlist_file)]);run('xvlog',['-sv',str(out/'tb.sv')])
    run('xelab',['-debug','typical','-timescale','1ns/1ps','tb_i2c_id_observer','-s','snap']);run('xsim',['snap','-runall'])
    log=(out/'xsim.log').read_text(errors='replace');passed='I2C_OBSERVER_PASS'in log and 'Fatal:'not in log
    blocked='license tier'in log and 'exceed maximum allowed'in log
    result=dict(passed=None if blocked else passed,validation_executed=not blocked,blocked_reason='simulator instance license limit'if blocked else None,
        physical_netlist=str(source.resolve()),vendor_library=str(library),build=str(out),cone=cone,
        scope='exact physical observer cone, cut at IOBUF outputs; ideal bus only, no SDF, full-design or actual board waveform validation'if cone else 'exact routed netlist functional replay through pad/primitive models; ideal bus only, no SDF or actual board waveform',programmed=False)
    (out/'result.json').write_text(json.dumps(result,indent=2));print(json.dumps(result))
    if not passed:raise RuntimeError(log[-2500:])


if __name__=='__main__':main()
