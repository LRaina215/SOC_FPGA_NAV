"""Run finite RTL tests with installed XSim. No implementation/programming commands."""
import argparse
from datetime import datetime
import json
from pathlib import Path
import shutil
import subprocess
import sys
import time

HERE=Path(__file__).resolve().parent
VIV=Path('E:/2026.1/Vivado/bin')

def run(candidate,out,profiles):
    out.mkdir(parents=True,exist_ok=False)
    manifest=json.loads((candidate/'manifest.json').read_text(encoding='utf8'))
    source=Path(manifest['source'])
    results=[]
    state=dict(started_at=datetime.now().astimezone().isoformat(),profiles=results,
               candidate=str(candidate),simulation_only=True,board_programmed=False)
    def save(phase):
        state.update(phase=phase,updated_at=datetime.now().astimezone().isoformat())
        (out/'job.json').write_text(json.dumps(state,indent=2)+'\n',encoding='utf8')
    try:
        for profile in profiles:
            folder=out/profile;folder.mkdir()
            guard=str(candidate/'ultrasonic_drive_guard.v')
            if profile=='guard': files=[guard,str(HERE/'sim/tb_guard.sv')]
            elif profile=='led': files=[str(candidate/'ultrasonic_mode_led.v'),str(HERE/'sim/tb_led.sv')]
            elif profile=='range':
                files=[str(candidate/'g2_range_servo.v'),str(HERE/'sim/tb_range.sv')]
            elif profile=='nec': files=[guard,str(source/'control/ir_remote_control.v'),str(HERE/'sim/tb_nec.sv')]
            elif profile=='sensor_guard': files=[guard,str(candidate/'g2_range_servo.v'),str(HERE/'sim/tb_sensor_guard.sv')]
            elif profile=='policy':
                names=['mem_policy','imu_fusion','imu_isqrt','imu_recip','leg_ik_ff',
                       'policy_action_map','rl_observation_v5','rl_policy_tick_v5','fc_v4_round_sat']
                files=[guard,*[str(source/'control'/(n+'.v')) for n in names],
                       str(source/'fc_v4_top.v'),str(source/'fixed4500_weight_rom.v'),str(HERE/'sim/tb_policy.sv')]
                shutil.copytree(source/'src',folder/'src')
            elif profile in ('core_elaboration','core'):
                files=manifest['verilog_files']
                shutil.copytree(source/'src',folder/'src')
                # Vivado rejects one late declaration in the unchanged upstream OLED.
                # Move that exact declaration in a simulation copy, preserving its bytes/logic.
                oled=source/'control/oled.v'
                text=oled.read_text(encoding='utf8')
                statement=('    wire [2:0] axis_index=segment-3\'d1;\n'
                           '    wire [15:0] current_value=frame_values[axis_index*16 +: 16];\n')
                assert text.count(statement)==1,'Expected upstream late declarations must be reviewed'
                revised=text.replace(statement,'',1)
                anchor=revised.index('    wire [15:0] magnitude=current_value')
                revised=revised[:anchor]+statement+revised[anchor:]
                sim_oled=folder/'oled_sim_declaration.v';sim_oled.write_text(revised,encoding='utf8')
                (folder/'simulation_only_declaration_move.json').write_text(json.dumps(dict(source=str(oled),declaration=statement.strip(),hardware_source_changed=False),indent=2)+'\n')
                files=[str(sim_oled) if Path(x)==oled else x for x in files]
                core=candidate/'control4500_core.v'
                core_text=core.read_text(encoding='utf8')
                declaration='    reg [2:0] diagnostic_left_motor_fault,diagnostic_right_motor_fault;\n'
                assert core_text.count(declaration)==1
                core_revised=core_text.replace(declaration,'',1)
                anchor=core_revised.index('    wire [959:0] telemetry_payload=')
                core_revised=core_revised[:anchor]+declaration+core_revised[anchor:]
                constants=''.join(s+'\n' for s in core_revised.splitlines() if
                                  s.startswith('    localparam signed [31:0] LEFT_TARGET_MMPS=') or
                                  s.startswith('    localparam signed [31:0] RIGHT_TARGET_MMPS='))
                assert constants.count(';')==2 and core_revised.count(constants)==1
                core_revised=core_revised.replace(constants,'',1)
                anchor=core_revised.index('    wire signed [31:0] model_left_mmps,model_right_mmps;')
                core_revised=core_revised[:anchor]+constants+core_revised[anchor:]
                sim_core=folder/'control4500_core_sim_declaration.v'
                sim_core.write_text(core_revised,encoding='utf8')
                (folder/'core_simulation_only_declaration_move.json').write_text(json.dumps(dict(source=str(core),declaration=declaration.strip(),constants=constants.strip(),hardware_source_changed=False),indent=2)+'\n')
                files=[str(sim_core) if Path(x)==core else x for x in files]
                if profile=='core': files.append(str(HERE/'sim/tb_core.sv'))
            else: raise ValueError(profile)
            # Freeze testbench bytes in the result folder before the compiler reads them.
            frozen=[]
            for f in files:
                p=Path(f)
                if p.parent==HERE/'sim':
                    dest=folder/p.name;shutil.copyfile(p,dest);f=str(dest)
                frozen.append(f)
            files=frozen
            (folder/'inputs.json').write_text(json.dumps(files,indent=2)+'\n')
            top='control4500_core' if profile=='core_elaboration' else 'tb_'+profile
            row=dict(name=profile,output=str(folder),commands=[],passed=False)
            results.append(row)
            tasks=[('xvlog',['-sv','-i',str(source/'nav/rtl/regs'),*files]),
                   ('xelab',['-debug','off','-timescale','1ns/1ps',top,'-s','probe'])]
            if profile!='core_elaboration': tasks.append(('xsim',['probe','-runall']))
            for tool,args in tasks:
                cmd=['cmd','/c',str(VIV/(tool+'.bat')),*args]
                save('running_'+profile+'_'+tool)
                started=time.time()
                with (folder/(tool+'_stdout.log')).open('w',encoding='utf8') as log:
                    proc=subprocess.Popen(cmd,cwd=folder,stdout=log,stderr=subprocess.STDOUT,
                                          creationflags=subprocess.CREATE_NO_WINDOW)
                    rt=dict(command=cmd,pid=proc.pid,started_at=datetime.now().astimezone().isoformat())
                    row['commands'].append(rt);save('running_'+profile+'_'+tool)
                    code=proc.wait()
                rt.update(exit_code=code,elapsed_seconds=round(time.time()-started,3))
                assert code==0,(profile,tool,code,str(folder))
            if profile=='core_elaboration': row['marker']='FULL_CURRENT_CONTROL_CORE_ELABORATED'
            else:
                log=(folder/'xsim.log').read_text(encoding='utf8',errors='replace')
                marker='TEST PASSED '+profile.upper()
                assert marker in log and 'Fatal:' not in log,(profile,log[-3000:])
                row['marker']=next(s.strip() for s in log.splitlines() if marker in s)
            row['passed']=True;save('profile_complete')
            print(json.dumps({k:v for k,v in row.items() if k!='commands'}),flush=True)
        state['passed']=True;save('complete')
    except Exception as exc:
        state.update(passed=False,error=str(exc));save('failed_preserved')
        print(json.dumps(state,indent=2),flush=True)
    (out/'result.json').write_text(json.dumps(state,indent=2)+'\n',encoding='utf8')
    return 0 if state.get('passed') else 1

if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--candidate',type=Path,required=True)
    p.add_argument('--out',type=Path,required=True)
    p.add_argument('--profiles',nargs='+',default=['guard','range','nec','sensor_guard','policy','led'])
    a=p.parse_args();sys.exit(run(a.candidate,a.out,a.profiles))
