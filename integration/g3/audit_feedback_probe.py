"""Audit actual B1 implementation before a separately authorized SRAM session."""
import argparse,json,re,sys
from pathlib import Path
ROOT=Path(__file__).resolve().parents[2]
sys.path.insert(0,str(ROOT/'integration/g2'))
from audit_pin_loads import CELLS,PORTS
from audit_netlist_memories import BSRAM
PINS=dict(clk='W19',reset_n='D21',uart_txd='M21',BMI088_SDA='F18',BMI088_SCL='E18',
    M1_ENCA='D14',M1_ENCB='F13',M2_ENCA='D15',M2_ENCB='F14',M1_PWM='D17',M1_DIR1='B20',M1_DIR2='B17',
    M2_PWM='C17',M2_DIR1='B18',M2_DIR2='A20',SERVO1='C18',SERVO2='C19',SERVO3='E19',SERVO4='D19')
ACTUATORS=('M1_PWM','M1_DIR1','M1_DIR2','M2_PWM','M2_DIR1','M2_DIR2','SERVO1','SERVO2','SERVO3','SERVO4')


def audit(build):
    build=Path(build);p=build/'impl/pnr'
    report=(p/'feedback_probe.rpt.txt').read_text();timing=(p/'feedback_probe.tr').read_text()
    netlist=(p/'feedback_probe.vo').read_text();log=(build/'gw_sh.log').read_text()
    cells=[(kind,name,dict(PORTS.findall(body)))for kind,name,body in CELLS.findall(netlist)]
    grounds={ports['G']for kind,name,ports in cells if kind=='GND'}
    outputs={ports['O']:ports['I']for kind,name,ports in cells if kind=='OBUF'}
    table=report.split('7. Pinout by Port Name',1)[1].split('8. All Package Pins',1)[0]
    pins={}
    for line in table.splitlines():
        fields=[s.strip()for s in line.split('|')]
        if len(fields)>4 and fields[0]in PINS:pins[fields[0]]=fields[2].split('/')[0]
    constraints=(build/'feedback_probe.sdc').read_text()
    exceptions=re.findall(r'^\s*set_(?:false_path|clock_groups|multicycle_path|max_delay|min_delay)\b.*',constraints,re.M)
    violations={kind:int(re.search(r'<Numbers of '+kind.capitalize()+r' Violated Endpoints>:\s*(\d+)',timing)[1])for kind in('setup','hold')}
    clock=re.search(r'^\s*1\s+core_clk\s+([\d.]+)\(MHz\)\s+([\d.]+)\(MHz\)',timing,re.M)
    secondary={}
    for key,title,stop in [('recovery','3.1.3 Recovery Paths Table','3.1.4 Removal Paths Table'),('removal','3.1.4 Removal Paths Table','3.2 Minimum Pulse Width Table'),('pulse','3.2 Minimum Pulse Width Table','3.3 Timing Report By Analysis Type')]:
        begin=re.search(r'^'+re.escape(title)+r'\s*$',timing,re.M)
        section=timing[begin.end():]if begin else ''
        end=re.search(r'^'+re.escape(stop)+r'\s*$',section,re.M)
        if end:section=section[:end.start()]
        slacks=[float(v)for v in re.findall(r'^\s*\d+\s+(-?[\d.]+)\s+',section,re.M)]
        secondary[key]=min(slacks)if slacks else None
    errors=re.findall(r'^.*\bERROR\s*\(.*$',log,re.M)
    result=dict(build=str(build.resolve()),constraint_mhz=float(clock[1])if clock else None,actual_fmax_mhz=float(clock[2])if clock else None,
        violations=violations,worst_reported_secondary_slacks_ns=secondary,physical_pins=pins,pins_match=pins==PINS,
        actuator_obuf_inputs={name:outputs.get(name)for name in ACTUATORS},all_ten_actuators_physical_ground=all(outputs.get(name)in grounds for name in ACTUATORS),
        bsram_primitives=sum(kind in BSRAM for kind,name,ports in cells),errors=errors,timing_exceptions=exceptions,
        bitstream_present=(p/'feedback_probe.fs').is_file(),device_c_report='<Device Version>: C'in report,programmed=False,
        scope='independent B1 diagnostic only; no joint G2, original control deployment, calibrated IMU or physical-stop acceptance')
    result['diagnostic_build_gate_pass']=bool(clock and float(clock[1])==50 and float(clock[2])>=50 and not any(violations.values())and
        all(v is not None and v>=0 for v in secondary.values())and result['pins_match']and result['all_ten_actuators_physical_ground']and
        result['bitstream_present']and result['device_c_report']and not errors and not exceptions and 'Placement and routing completed'in log)
    return result


def main():
    p=argparse.ArgumentParser();p.add_argument('build',type=Path);p.add_argument('--out',type=Path,required=True);a=p.parse_args()
    result=audit(a.build);a.out.write_text(json.dumps(result,indent=2),encoding='utf-8');print(json.dumps(result));return 0 if result['diagnostic_build_gate_pass']else 2


if __name__=='__main__':raise SystemExit(main())
