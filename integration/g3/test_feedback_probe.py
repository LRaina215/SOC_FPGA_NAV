"""Check the B1 wire format against original encoder/UART RTL and synthetic BMI."""
import argparse,json,subprocess
from pathlib import Path
from read_feedback_probe import Decoder,crc16
ROOT=Path(__file__).resolve().parents[2]


def main():
    p=argparse.ArgumentParser();p.add_argument('--out',type=Path,required=True);a=p.parse_args()
    out=a.out.resolve();out.mkdir(parents=True,exist_ok=False)
    viv=Path('E:/2026.1/Vivado/bin')
    def run(tool,args):
        with(out/(tool+'_stdout.log')).open('w')as log:
            subprocess.run(['cmd','/c',str(viv/(tool+'.bat')),*args],cwd=out,stdout=log,stderr=subprocess.STDOUT,check=True,timeout=120)
    files=[ROOT/'integration/g3'/n for n in ('feedback_probe_top.v','i2c_id_observer.v')]
    files += [ROOT/'control/fpga/src'/n for n in ('bmi088.v','oled_driver.v','motor520_speed.v','board_uart_tx.v','uart_telemetry.v')]
    run('xvlog',list(map(str,files)))
    run('xvlog',['-sv',str(ROOT/'integration/g3/tb_feedback_probe.sv')])
    run('xelab',['-debug','typical','-timescale','1ns/1ps','tb_feedback_probe','-s','snap'])
    run('xsim',['snap','-runall'])
    log=(out/'xsim.log').read_text(errors='replace');assert 'FEEDBACK_PROBE_WIRE_PASS' in log and 'Fatal:' not in log,log[-2500:]
    raw=bytes(int(x,16)for x in(out/'feedback_uart_bytes.hex').read_text().split());assert len(raw)==378
    (out/'rx.bin').write_bytes(raw);d=Decoder();rows=[]
    for i in range(0,len(raw),7):rows.extend(d.feed(raw[i:i+7]))
    assert len(rows)==3 and d.bad_crc==0,rows
    assert [r['sequence']for r in rows]==[0,1,2],rows
    assert [r['timestamp_ms']for r in rows]==[9,19,29],rows
    assert [r['encoder_sample_sequence']for r in rows]==[0,1,2],rows
    assert [(r['left_delta'],r['right_delta'])for r in rows]==[(0,0),(4,-4),(0,0)],rows
    assert [(r['left_position'],r['right_position'])for r in rows]==[(0,0),(4,-4),(4,-4)],rows
    for r in rows:
        assert [r[k]for k in ('ax_raw','ay_raw','az_raw','gx_raw','gy_raw','gz_raw')]==[-32768,32767,10920,-123,456,-789],r
        assert r['bmi_sample_sequence']==1 and r['bmi_initialized_sample'] and r['bmi_fresh_50ms'],r
        assert r['ack_69']==2 and r['nack_68']==1 and r['id_reads_69']==1 and r['last_id_69']==0xa2 and r['id_seen']==8,r
        assert r['left_invalid']==r['right_invalid']==0 and r['encoder_flags']in (0,1),r
    assert rows[0]['encoder_age_ms']==65535 and all(0<=r['encoder_age_ms']<11 for r in rows[1:]),rows
    bad=bytearray(raw[:126]);bad[20]^=1;recover=Decoder()
    assert len(recover.feed(b'junk'+bad+raw[126:]))==2 and recover.bad_crc==1
    stale=bytearray(raw[:126]);stale[4+18:4+20]=(60).to_bytes(2,'little');stale[-2:]=crc16(stale[2:-2]).to_bytes(2,'little')
    stale_row=Decoder().feed(stale)[0];assert stale_row['bmi_initialized_sample'] and not stale_row['bmi_fresh_50ms']
    result=dict(passed=True,build=str(out),wire_frames=3,wire_bytes=len(raw),rows=rows,cases=['raw M2 left/M1 right quadrature signs','publication sequence and age','atomic six-axis hold','per-address I2C ACK/NACK and wrong ID','UART CRC and recovery','stale age separate from sample presence','constant zero actuator outputs'],scope='scaled RTL wire contract; synthetic BMI publication and bus, no real sensor, timing or stop acceptance',programmed=False)
    (out/'result.json').write_text(json.dumps(result,indent=2),encoding='utf-8');print(json.dumps({k:v for k,v in result.items()if k!='rows'}))


if __name__=='__main__':main()
