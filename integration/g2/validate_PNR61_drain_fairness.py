"""Rebind bounded-receive schedule tests to current PNR61 firmware C.

HAL queues/time and two planner service hooks are explicit substitutes. Three
new G3/window HAL entry points are counted stubs, not real sensor/map models.
"""
from datetime import datetime
import json
from pathlib import Path

from validate_cpu250_host_drain import execute,NATIVE

ROOT=Path(__file__).resolve().parents[2]
BASE=Path('F:/nav_integration_g2_20261023')
FW=BASE/'K_timing_syn49/fw'
OUT=BASE/'PNR61_current_drain_fairness_native3'


def main():
    assert not OUT.exists();OUT.mkdir()
    original=(ROOT/'integration/g2/test_cpu250_host_drain_schedule.c').read_text()
    s=original.replace('@FW@',FW.as_posix())
    # The older schedule fixture used a v2/40-byte map. Adapt only its public
    # test messages to the current explicit v3 geometry; production C is read-only.
    replacements={
        'expected_header[40]':'expected_header[PRIOR_HDR_LEN]',
        'uint8_t b[48]={0};':'uint8_t b[8+PRIOR_HDR_LEN]={0};',
        "b[8]='N';b[9]='V';b[10]='M';b[11]='P';b[12]=2;":
            "b[8]='N';b[9]='V';b[10]='M';b[11]='P';b[12]=3;\n"
            '    b[14]=7;b[15]=1; /* flags 0x0107 */\n'
            '    b[48]=1;b[50]=0x58;b[51]=0x33; /* profile 0x33580001 */\n'
            '    b[52]=0x5e;b[53]=1;b[54]=0xd4;b[55]=0xfe; /* 350/-300 */',
        'memcpy(expected_header,b+8,40);enqueue(b,48,g_sim.now_us);':
            'memcpy(expected_header,b+8,PRIOR_HDR_LEN);enqueue(b,sizeof b,g_sim.now_us);',
        'nav_app_init();':'nav_app_init();g_app.match_zhi=350;',
        'host_pending_len==48':'host_pending_len==8+PRIOR_HDR_LEN',
        'host_rx_buf+8,40':'host_rx_buf+8,PRIOR_HDR_LEN',
        'b[4]=40;':'b[4]=PRIOR_HDR_LEN;',
        'g_app.prx.next==40':'g_app.prx.next==PRIOR_HDR_LEN',
        'g_app.prx.hdr,40':'g_app.prx.hdr,PRIOR_HDR_LEN',
        '.state=PRIOR_RECV,.total=40':
            '.state=PRIOR_RECV,.total=PRIOR_HDR_LEN,.zhi_expected=350',
    }
    for old,new in replacements.items():
        assert old in s,old
        s=s.replace(old,new)
    extra='''
static unsigned imu_config_calls,lf_center_calls,lf_forward_calls;
void hal_imu_config(uint32_t baud_div){if(baud_div!=434)abort();imu_config_calls++;}
void hal_match_lf_center_mm(int32_t x,int32_t y){(void)x;(void)y;lf_center_calls++;}
void hal_match_lf_forward(int32_t distance_mm,angle_t yaw){(void)distance_mm;(void)yaw;lf_forward_calls++;}
'''
    anchor='static unsigned checks,cases;';assert s.count(anchor)==1
    s=s.replace(anchor,extra+'\n'+anchor)
    (OUT/'schedule.c').write_text(s,encoding='utf8')
    cmd=[NATIVE,'-std=c99','-O2','-Wall','-Wextra','-Wno-unused-parameter','-ffunction-sections','-fdata-sections',
        '-D_USE_MATH_DEFINES','-DGM_N=64','-DASTAR_MAXN=64','-DGM_DS=2','-DAPP_IMU_PROTO=1','-DAPP_IMU_BAUD_DIV=434','-Wl,--gc-sections']
    cmd += ['-I'+str(FW/d) for d in ['include','hal','nav','app','test','bsp']]
    cmd += [OUT/'schedule.c',*sorted((FW/'nav').glob('*.c')),'-o',OUT/'schedule.exe','-lm']
    execute(OUT,'compile',cmd);text=execute(OUT,'actual_current_schedule',[OUT/'schedule.exe'])
    assert 'DRAIN_SCHEDULE passed cases=15 checks=394' in text,text
    result=dict(passed=True,recorded_at=datetime.now().astimezone().isoformat(),source=str(FW),output=text,
        scope='Actual current app/guard/proto/explore C with explicit HAL FIFO/time substitutes and two planner hooks; not CPU timing/full APP WCET',
        additional_counted_stub_functions=['hal_imu_config','hal_match_lf_center_mm','hal_match_lf_forward'],
        previous_harness_missing_G3_symbols=str(BASE/'PNR61_current_drain_fairness_native1/compile.log'),
        previous_legacy_header_fixture=str(BASE/'PNR61_current_drain_fairness_native2/actual_current_schedule.log'),
        fixture_only_map_header_adaptation='v2/40 bytes to current v3/48 bytes, explicit profile/ZHI/ZLO; scheduling assertions retained',
        production_FW_or_RTL_modified=False,hardware_access=False)
    (OUT/'result.json').write_text(json.dumps(result,indent=2),encoding='utf8');print(json.dumps(result))


if __name__=='__main__':main()
