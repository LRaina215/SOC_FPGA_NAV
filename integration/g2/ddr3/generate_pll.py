"""Current-C PLL from factory DDR 50->400 MHz settings, no PNR/programming."""
import argparse
from datetime import datetime
import json
from pathlib import Path
import re
import subprocess

MOD='''-series GW5AT
-device GW5AT-138
-device_version C
-package PBGA484A
-part_number GW5AT-LV138PG484AC1/I0
-mod_name Nav_DDR3_PLL
-file_name nav_ddr3_pll
-path OUT_PATH/
-type PLL_ADV
-file_type vlg
-ssc false
-clock_en true
-rst true
-rst_pwd false
-rst_i false
-rst_o false
-fclkin 50
-dyn_idiv_sel false
-idiv_sel 1
-clkfb_sel 0
-dyn_fbdiv_sel false
-fbdiv_sel 1
-en_lock true
-dyn_dpa_en false
-clkout0_bypass false
-dyn_odiv0_sel false
-odiv0_sel 2
-odiv0_frac_sel 0
-dyn_dt0_sel false
-clkout0_dt_dir 1
-clkout0_dt_step 0
-dyn_pe0_sel false
-clkout0_pe_coarse 0
-clkout0_pe_fine 0
-de0_en false
-en_clkout1 true
-clkout1_bypass false
-dyn_odiv1_sel false
-odiv1_sel 16
-dyn_dt1_sel false
-clkout1_dt_dir 1
-clkout1_dt_step 0
-dyn_pe1_sel false
-clkout1_pe_coarse 0
-clkout1_pe_fine 0
-de1_en false
-en_clkout2 true
-clkout2_bypass false
-dyn_odiv2_sel false
-odiv2_sel 2
-dyn_dt2_sel false
-clkout2_dt_dir 1
-clkout2_dt_step 0
-dyn_pe2_sel false
-clkout2_pe_coarse 0
-clkout2_pe_fine 0
-de2_en false
-en_clkout3 false
-en_clkout4 false
-en_clkout5 false
-en_clkout6 false
-en_clkfbout false
-dyn_mdiv_sel false
-mdiv_sel 16
-mdiv_frac_sel 0
'''


def inspect(out):
    assert 'Module Generated Successfully' in (out/'generate.log').read_text(errors='replace')
    s=(out/'nav_ddr3_pll.v').read_text(errors='replace')
    assert 'Device Version: C' in s and 'GW5AT-LV138PG484AC1/I0' in s
    for name,value in [('IDIV_SEL','1'),('FBDIV_SEL','1'),('MDIV_SEL','16'),('ODIV2_SEL','2')]:
        assert re.search(r'defparam\s+PLL_inst\.'+name+r'\s*=\s*'+value+r'\s*;',s),name
    r=dict(generated_at=datetime.now().astimezone().isoformat(),tool_exit=0,
        device='GW5AT-LV138PG484AC1/I0',device_version='C',input_mhz=50,
        requested_clkout2_mhz=400,physical_clock_verified=False,programmed=False)
    (out/'result.json').write_text(json.dumps(r,indent=2),encoding='utf-8');return r


def generate(out):
    assert not out.exists();out.mkdir(parents=True)
    (out/'nav_ddr3_pll.mod').write_text(MOD.replace('OUT_PATH',out.as_posix()))
    with (out/'generate.log').open('w') as log:
        p=subprocess.run(['D:/Gowin2/Gowin_V1.9.12.04_x64/IDE/bin/GowinModGen.exe',
            '-do',str(out/'nav_ddr3_pll.mod')],cwd=out,stdout=log,stderr=subprocess.STDOUT,
            timeout=60,creationflags=subprocess.CREATE_NO_WINDOW)
    assert p.returncode==0,(out/'generate.log').read_text(errors='replace')
    return inspect(out)


if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--out',type=Path,required=True)
    p.add_argument('--inspect',action='store_true');a=p.parse_args()
    print(json.dumps(inspect(a.out.resolve()) if a.inspect else generate(a.out.resolve()),indent=2))
