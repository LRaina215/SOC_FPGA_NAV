"""Reproduce baked/live firmware bytes, then audit their actual linked stacks.

Independent FW copies only; preserve both original images and the FPGA. The
old copied stack JSON belongs to a different ELF and is not accepted here.
"""
from datetime import datetime
import json
from pathlib import Path
import re
import shutil
import subprocess
import sys

from review_slf128_linked_stack import review

BASE=Path('F:/nav_integration_g2_20261023')
SOURCE=BASE/'K_timing_syn49'
OUT=BASE/'PNR61_exact_firmware_stack_review1'
GCC=Path('D:/tools/xpack-riscv-none-elf-gcc-15.2.0-1/bin')


def main():
    assert not OUT.exists(),'Read existing result instead of recompiling'
    OUT.mkdir()
    results=[]
    profiles=[('baked1500',1500,SOURCE/'firmware/nav_fw.bin'),
              ('loaded1000',1000,Path('D:/tools/nav_fw_g3_fwd1000_20261027.bin'))]
    for label,forward,target in profiles:
        folder=OUT/label;folder.mkdir()
        fw=folder/'fw'
        shutil.copytree(SOURCE/'fw',fw,ignore=shutil.ignore_patterns('build','__pycache__','*.pyc'))
        img=target.read_bytes()
        stamp=re.search(rb'nav fw (.{11}) (.{8})',img)
        assert stamp
        mainc=fw/'app/main.c';s=mainc.read_text()
        assert s.count('__DATE__')==s.count('__TIME__')==1
        # Restore the two observed build-banner literals. This is reproducible
        # diagnostic source binding, not a change to application behavior.
        s=s.replace('__DATE__',json.dumps(stamp[1].decode('ascii'))).replace('__TIME__',json.dumps(stamp[2].decode('ascii')))
        mainc.write_text(s,encoding='utf8')
        header=fw/'app/nav_app.h';s=header.read_text()
        s,n=re.subn(r'(#define APP_LF_FORWARD_MM\s+)1000\b',lambda m:m[1]+str(forward),s)
        assert n==1;header.write_text(s,encoding='utf8')
        build=folder/'firmware'
        with (folder/'compile.log').open('w',encoding='utf8') as f:
            r=subprocess.run([sys.executable,'-B','-X','utf8',str(fw/'bsp/build_rv_k.py'),
                '--ram-kb','128','--stack-kb','6','--stack-usage','--output-dir',str(build)],
                cwd=SOURCE/'nav',stdout=f,stderr=subprocess.STDOUT,creationflags=subprocess.CREATE_NO_WINDOW)
        assert r.returncode==0,('Build failed; original image unchanged',label)
        new=(build/'nav_fw.bin').read_bytes()
        assert new==img,('Not the exact selected image; do not transfer stack proof',label,len(new),len(img))
        checked=folder/'strict';checked.mkdir()
        strict=review(build,SOURCE/'nav/rtl/soc/nav_soc.v',checked,GCC/'riscv-none-elf-objdump.exe',3072)
        (checked/'result.json').write_text(json.dumps(strict,indent=2),encoding='utf8')
        assert strict['passed'] and strict['complete']
        results.append(dict(profile=label,target=str(target),BIN_bytes=len(img),whole_BIN_byte_identical=True,
            exact_banner_date=stamp[1].decode(),exact_banner_time=stamp[2].decode(),forward_mm=forward,
            actual_static_bound_bytes=strict['actual_static_bound_bytes'],stack_reserved_bytes=strict['stack_reserved_bytes'],
            actual_jr_count=strict['actual_jr_count'],actual_jalr_count=strict['actual_jalr_count'],
            full_stack_result=str(checked/'result.json')))
    result=dict(recorded_at=datetime.now().astimezone().isoformat(),passed=True,profiles=results,
        copied_old_stack_report_rejected=True,hashes_computed=False,hardware_access=False,programmed=False,
        original_images_unchanged=True,board_stack_watermark_or_busy_deadline_accepted=False)
    (OUT/'result.json').write_text(json.dumps(result,indent=2),encoding='utf8')
    print(json.dumps(result))


if __name__=='__main__':main()
