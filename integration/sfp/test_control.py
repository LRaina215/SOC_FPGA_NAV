"""Test the new UART/config/reset top with a MAC stub, without using hardware."""
import argparse
import json
import os
from pathlib import Path
import subprocess
from build import ROOT, HERE, SOURCES


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument("--out",type=Path,required=True)
    p.add_argument("--vivado-bin",type=Path,default=Path(os.environ.get("VIVADO_BIN","E:/2026.1/Vivado/bin")))
    a=p.parse_args(); out=a.out.resolve(); out.mkdir(parents=True,exist_ok=False)
    sources=[str(ROOT/n) for n in SOURCES if not n.endswith("sfp_host_mac.v")]
    commands=[("v",["xvlog"]+sources),
              ("sv",["xvlog","-sv",str(HERE/"tb_sfp_control.sv")]),
              ("elab",["xelab","-debug","off","-timescale","1ns/1ps","tb_sfp_control","-s","snap"]),
              ("sim",["xsim","snap","-R"])]
    for tag,command in commands:
        with (out/(tag+"_stdout.log")).open("wb") as log:
            rc=subprocess.run(["cmd","/c",str(a.vivado_bin/(command[0]+".bat"))]+command[1:],
                              cwd=out,stdout=log,stderr=subprocess.STDOUT,timeout=180).returncode
        if rc: raise RuntimeError(f"{tag} failed: {out}")
    log=(out/"sim_stdout.log").read_text(errors="replace")
    if "TEST PASSED" not in log or any(s in log for s in ("Fatal","FATAL","ERROR:")):
        raise AssertionError(log)
    result=dict(passed=True,scope="UART transactions, top reset/config, LOS/link recovery, actuator constants",
                vendor_mac_stubbed=True,hardware_tested=False)
    (out/"result.json").write_text(json.dumps(result,indent=2),encoding="utf-8")
    print(json.dumps(result,indent=2))


if __name__=="__main__": main()
