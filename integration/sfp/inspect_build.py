"""Summarize actual Gowin PNR reports and check the diagnostic image contract.

Checks clocks and reported constrained-path timing. Does not certify physical
CDC/Gray-bus skew or a real SFP link. No hashes and no hardware access.
"""
import argparse
from html.parser import HTMLParser
import json
from pathlib import Path
import re


class Tables(HTMLParser):
    def __init__(self):
        super().__init__(); self.tables=[]; self.table=None; self.row=None; self.cell=None
    def handle_starttag(self,tag,attrs):
        if tag=="table": self.table=[]
        elif tag=="tr": self.row=[]
        elif tag in ("td","th"): self.cell=""
    def handle_data(self,data):
        if self.cell is not None: self.cell+=data
    def handle_endtag(self,tag):
        if tag in ("td","th") and self.cell is not None:
            if self.row is not None: self.row.append(" ".join(self.cell.split()))
            self.cell=None
        elif tag=="tr" and self.row is not None:
            if self.table is not None: self.table.append(self.row)
            self.row=None
        elif tag=="table" and self.table is not None:
            self.tables.append(self.table); self.table=None


def inspect(out):
    out=Path(out); pnr=out/"impl/pnr"; p=Tables()
    p.feed((pnr/"sfp_bringup_tr_content.html").read_text(encoding="utf-8"))
    def find(header): return next(t for t in p.tables if t and header in t[0])
    number=lambda s: float(re.match(r"[-+\d.]+",s).group())
    freq={r[1]:dict(required_mhz=number(r[2]),reported_fmax_mhz=number(r[3]))
          for r in find("Actual Fmax")[1:]}
    expected={"core_clk":50,"rx_mac_clk":125,"tx_mac_clk":125}
    assert set(freq)==set(expected),freq
    for name,need in expected.items():
        assert freq[name]["required_mhz"]==need and freq[name]["reported_fmax_mhz"]>=need
    tns=find("Endpoints TNS")[1:]
    assert len(tns)==6 and all(number(r[2])==0 and int(r[3])==0 for r in tns)
    paths=[t for t in p.tables if t and "Path Slack" in t[0]]
    assert len(paths)==4, "Expected setup/hold/recovery/removal tables"
    min_slacks={name:min(number(r[1]) for r in t[1:])
                for name,t in zip(("setup","hold","recovery","removal"),paths)}
    min_slacks["pulse_width"]=min(number(r[1]) for r in find("Actual Width")[1:])
    assert all(n>=0 for n in min_slacks.values()),min_slacks
    net=(pnr/"sfp_bringup.vo").read_text(encoding="utf-8")
    assert re.search(r"GND\s+\w+\s*\(\s*\.G\(GND\)",net), "Check ground primitive"
    outputs=("M1_DIR1","M1_DIR2","M1_PWM","M2_DIR1","M2_DIR2","M2_PWM", "SERVO1","SERVO2","SERVO3","SERVO4")
    for name in outputs:
        assert re.search(r"OBUF\s+"+name+r"_obuf\s*\(\s*\.I\(GND\),\s*\.O\("+name+r"\)",net),name
    cmd=(pnr/"cmd.do").read_text()
    assert "-csr " in cmd and "serdes.csr" in cmd
    assert (pnr/"sfp_bringup.fs").stat().st_size>0
    rpt=(pnr/"sfp_bringup.rpt.txt").read_text()
    resources={name:int(re.search(r"^\s*"+name+r"\s*\|\s*(\d+)/",rpt,re.M).group(1))
               for name in ("Logic","Register","BSRAM","GTR12_QUAD")}
    result=dict(constrained_timing_passed=True,clocks=freq,minimum_reported_slack_ns=min_slacks,
                resources=resources,actuator_outputs_tied_to_ground=list(outputs),csr_passed_to_pnr=True,
                bitstream=str(pnr/"sfp_bringup.fs"),physical_cdc_skew_signoff=False,hardware_tested=False,
                note="Independent-clock paths use vendor-reference exceptions; not joint G2 acceptance")
    (out/"inspection.json").write_text(json.dumps(result,indent=2),encoding="utf-8")
    return result


if __name__=="__main__":
    p=argparse.ArgumentParser(description=__doc__); p.add_argument("out",type=Path)
    print(json.dumps(inspect(p.parse_args().out),indent=2))
