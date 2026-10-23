"""Reproduce ACX750 SFP1 SerDes/CSR and use the matching GUI-generated MAC.

The MAC export must come from Gowin V1.9.12.04 for GW5AT-138C PG484.
No programming is performed. The output directory must be new.
"""
import argparse
import json
from pathlib import Path
import re
import shutil
import subprocess
import time
import tomllib

HERE = Path(__file__).resolve().parent
DEFAULT_IDE = Path("D:/Gowin2/Gowin_V1.9.12.04_x64/IDE")
DEFAULT_CACHE = Path("D:/tools/nav_sfp_ip_20261023")
REF_URL = "https://www.gowinsemi.com/upload/database_doc/2505/document/69558f8490732.zip"


def generate(out, ide=DEFAULT_IDE, mac_export=DEFAULT_CACHE):
    out = Path(out).resolve()
    out.mkdir(parents=True, exist_ok=False)
    result = dict(device="GW5AT-LV138PG484AC1/I0", version="C", lane="Q0 Lane0",
                  refclock="Q0 REFCLK1 125MHz", line_gbps=1.25, loopback="OFF",
                  source_url=REF_URL, mac_export=str(mac_export), generated=False,
                  programmed=False)
    start = time.monotonic()
    try:
        cfg = tomllib.loads((HERE / "config/serdes_tmp.toml").read_text(encoding="utf-8"))
        enabled = [f"q{q}.ln{n}" for q in range(2) for n in range(4)
                   if cfg[f"q{q}"][f"ln{n}"]["enable"]]
        assert cfg["device"] == "GW5AT-138" and enabled == ["q0.ln0"]
        assert cfg["q0"]["ref_pad1_freq"] == "125M"
        assert cfg["q0"]["ln0"]["loopBack"] == "OFF"
        for name in ("serdes_tmp.toml", "serdes.ipc"):
            shutil.copyfile(HERE / "config" / name, out / name)
        mod = (HERE / "config/serdes.mod").read_text(encoding="utf-8")
        mod, count = re.subn(r"(?m)^-path .*", f"-path {out.as_posix()}/", mod)
        assert count == 1
        (out / "serdes.mod").write_text(mod, encoding="utf-8")
        template = (mac_export / "giga_serial_ethernet_tmp.v").read_text(encoding="utf-8")
        assert "GW5AT-LV138PG484AC1/I0" in template and "Device Version: C" in template
        for name in ("giga_serial_ethernet.v", "giga_serial_ethernet_tmp.v"):
            shutil.copyfile(mac_export / name, out / name)
        commands = [
            [str(ide / "bin/serdes_toml_to_csr_138k.exe"), str(out / "serdes_tmp.toml"),
             "-o", str(out / "serdes.csr")],
            [str(ide / "bin/GowinModGen.exe"), "-do", str(out / "serdes.mod")],
        ]
        result["commands"] = commands
        with (out / "generate.log").open("wb") as log:
            for command in commands:
                completed = subprocess.run(command, cwd=out, stdout=log, stderr=subprocess.STDOUT,
                                           timeout=120, creationflags=getattr(subprocess, "CREATE_NO_WINDOW", 0))
                if completed.returncode:
                    raise RuntimeError(f"Generator exit {completed.returncode}; see generate.log")
        # Expose existing status wires without changing the vendor PMA/PCS logic.
        source = (out / "serdes.v").read_text(encoding="utf-8")
        signals = dict(diag_qpll_ok="q0_fabric_cmu1_ok_o",
                       diag_rx_lock="q0_fabric_ln0_pma_rx_lock_o",
                       diag_align="q0_lane0_align_link")
        for signal in signals.values():
            assert re.search(r"\bwire\s+" + signal + r"\s*;", source), signal
        source, count = re.subn(r"(module SfpSerDes\s*\()", r"\1" + ", ".join(signals) + ", ", source)
        assert count == 1
        end = source.index(";", source.index("module SfpSerDes")) + 1
        diagnostics = "\n" + "\n".join(f"output {p};\nassign {p} = {s};" for p, s in signals.items()) + "\n"
        source = source[:end] + diagnostics + source[end:]
        (out / "serdes.v").write_text(source, encoding="utf-8")
        result["generated"] = True
        result["scope"] = "Generator completed; not proof of timing closure or physical link"
        return out
    except Exception as error:
        result["error"] = str(error)
        raise
    finally:
        result["elapsed_s"] = round(time.monotonic() - start, 2)
        (out / "generation_result.json").write_text(json.dumps(result, indent=2), encoding="utf-8")
        print(json.dumps(result, indent=2))


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--out", type=Path, required=True)
    parser.add_argument("--ide", type=Path, default=DEFAULT_IDE)
    parser.add_argument("--mac-export", type=Path, default=DEFAULT_CACHE)
    args = parser.parse_args()
    generate(args.out, args.ide, args.mac_export)
