"""从 config/*.yaml 生成 Verilog / C / Python / Markdown 常量文件。

用法: python tools/gen_config.py
生成的文件带 "自动生成，勿手改" 标记，改参数请改 yaml。
"""
from pathlib import Path
import yaml

ROOT = Path(__file__).resolve().parents[1]
HDR = "自动生成，勿手改。来源: config/{src}，生成器: tools/gen_config.py"


class _Loader(yaml.SafeLoader):
    """YAML 1.1 会把 off/on/yes/no 解析成布尔，这里只保留 true/false。"""


_Loader.yaml_implicit_resolvers = {
    k: [(tag, rx) for tag, rx in v if tag != "tag:yaml.org,2002:bool"]
    for k, v in yaml.SafeLoader.yaml_implicit_resolvers.items()
}
_Loader.add_implicit_resolver(
    "tag:yaml.org,2002:bool",
    __import__("re").compile(r"^(?:true|True|TRUE|false|False|FALSE)$"),
    list("tTfF"),
)


def load(name):
    with open(ROOT / "config" / name, encoding="utf-8") as f:
        return yaml.load(f, Loader=_Loader)


def flat_params(p):
    out = []
    for group, items in p.items():
        for k, v in items.items():
            out.append((group, k, v))
    return out


def gen_params(p):
    rows = flat_params(p)
    derived = {
        "MAP_N": 1 << p["map"]["MAP_LOG2N"],
        "MAP_CELLS": 1 << (2 * p["map"]["MAP_LOG2N"]),
        "LIDAR_PIX": p["lidar"]["LIDAR_W"] * p["lidar"]["LIDAR_H"],
    }
    vh = [f"// {HDR.format(src='nav_params.yaml')}", "`ifndef NAV_PARAMS_VH", "`define NAV_PARAMS_VH"]
    for g, k, v in rows:
        vh.append(f"localparam integer NP_{k} = {v};  // {g}")
    for k, v in derived.items():
        vh.append(f"localparam integer NP_{k} = {v};  // 派生")
    vh.append("`endif")
    (ROOT / "rtl/regs/nav_params.vh").write_text("\n".join(vh) + "\n", encoding="utf-8")

    h = [f"/* {HDR.format(src='nav_params.yaml')} */", "#ifndef NAV_PARAMS_H", "#define NAV_PARAMS_H"]
    for g, k, v in rows:
        h.append(f"#define NP_{k} ({v})")
    for k, v in derived.items():
        h.append(f"#define NP_{k} ({v})")
    h.append("#endif")
    (ROOT / "fw/include/nav_params.h").write_text("\n".join(h) + "\n", encoding="utf-8")

    py = [f"# {HDR.format(src='nav_params.yaml')}"]
    for g, k, v in rows:
        py.append(f"{k} = {v}")
    for k, v in derived.items():
        py.append(f"{k} = {v}")
    (ROOT / "model/nav_params.py").write_text("\n".join(py) + "\n", encoding="utf-8")


def field_mask(msb, lsb):
    return ((1 << (msb - lsb + 1)) - 1) << lsb


def md_cell(s):
    """说明文字里的 | 会被 Markdown 当成列分隔符"""
    return str(s).replace("|", "\\|")


def gen_regmap(r):
    vh = [f"// {HDR.format(src='regmap.yaml')}", "`ifndef NAV_REGS_MAP_VH", "`define NAV_REGS_MAP_VH"]
    h = [f"/* {HDR.format(src='regmap.yaml')} */", "#ifndef NAV_REGS_MAP_H", "#define NAV_REGS_MAP_H"]
    py = [f"# {HDR.format(src='regmap.yaml')}", "REGS = {}", "RESET = {}", "FIELDS = {}"]
    md = [f"<!-- {HDR.format(src='regmap.yaml')} -->", "# 寄存器表", "",
          "地址为相对导航子系统基址的字节偏移；总线 32 位，按字访问。", ""]
    seen = {}
    for b in r["blocks"]:
        bn, base = b["name"], b["base"]
        vh.append(f"localparam [15:0] RB_{bn} = 16'h{base:04X};")
        h.append(f"#define NAV_{bn}_BASE 0x{base:04X}u")
        md += [f"## {bn}（0x{base:04X}）{b['desc']}", "", "| 偏移 | 名称 | 访问 | 复位值 | 说明 |", "|---|---|---|---|---|"]
        for reg in b["regs"]:
            name = f"{bn}_{reg['name']}"
            addr = base + reg["off"]
            if addr in seen:
                raise SystemExit(f"地址冲突: {name} 与 {seen[addr]} @0x{addr:04X}")
            seen[addr] = name
            rst = reg.get("reset", 0)
            vh.append(f"localparam [15:0] RA_{name} = 16'h{addr:04X};")
            vh.append(f"localparam [31:0] RR_{name} = 32'h{rst & 0xFFFFFFFF:08X};")
            h.append(f"#define NAV_{name} 0x{addr:04X}u")
            h.append(f"#define NAV_{name}_RESET 0x{rst & 0xFFFFFFFF:08X}u")
            py.append(f"REGS['{name}'] = 0x{addr:04X}")
            py.append(f"RESET['{name}'] = 0x{rst & 0xFFFFFFFF:08X}")
            fdesc = []
            for fn, (msb, lsb) in (reg.get("fields") or {}).items():
                vh.append(f"localparam integer RF_{name}_{fn}_LSB = {lsb};")
                vh.append(f"localparam integer RF_{name}_{fn}_MSB = {msb};")
                h.append(f"#define NAV_{name}_{fn}_SHIFT {lsb}")
                h.append(f"#define NAV_{name}_{fn}_MASK 0x{field_mask(msb, lsb):08X}u")
                py.append(f"FIELDS['{name}_{fn}'] = ({msb}, {lsb})")
                fdesc.append(f"{fn}[{msb}:{lsb}]" if msb != lsb else f"{fn}[{lsb}]")
            desc = reg.get("desc", "")
            if fdesc:
                desc = (desc + "；" if desc else "") + " ".join(fdesc)
            md.append(f"| 0x{addr:04X} | {reg['name']} | {reg['acc']} | 0x{rst & 0xFFFFFFFF:08X} | {md_cell(desc)} |")
        md.append("")
    md += ["## 数据窗口", "", "| 基址 | 大小 | 名称 | 说明 |", "|---|---|---|---|"]
    for w in r["windows"]:
        vh.append(f"localparam [15:0] RW_{w['name']}_BASE = 16'h{w['base']:04X};")
        vh.append(f"localparam [15:0] RW_{w['name']}_SIZE = 16'h{w['size']:04X};")
        h.append(f"#define NAV_WIN_{w['name']}_BASE 0x{w['base']:04X}u")
        h.append(f"#define NAV_WIN_{w['name']}_SIZE 0x{w['size']:04X}u")
        py.append(f"REGS['WIN_{w['name']}'] = 0x{w['base']:04X}")
        md.append(f"| 0x{w['base']:04X} | 0x{w['size']:04X} | {w['name']} | {md_cell(w['desc'])} |")
    vh.append("`endif")
    h.append("#endif")
    (ROOT / "rtl/regs/nav_regs_map.vh").write_text("\n".join(vh) + "\n", encoding="utf-8")
    (ROOT / "fw/include/nav_regs_map.h").write_text("\n".join(h) + "\n", encoding="utf-8")
    (ROOT / "model/nav_regs_map.py").write_text("\n".join(py) + "\n", encoding="utf-8")
    (ROOT / "docs/regmap.md").write_text("\n".join(md) + "\n", encoding="utf-8")


def gen_tables(p):
    """RTL 只读表：正弦表（traj_eval）。雷达方向查找表由固件上电后写入（fw/nav/lidar_cal.c），不再预置。"""
    import math
    import sys
    n = 1 << p["traj"]["SIN_LUT_LOG2"]
    frac = p["traj"]["SIN_FRAC"]
    lines = [f"{int(round(math.sin(2 * math.pi * i / n) * (1 << frac))) & 0xFFFF:04x}" for i in range(n)]
    (ROOT / "rtl/plan/sin_lut.hex").write_text("\n".join(lines) + "\n", encoding="utf-8")
    vals = [int(round(math.sin(2 * math.pi * i / n) * (1 << frac))) for i in range(n)]
    c = [f"/* {HDR.format(src='nav_params.yaml')} */", "#ifndef NAV_SIN_LUT_H", "#define NAV_SIN_LUT_H",
         "#include <stdint.h>", f"/* sin(2*pi*i/{n}) * 2^{frac}，与 rtl/plan/sin_lut.hex 相同 */",
         f"static const int16_t NAV_SIN_LUT[{n}] = {{"]
    for i in range(0, n, 16):
        c.append("    " + ", ".join(str(v) for v in vals[i:i + 16]) + ",")
    c += ["};", "#endif"]
    (ROOT / "fw/include/nav_sin_lut.h").write_text("\n".join(c) + "\n", encoding="utf-8")


def gen_regfile(r):
    """生成寄存器文件 rtl/regs/nav_regfile.v：
      rw  -> output reg  r_<N>（带复位值）
      ro  -> input       i_<N>
      rop -> input i_<N>，读取时 output p_<N> 单拍（读后弹出）
      wo/w1p -> output p_<N> 单拍（写入时），写数据取 wdata_q
      w1c -> 内部状态 r_<N>，input s_<N> 置位，写 1 清零
    读数据 rdata 组合输出（按 addr 选择）；hit 表示地址落在寄存器上。
    """
    regs = []
    for b in r["blocks"]:
        for reg in b["regs"]:
            regs.append((f"{b['name']}_{reg['name']}", b["base"] + reg["off"], reg["acc"], reg.get("reset", 0) & 0xFFFFFFFF))
    ports, body_decl, wr, rd, pulse_clr = [], [], [], [], []
    for n, a, acc, rst in regs:
        if acc == "rw":
            ports.append(f"    output reg  [31:0] r_{n},")
            wr.append(f"            16'h{a:04X}: r_{n} <= wdata;")
            rd.append(f"        16'h{a:04X}: rdata = r_{n};")
            pulse_clr.append(f"            r_{n} <= 32'h{rst:08X};")
        elif acc == "ro":
            ports.append(f"    input  wire [31:0] i_{n},")
            rd.append(f"        16'h{a:04X}: rdata = i_{n};")
        elif acc == "rop":
            ports.append(f"    input  wire [31:0] i_{n},")
            ports.append(f"    output wire        p_{n},")
            rd.append(f"        16'h{a:04X}: rdata = i_{n};")
            body_decl.append(f"    assign p_{n} = rd && (addr == 16'h{a:04X});")
        elif acc in ("wo", "w1p"):
            ports.append(f"    output wire        p_{n},")
            body_decl.append(f"    assign p_{n} = wr && (addr == 16'h{a:04X});")
            rd.append(f"        16'h{a:04X}: rdata = 32'h0;")
        elif acc == "rwe":
            # 写入值 r_ + 写脉冲 p_，读回外部生效值 i_（例如可被硬件自动更新的对端地址）
            ports.append(f"    output reg  [31:0] r_{n},")
            ports.append(f"    output wire        p_{n},")
            ports.append(f"    input  wire [31:0] i_{n},")
            wr.append(f"            16'h{a:04X}: r_{n} <= wdata;")
            rd.append(f"        16'h{a:04X}: rdata = i_{n};")
            pulse_clr.append(f"            r_{n} <= 32'h{rst:08X};")
            body_decl.append(f"    assign p_{n} = wr && (addr == 16'h{a:04X});")
        elif acc == "w1c":
            ports.append(f"    output reg  [31:0] r_{n},")
            ports.append(f"    input  wire [31:0] s_{n},")
            rd.append(f"        16'h{a:04X}: rdata = r_{n};")
            body_decl.append(f"    always @(posedge clk) begin\n"
                             f"        if (rst) r_{n} <= 32'h0;\n"
                             f"        else r_{n} <= (r_{n} & ~((wr && addr == 16'h{a:04X}) ? wdata : 32'h0)) | s_{n};\n"
                             f"    end")
        else:
            raise SystemExit(f"未知访问类型 {acc}")
    v = [f"// {HDR.format(src='regmap.yaml')}",
         "module nav_regfile (",
         "    input  wire        clk,",
         "    input  wire        rst,",
         "    input  wire        wr,          // 单拍写",
         "    input  wire        rd,          // 单拍读（仅用于 rop 弹出）",
         "    input  wire [15:0] addr,",
         "    input  wire [31:0] wdata,",
         "    output reg  [31:0] rdata,",
         "    output reg         hit,"]
    v += ports
    v[-1] = v[-1].rstrip(",")
    v += [");"]
    v += body_decl
    v += ["    always @(posedge clk) begin", "        if (rst) begin"] + pulse_clr + \
         ["        end else if (wr) begin", "            case (addr)"] + wr + \
         ["            default: ;", "            endcase", "        end", "    end"]
    v += ["    always @(*) begin", "        hit = 1'b1;", "        case (addr)"] + rd + \
         ["        default: begin rdata = 32'h0; hit = 1'b0; end", "        endcase", "    end", "endmodule"]
    (ROOT / "rtl/regs/nav_regfile.v").write_text("\n".join(v) + "\n", encoding="utf-8")

    # 顶层用的线网声明 + 实例（nav_subsystem 中 `include）
    inc = [f"// {HDR.format(src='regmap.yaml')}"]
    conns = []
    for n, a, acc, rst in regs:
        if acc in ("rw", "w1c", "rwe"):
            inc.append(f"    wire [31:0] r_{n};")
            conns.append(f"        .r_{n}(r_{n}),")
        if acc in ("ro", "rop", "rwe"):
            inc.append(f"    wire [31:0] i_{n};")
            conns.append(f"        .i_{n}(i_{n}),")
        if acc in ("rop", "wo", "w1p", "rwe"):
            inc.append(f"    wire        p_{n};")
            conns.append(f"        .p_{n}(p_{n}),")
        if acc == "w1c":
            inc.append(f"    wire [31:0] s_{n};")
            conns.append(f"        .s_{n}(s_{n}),")
    conns[-1] = conns[-1].rstrip(",")
    inc += ["    nav_regfile u_rf (",
            "        .clk(clk), .rst(rst), .wr(rf_wr), .rd(rf_rd), .addr(a_q), .wdata(wd_q),",
            "        .rdata(rf_rdata), .hit(rf_hit),"] + conns + ["    );"]
    (ROOT / "rtl/regs/nav_subsystem_rf.vh").write_text("\n".join(inc) + "\n", encoding="utf-8")


if __name__ == "__main__":
    gen_params(load("nav_params.yaml"))
    gen_regmap(load("regmap.yaml"))
    gen_regfile(load("regmap.yaml"))
    gen_tables(load("nav_params.yaml"))
    print("gen_config: ok")
