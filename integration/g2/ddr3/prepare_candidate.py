"""Generate an isolated DDR-backed nav_soc from the frozen PNR15 source.

Does not edit the live PNR, build_g2.py, control RTL, firmware or board image.
Full physical integration is gated on a current-device DDR PHY and timing.
"""
import argparse
from datetime import datetime
import json
from pathlib import Path
import shutil

ROOT = Path(__file__).resolve().parents[3]
HERE = Path(__file__).resolve().parent


def once(text, old, new):
    assert text.count(old) == 1, f"source contract changed: {old[:90]!r}"
    return text.replace(old, new, 1)


EXTRA_PORTS = '''    // Cold reset is power-on/DDR initialization reset, NOT key/CPU warm reset.
    input wire ddr_clk, ddr_cold_rst, ddr_calib_done, ddr_pll_locked,
    output wire [2:0] ddr_cmd,
    output wire ddr_cmd_en,
    output wire [27:0] ddr_addr,
    input wire ddr_cmd_ready,
    output wire [255:0] ddr_wr_data,
    output wire [31:0] ddr_wr_mask,
    output wire ddr_wr_en, ddr_wr_end,
    input wire ddr_wr_ready,
    input wire [255:0] ddr_rd_data,
    input wire ddr_rd_valid, ddr_rd_end,
    output wire memory_initialized, memory_fault,
'''

BACKEND = '''    // Shared CPU/debug port. Writes are acknowledged only after DDR admission.
    // The backend continues draining across CPU/key reset, like retained RAM.
    reg [2:0] d_st;
    wire d_ram = d_addr == 16'hF008;
    wire memory_reset;
    nav_ddr_reset u_memory_reset(clk,ddr_cold_rst,memory_reset);
    reg ram_bus_req;
    reg [14:0] ram_bus_addr;
    reg [31:0] ram_bus_data;
    reg [3:0] ram_bus_strb;
    wire ram_bus_ack;
    wire line_req, line_we, line_ack;
    wire [24:0] line_addr;
    wire [255:0] line_wdata, line_rdata;
    wire [31:0] line_mask;
    wire native_fault;
    (* syn_keep = 1 *) reg fault1, fault2;
    always @(posedge clk or posedge memory_reset) begin
        if(memory_reset) begin fault1<=0; fault2<=0; end
        else begin fault1<=native_fault; fault2<=fault1; end
    end
    // UI clock can stop with PLL loss. Detect health independently on the
    // unchanged core clock and retain the fault across CPU/key warm resets.
    (* syn_keep = 1 *) reg health1, health2;
    reg health_seen, health_fault;
    always @(posedge clk or posedge memory_reset) begin
        if(memory_reset) begin health1<=0; health2<=0; health_seen<=0; health_fault<=0; end
        else begin
            health1<=ddr_pll_locked && ddr_calib_done; health2<=health1;
            if(health2) health_seen<=1;
            if(health_seen && !health2) health_fault<=1;
        end
    end
    assign memory_fault = fault2 | health_fault;
    nav_ddr_cache #(.RAM_WORDS(RAM_KB*256), .IMAGE_WORDS(DDR_IMAGE_WORDS),
        .INIT_PREFIX(DDR_INIT_PREFIX)) u_memory (
        .clk(clk), .cold_rst(memory_reset), .req(ram_bus_req),
        .word_addr(ram_bus_addr), .wdata(ram_bus_data), .wstrb(ram_bus_strb),
        .ack(ram_bus_ack), .rdata(ram_q), .initialized(memory_initialized),
        .line_req(line_req), .line_we(line_we), .line_addr(line_addr),
        .line_wdata(line_wdata), .line_mask(line_mask),
        .line_ack(line_ack), .line_rdata(line_rdata)
    );
    nav_ddr_native u_ddr (
        .core_clk(clk), .ui_clk(ddr_clk), .cold_rst(ddr_cold_rst),
        .req(line_req), .we(line_we), .line_addr(line_addr),
        .wdata(line_wdata), .wmask(line_mask), .ack(line_ack), .rdata(line_rdata),
        .calib_done(ddr_calib_done), .cmd(ddr_cmd), .cmd_en(ddr_cmd_en),
        .addr(ddr_addr), .cmd_ready(ddr_cmd_ready),
        .wr_data(ddr_wr_data), .wr_data_mask(ddr_wr_mask),
        .wr_data_en(ddr_wr_en), .wr_data_end(ddr_wr_end), .wr_data_rdy(ddr_wr_ready),
        .rd_data(ddr_rd_data), .rd_data_valid(ddr_rd_valid), .rd_data_end(ddr_rd_end),
        .fault(native_fault)
    );
    reg [1:0] ram_owner_state;
    reg ram_debug_owner, cpu_aborted, debug_done;
    wire dbg_ram_req = d_req && d_ram && hold && (d_st == 0 || d_st == 4);
    always @(posedge clk or posedge memory_reset) begin
        if(memory_reset) begin
            ram_owner_state<=0; ram_debug_owner<=0; cpu_aborted<=0;
            ram_bus_req<=0; ram_bus_addr<=0; ram_bus_data<=0; ram_bus_strb<=0;
            ram_ready<=0; debug_done<=0;
        end else begin
            ram_ready<=0; debug_done<=0;
            if(cpu_rst) cpu_aborted<=1;
            case(ram_owner_state)
                0: if(memory_initialized && !memory_fault) begin
                    if(dbg_ram_req || (!cpu_rst && req && sel_ram)) begin
                        ram_debug_owner<=dbg_ram_req;
                        ram_bus_addr<=dbg_ram_req ? dbg_ra : mem_addr[16:2];
                        ram_bus_data<=dbg_ram_req ? d_wdata : mem_wdata;
                        ram_bus_strb<=dbg_ram_req ? (d_we ? 4'hf : 4'h0) : mem_wstrb;
                        cpu_aborted<=0; ram_bus_req<=1; ram_owner_state<=1;
                    end
                end
                1: if(ram_bus_ack) begin
                    ram_bus_req<=0; ram_owner_state<=2;
                    if(ram_debug_owner) debug_done<=1;
                    else if(!cpu_rst && !cpu_aborted) ram_ready<=1;
                end
                2: if(!ram_bus_ack) ram_owner_state<=0;
                default: ram_owner_state<=0;
            endcase
        end
    end
    genvar b;

'''


def transform(text, image_words):
    assert 'parameter integer RAM_KB = 160' in text
    text = once(text, 'module nav_soc #(', 'module nav_soc_ddr #(')
    text = once(text, 'parameter integer RAM_KB = 160,', 'parameter integer RAM_KB = 128,')
    text = once(text, '    parameter INIT_B3 = ""',
        f'    parameter INIT_B3 = "",\n    parameter integer DDR_IMAGE_WORDS = {image_words},\n'
        '    parameter DDR_INIT_PREFIX = "boot_b"')
    text = once(text, '    input  wire        rst,', '    input  wire        rst,\n'+EXTRA_PORTS)
    text = once(text, 'wire         cpu_rst = rst | hold;',
        'wire         cpu_rst = rst | hold | !memory_initialized | memory_fault;')
    start = text.index('    // ================================================================ RAM（4 个字节通道）')
    end = text.index('    // ================================================================ 控制台环形缓冲', start)
    text = text[:start] + BACKEND + text[end:]
    text = once(text, "ram_ready <= 1'b0; nav_ready <= 1'b0; per_ready <= 1'b0;",
        "nav_ready <= 1'b0; per_ready <= 1'b0;")
    text = once(text, "            if (req && sel_ram) ram_ready <= 1'b1;\n", '')
    text = once(text, '    reg [1:0] d_st;', '    // d_st is declared above the DDR arbiter.\n    //')
    text = once(text, '    wire      d_ram = (d_addr == 16\'hF008);\n', '')
    text = once(text, '    assign dbg_ram_we = d_req && d_we && d_ram && (d_st == 2\'d0) && hold;\n', '')
    text = once(text, "            2'd0: if (d_req) begin\n                if (d_we) begin",
        "            2'd0: if (d_req) begin\n                if (d_ram && hold) d_st <= 3'd4;\n                else if (d_we) begin")
    text = once(text, "            2'd2: if (!d_req) d_st <= 2'd0;",
        "            3'd4: if(debug_done) begin\n"
        "                if(d_we) dbg_ra<=dbg_ra+1'b1; else d_rdata<=ram_q;\n"
        "                d_ack<=1; d_st<=2;\n            end\n"
        "            2'd2: if (!d_req) d_st <= 2'd0;")
    return text


def prepare(source, out):
    assert not out.exists(), 'do not overwrite a candidate'
    src = source/'nav/rtl/soc/nav_soc.v'
    assert src.read_bytes() == (ROOT/'rtl/soc/nav_soc.v').read_bytes(), 'unreviewed SoC input difference'
    m = json.loads((source/'manifest.json').read_text())
    assert m['real_sfp'] and m['full_control_telemetry'] and m['device_version']=='C'
    words = [int(x,16) for x in (source/'ram_w.hex').read_text().split()]
    assert len(words)==32768
    for b in range(4):
        lane=[int(x,16) for x in (source/f'ram_b{b}.hex').read_text().split()]
        assert lane == [(w>>(8*b))&255 for w in words], 'firmware channel mismatch'
    used = max(i for i,w in enumerate(words) if w)+1
    assert 0<used<=16384, 'this candidate supports a 128 KiB RAM and at most 64 KiB initialized image'
    assert used*4 == (source/'firmware/nav_fw.bin').stat().st_size
    out.mkdir(parents=True)
    for b,n in ((0,'lo'),(16,'hi')):
        (out/f'boot_{n}.hex').write_text(''.join(f'{(w>>b)&65535:04x}\n' for w in words[:used]))
    for b in range((used+1023)//1024):
        (out/f'boot_b{b:x}.hex').write_text(''.join(f'{w:08x}\n' for w in
            words[b*1024:(b+1)*1024]))
    for f in HERE.glob('*.v'): shutil.copyfile(f,out/f.name)
    shutil.copyfile(src, out/'nav_soc_baseline.v')
    (out/'nav_soc_ddr.v').write_text(transform(src.read_text(encoding='utf-8'),used),encoding='utf-8')
    for f in ('ram_w.hex','ram_b0.hex','ram_b1.hex','ram_b2.hex','ram_b3.hex'):
        shutil.copyfile(source/f,out/f)
    shutil.copyfile(source/'nav/rtl/soc/third_party/picorv32.v',out/'picorv32.v')
    for f in ('nav_ram_sp.v','nav_ram_sdp.v'):
        shutil.copyfile(source/'nav/rtl/common'/f,out/f)
    report=dict(created_at=datetime.now().astimezone().isoformat(),source=str(source),out=str(out),
        logical_ram_bytes=131072,boot_image_bytes=used*4,zero_tail_bytes=(len(words)-used)*4,
        cache_bytes=4096,all_32768_words_reconstructed_exactly=True,
        firmware_and_control_modified=False,programmed=False,physically_integrated=False,
        current_pnr_inputs_modified=False,
        scope='isolated candidate; DDR PHY, physical resources and navigation latency remain gated')
    (out/'candidate.json').write_text(json.dumps(report,indent=2),encoding='utf-8')
    return report


if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('source',type=Path);p.add_argument('--out',type=Path,required=True)
    a=p.parse_args();print(json.dumps(prepare(a.source.resolve(),a.out.resolve()),indent=2))
