"""Lossless LF line-storage compression, isolated proof and optional small SYN.

Keep the original 256x256 map, scan, arithmetic and 19-bit window interface.
The line RAM stores only occupancy plus the existing ten high slot bits; the
duplicated centroid bits and constant midpoint bits are rebuilt by wiring.
No joint synthesis, placement, routing or hardware action is implemented here.
"""
import argparse
from collections import Counter
from datetime import datetime
import json
from pathlib import Path
import re
import shutil
import subprocess
import time
import xml.etree.ElementTree as ET

import psutil
from audit_netlist_memories import BSRAM, PARAM, MODULE, INST
from retry_frozen_synthesis import memory_status
from synthesis_connectivity import read
from measure_slf_row_banks import namespaced, LIBRARY


ROOT = Path(__file__).resolve().parents[2]
TOOL = Path('D:/Gowin2/Gowin_V1.9.12.04_x64/IDE/bin/gw_sh.exe')
VIV = Path('E:/2026.1/Vivado/bin')
RELATIVE = 'nav/rtl/match/slf_lf.v'
OLD = '''    reg [CW-1:0] lb1 [0:LB-1];
    reg [CW-1:0] lb2 [0:LB-1];
    reg [CW-1:0] lb3 [0:LB-1];
    reg [CW-1:0] lb4 [0:LB-1];
    wire [CW-1:0] l1 = lb1[c1], l2 = lb2[c1], l3 = lb3[c1], l4 = lb4[c1];'''
NEW = '''    // Lossless storage: {occupancy, tag, hit, cx3, cy3}; rebuild duplicate bits by wiring.
    localparam integer LINE_CW = 1 + HB;
    function [CW-1:0] lf_line_expand(input [LINE_CW-1:0] word);
        lf_line_expand = {word[HB], word[5:3], 1'b1, word[2:0], 1'b1, word[HB-1:0]};
    endfunction
    reg [LINE_CW-1:0] lb1 [0:LB-1];
    reg [LINE_CW-1:0] lb2 [0:LB-1];
    reg [LINE_CW-1:0] lb3 [0:LB-1];
    reg [LINE_CW-1:0] lb4 [0:LB-1];
    wire [CW-1:0] l1 = lf_line_expand(lb1[c1]), l2 = lf_line_expand(lb2[c1]),
                  l3 = lf_line_expand(lb3[c1]), l4 = lf_line_expand(lb4[c1]);'''
OLD_WRITE = '                lb1[c1] <= cur; lb2[c1] <= l1; lb3[c1] <= l2; lb4[c1] <= l3;'
NEW_WRITE = '                lb1[c1] <= {cur[CW-1], cur[HB-1:0]}; lb2[c1] <= lb1[c1]; lb3[c1] <= lb2[c1]; lb4[c1] <= lb3[c1];'


def transform(text):
    text = text.replace('\r\n', '\n')
    assert re.search(r'parameter\s+integer\s+LOG2N\s*=\s*8\s*,', text), 'retain the frozen 256x256 default'
    assert text.count(OLD) == text.count(OLD_WRITE) == 1, 'unexpected frozen LF line organization'
    changed = text.replace(OLD, NEW).replace(OLD_WRITE, NEW_WRITE)
    assert changed.replace(NEW, OLD).replace(NEW_WRITE, OLD_WRITE) == text
    return changed


UNIT_TB = r'''`timescale 1ns/1ps
module tb;
reg clk=0,rst=1,start=0;always #10 clk=~clk;
reg signed[31:0]cx=0,cy=0;
wire ba,bb,da,db,ra,rb,wa,wb;wire[15:0]aa,ab,xa,xb;wire[17:0]ya,yb;
reg[17:0]qa=0,qb=0;reg[17:0]ma[0:65535],mb[0:65535];
slf_lf_old a(clk,rst,start,cx,cy,ba,da,ra,aa,qa,wa,xa,ya);
slf_lf_new b(clk,rst,start,cx,cy,bb,db,rb,ab,qb,wb,xb,yb);
integer checks=0,writes=0,reads=0,backing=0,line_checks=0,runs=0,depth[0:257];
integer saved_address,saved_depth;reg saved_write;
function[18:0]expand(input[10:0]x);
 begin expand={x[10],x[5:3],1'b1,x[2:0],1'b1,x[9:0]};end
endfunction
reg[31:0]rng=32'h4500c019;
always@(posedge clk)begin
 if(ra)begin qa<=ma[aa];reads++;end
 if(rb)qb<=mb[ab];
 if(wa)begin ma[xa]<=ya;writes++;end
 if(wb)mb[xb]<=yb;
 saved_write=!rst && a.v1;saved_address=a.c1;
 if(saved_write)begin
  depth[saved_address]++;saved_depth=depth[saved_address];#1;
  if(a.lb1[saved_address]!==expand(b.lb1[saved_address]))$fatal(1,"line1 mismatch at %0d",saved_address);
  line_checks++;
  if(saved_depth>=2)begin if(a.lb2[saved_address]!==expand(b.lb2[saved_address]))$fatal(1,"line2 mismatch");line_checks++;end
  if(saved_depth>=3)begin if(a.lb3[saved_address]!==expand(b.lb3[saved_address]))$fatal(1,"line3 mismatch");line_checks++;end
  if(saved_depth>=4)begin if(a.lb4[saved_address]!==expand(b.lb4[saved_address]))$fatal(1,"line4 mismatch");line_checks++;end
 end
end
always@(negedge clk)begin
 checks++;
 if(ba!==bb || da!==db || ra!==rb || wa!==wb)$fatal(1,"LF handshake/cycle mismatch %0d",checks);
 if(ra && aa!==ab)$fatal(1,"LF read address mismatch");
 if(wa && (xa!==xb || ya!==yb))$fatal(1,"LF write mismatch %0d old=%h new=%h",checks,ya,yb);
end
task reset_both;
 begin @(negedge clk);#1;rst=1;start=0;repeat(3)@(negedge clk);#1;rst=0;
 for(integer i=0;i<258;i++)depth[i]=0;end
endtask
task seed(input integer pattern);
 begin
 for(integer i=0;i<65536;i++)begin
  rng=rng^(rng<<13);rng=rng^(rng>>17);rng=rng^(rng<<5);
  case(pattern)
   0:ma[i]=0;
   1:ma[i]=18'h3ffff;
   2:ma[i]={i[15],i[7],2'b11,i[5:0],8'hff};
   default:ma[i]=rng[17:0];
  endcase
  mb[i]=ma[i];
 end
 end
endtask
task run(input signed[31:0]x,y,input integer interrupt_after);
 integer elapsed;
 begin
 @(negedge clk);#1;cx=x;cy=y;start=1;
 @(negedge clk);#1;start=0;elapsed=0;
 while(ba)begin
  @(negedge clk);#1;elapsed++;
  // Start while busy is ignored by both original and candidate.
  start=(elapsed==71 || elapsed==1031);
  if(interrupt_after>0 && elapsed==interrupt_after)begin reset_both();break;end
  if(elapsed>70000)$fatal(1,"LF watchdog");
 end
 start=0;repeat(3)@(negedge clk);
 for(integer i=0;i<65536;i++)begin
  if(ma[i]!==mb[i])$fatal(1,"LF full-map mismatch slot=%0d",i);
  backing++;
 end
 runs++;
 $display("LF_RUN run=%0d x=%0d y=%0d cycles=%0d interrupted=%0d",runs,x,y,elapsed,interrupt_after);
 end
endtask
initial begin
 for(integer i=0;i<258;i++)depth[i]=0;
 for(integer i=0;i<2048;i++)begin
  if(expand(i[10:0])!=={i[10],i[5:3],1'b1,i[2:0],1'b1,i[9:0]})$fatal(1,"encoding mismatch");
 end
 seed(0);reset_both();run(0,0,0);
 seed(1);run(127,-128,0);
 seed(2);run(-128,511,0);
 seed(3);run(255,256,0);
 run(-257,-513,0);
 run(777,-999,1003);
 run(777,-999,0);
 seed(3);reset_both();run(32'sh7ffffe80,-32'sh7ffffe80,0);
 $display("TEST PASSED LF checks=%0d runs=%0d full_map_words=%0d writes=%0d reads=%0d four_line_checks=%0d all2048encodings=1 initial_zero_nonzero=1 resets=3",checks,runs,backing,writes,reads,line_checks);$finish;
end
initial begin #20000000;$fatal(1,"LF proof timeout");end
endmodule
'''


def save(path, value):
    path.write_text(json.dumps(value, ensure_ascii=False, indent=2),
                    encoding='utf-8', newline='\n')


def execute(argv, folder, name, seconds=300, rss_limit=768*1024**2):
    """Guard only this newly-created low-priority, one-core process tree."""
    started = time.monotonic()
    runtime = dict(command=[str(x) for x in argv], started_at=datetime.now().astimezone().isoformat(),
        rss_limit_bytes=rss_limit, max_runtime_s=seconds, cancelled=False,
        priority='IDLE', cores=1, programmed=False)
    with (folder/(name+'_stdout.log')).open('w', encoding='utf-8') as log:
        process = subprocess.Popen([str(x) for x in argv], cwd=folder, stdout=log,
            stderr=subprocess.STDOUT, creationflags=subprocess.CREATE_NO_WINDOW|subprocess.IDLE_PRIORITY_CLASS)
        owned = psutil.Process(process.pid)
        runtime['pid'] = process.pid
        runtime['creation_time'] = owned.create_time()
        core = owned.cpu_affinity()[0]
        owned.cpu_affinity([core])
        peak = 0
        while process.poll() is None:
            children = []
            try:children = owned.children(recursive=True)
            except psutil.NoSuchProcess:pass
            used = 0
            for p in [owned]+children:
                try:
                    used += p.memory_info().rss
                    p.nice(psutil.IDLE_PRIORITY_CLASS)
                    p.cpu_affinity([core])
                except (psutil.NoSuchProcess, psutil.AccessDenied):pass
            peak = max(peak, used)
            memory = memory_status()
            reason = ('owned RSS exceeds guard' if used>rss_limit else
                'isolated tool simulation/SYN budget' if time.monotonic()-started>seconds else
                'host reserve protects main PNR' if memory.avail_phys<1024**3 or memory.avail_page<2*1024**3 else None)
            if reason:
                runtime.update(cancelled=True, reason=reason)
                for p in list(reversed(children))+[owned]:
                    try:p.kill()
                    except psutil.NoSuchProcess:pass
                process.wait()
                break
            time.sleep(.25)
        process.wait()
    runtime.update(tool_exit=process.returncode, elapsed_s=time.monotonic()-started,
        peak_owned_RSS=peak, ended_at=datetime.now().astimezone().isoformat())
    save(folder/(name+'_runtime.json'), runtime)
    assert process.returncode == 0 and not runtime['cancelled'], runtime
    return runtime


def simulate(folder, files, top):
    memory = memory_status()
    assert memory.avail_phys>=2*1024**3 and memory.avail_page>=4*1024**3, 'simulation host reserve'
    runtime = []
    runtime.append(execute(['cmd','/c',VIV/'xvlog.bat','-sv',*[str(p) for p in files]], folder, 'xvlog'))
    runtime.append(execute(['cmd','/c',VIV/'xelab.bat','-debug','off','-timescale','1ns/1ps',top,'-s','snap'], folder, 'xelab'))
    runtime.append(execute(['cmd','/c',VIV/'xsim.bat','snap','-runall'], folder, 'xsim', seconds=1200))
    text = (folder/'xsim.log').read_text(encoding='utf-8', errors='replace')
    assert 'TEST PASSED' in text and 'TEST FAILED' not in text and 'Fatal:' not in text, text[-4000:]
    return dict(passed=True, result=re.findall(r'TEST PASSED[^\n]*', text),
                timing_lines=re.findall(r'(?:LF_RUN|SLF_TIMING)[^\n]*', text), runtimes=runtime)


def frozen_LF_RAM(source):
    modules = dict(MODULE.findall((source/'impl/gwsynthesis/g2_top.vg').read_text(encoding='utf-8')))
    kind = 'g2_top'
    for name in ('u_nav', 'u_match', 'u_lf'):
        kind, = [child for child, instance in INST.findall(modules[kind])
                 if instance.lstrip('\\') == name]
    assert kind == 'slf_lf'
    ram = []
    for child, instance in INST.findall(modules[kind]):
        if child in BSRAM:
            values = {k:v for n,k,v in PARAM.findall(modules[kind])
                      if n.strip() == instance and not k.startswith('INIT_RAM')}
            ram.append(dict(cell=instance, kind=child, parameters=values))
    wide = sum(any(r['parameters'].get(k) in ('32','36')
               for k in ('BIT_WIDTH','BIT_WIDTH_0','BIT_WIDTH_1')) for r in ram)
    assert len(ram) == 4 and wide == 4
    return dict(source=str(source), RAM=ram, actual_source_BSRAM=len(ram),
                actual_wide_leaves=wide, predicted_split_physical_BSRAM=len(ram)+wide,
                scope='completed full SYN17 LF subtree; physical block comparison still requires new joint PNR')


def optional_synthesis(out, source, changed):
    memory = memory_status()
    reserve = dict(available_physical=memory.avail_phys, available_commit=memory.avail_page)
    if memory.avail_phys<2*1024**3 or memory.avail_page<4*1024**3:
        return dict(started=False, reason='2GiB physical / 4GiB commit unavailable; main PNR protected', **reserve)
    folder = out/'isolated_syn'
    folder.mkdir()
    # Only the LF module is synthesized; no new candidate full-system build.
    assert changed.count('module slf_lf #(') == 1
    (folder/'lf.v').write_text(changed.replace('module slf_lf #(', 'module g2_top #(', 1), encoding='utf-8')
    (folder/'clock.sdc').write_text('create_clock -name clk -period 20 [get_ports {clk}]\n', encoding='utf-8')
    settings = ['set_device -device_version C GW5AT-LV138PG484AC1/I0',
        f'add_file -type verilog {{{(folder/"lf.v").as_posix()}}}',
        f'add_file -type sdc {{{(folder/"clock.sdc").as_posix()}}}',
        'set_option -top_module g2_top', 'set_option -output_base_name g2_top',
        'set_option -global_freq 50', 'set_option -convert_sdp32_36_to_sdp16_18 1',
        'set_option -looplimit 100000', 'set_option -replicate_resources 1']
    (folder/'run.tcl').write_text('\n'.join(settings+['run syn','exit'])+'\n', encoding='utf-8')
    runtime = execute([TOOL,'run.tcl'], folder, 'gw_sh', seconds=180)
    log = (folder/'gw_sh_stdout.log').read_text(encoding='utf-8', errors='replace')
    assert not re.search(r'\bERROR\s*\(', log), log[-3000:]
    vg = folder/'impl/gwsynthesis/g2_top.vg'
    cells = read(vg)
    kinds = Counter(c['kind'] for c in cells.values())
    modules = dict(MODULE.findall(vg.read_text(encoding='utf-8')))
    ram = [dict(cell=n, kind=c['kind'], parameters={k:v for inst,k,v in PARAM.findall(modules['g2_top'])
            if inst.strip().lstrip('\\') == n and not k.startswith('INIT_RAM')})
            for n,c in cells.items() if c['kind'] in BSRAM]
    wide = sum(any(r['parameters'].get(k) in ('32','36') for k in ('BIT_WIDTH','BIT_WIDTH_0','BIT_WIDTH_1')) for r in ram)
    resources = ET.parse(folder/'impl/gwsynthesis/g2_top_syn_rsc.xml').getroot().attrib
    baseline = frozen_LF_RAM(source)
    return dict(started=True, completed=True, runtime=runtime, startup_memory=reserve,
        frozen_joint_LF_baseline=baseline,
        predicted_physical_blocks_saved=baseline['predicted_split_physical_BSRAM']-len(ram)-wide,
        actual_SYN_resources=resources, RAM=ram, actual_source_BSRAM=len(ram),
        actual_wide_leaves=wide, predicted_physical_BSRAM=len(ram)+wide,
        primitives=dict(kinds), physical_BSRAM_unmeasured=True,
        physical_timing_unverified=True, joint_candidate_unchanged=True)


def mapped_public_interface_proof(out):
    """Reuse the one completed SYN, with no private mapped array assumptions."""
    out = Path(out)
    folder = out/'vendor_public_interface'
    folder.mkdir(exist_ok=False)
    vg = out/'isolated_syn/impl/gwsynthesis/g2_top.vg'
    (folder/'vendor.v').write_text(namespaced(vg.read_text(encoding='utf-8'), 'new'), encoding='utf-8')
    shutil.copyfile(out/'unit/old.v', folder/'old.v')
    bench = UNIT_TB
    start = bench.index(' saved_write=!rst')
    end = bench.index('\nend\nalways@(negedge clk)', start)
    bench = bench[:start]+bench[end:]
    bench = bench.replace('slf_lf_new b(clk,rst,start,cx,cy,bb,db,rb,ab,qb,wb,xb,yb);',
        'g2_new b(.clk(clk),.rst(rst),.start(start),.cx(cx),.cy(cy),.busy(bb),.done(db),'
        '.sm_re(rb),.sm_raddr(ab),.sm_q(qb),.sm_we(wb),.sm_waddr(xb),.sm_wdata(yb));')
    old = ' $display("TEST PASSED LF checks=%0d runs=%0d full_map_words=%0d writes=%0d reads=%0d four_line_checks=%0d all2048encodings=1 initial_zero_nonzero=1 resets=3",checks,runs,backing,writes,reads,line_checks);$finish;'
    new = ' $display("TEST PASSED MAPPED LF checks=%0d runs=%0d full_map_words=%0d writes=%0d reads=%0d initial_zero_nonzero=1 resets=3",checks,runs,backing,writes,reads);$finish;'
    assert bench.count(old) == 1 and 'a.v1' not in bench and 'b.lb' not in bench
    bench = bench.replace(old, new)
    (folder/'tb.sv').write_text(bench, encoding='utf-8')
    return dict(scope='same unique mapped LF and original vendor primitive models versus original RTL; public pins and all backing words',
        source_netlist=str(vg), original_vendor_library=str(LIBRARY),
        private_array_observation=False, **simulate(folder,
            [LIBRARY, folder/'old.v', folder/'vendor.v', folder/'tb.sv'], 'tb'))


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--source', type=Path, required=True)
    ap.add_argument('--out', type=Path, required=True)
    ap.add_argument('--fixtures', type=Path,
        default=Path('F:/nav_integration_g2_20261023/ddr_complete_matcher_dirty_probe1'))
    ap.add_argument('--verify', action='store_true')
    ap.add_argument('--synthesize', action='store_true')
    a = ap.parse_args()
    assert not a.synthesize or a.verify, 'functional proof precedes isolated SYN'
    source, out = a.source.resolve(), a.out.resolve()
    assert not out.exists() and source not in out.parents and out not in source.parents
    old = (source/RELATIVE).read_text(encoding='utf-8')
    changed = transform(old)
    out.mkdir(parents=True)
    candidate = out/RELATIVE
    candidate.parent.mkdir(parents=True)
    candidate.write_text(changed, encoding='utf-8', newline='\n')
    # Exhaust every eleven-bit information state, not just observed values.
    for word in range(2048):
        hb, occ = word&1023, word>>10
        expanded = (occ<<18)|((((hb>>3)&7)*2+1)<<14)|(((hb&7)*2+1)<<10)|hb
        packed = ((expanded>>18)<<10)|(expanded&1023)
        assert packed == word
    result = dict(source=str(source), build=str(out), prepared_at=datetime.now().astimezone().isoformat(),
        modified_RTL_inputs=[RELATIVE], exact_inverse_transform=True,
        map_words=65536, slot_bits=18, scan_N=256, line_depth=258,
        old_line_width=19, new_line_information_width=11, original_19bit_window_interface_retained=True,
        logical_line_bits_before=4*258*19, logical_line_bits_after=4*258*11,
        all2048_information_encodings_passed=True, original_scan_arithmetic_precision_and_latency_retained=True,
        programmed=False, deployment_accepted=False, joint_build_started=False, PNR_started=False,
        source_asset_files_unchanged=True, functional_proofs=[], synthesis=dict(started=False),
        limits='Isolated behavior and optional SYN only. No whole-system route/timing or 5ms claim.')
    save(out/'result.json', result)
    if a.verify:
        unit = out/'unit'
        unit.mkdir()
        (unit/'old.v').write_text(old.replace('module slf_lf #(', 'module slf_lf_old #(', 1), encoding='utf-8')
        (unit/'new.v').write_text(changed.replace('module slf_lf #(', 'module slf_lf_new #(', 1), encoding='utf-8')
        (unit/'tb.sv').write_text(UNIT_TB, encoding='utf-8')
        result['functional_proofs'].append(dict(scope='full original LF scan old/new cycle and map equality',
            **simulate(unit, [unit/x for x in ('old.v','new.v','tb.sv')], 'tb')))
        save(out/'result.json', result)
        full = out/'original_full_matcher'
        full.mkdir()
        files = []
        for relative in ('nav/rtl/common/nav_ram_sdp.v','nav/rtl/match/slf_extract.v',
                         'nav/rtl/match/slf_lf.v','nav/rtl/match/slf_match.v'):
            destination = full/Path(relative).name
            destination.write_text(changed if relative == RELATIVE else (source/relative).read_text(encoding='utf-8'), encoding='utf-8')
            files.append(destination)
        bench = ROOT/'rtl/match/tb/tb_slf.sv'
        shutil.copyfile(bench, full/'tb_slf.sv')
        files.append(full/'tb_slf.sv')
        for filename in ('pts.hex','frm.hex','cand.hex','res.hex','sm.hex','smm.hex',
                         'prior.hex','pres.hex','counts.txt','sin_lut.hex'):
            shutil.copyfile(a.fixtures/filename, full/filename)
        result['functional_proofs'].append(dict(scope='unmodified original full SLF five-frame/245-candidate/prior/model full-map fixture',
            fixture=str(a.fixtures.resolve()), testbench=str(bench), **simulate(full, files, 'tb_slf')))
        save(out/'result.json', result)
    if a.synthesize:
        result['synthesis'] = optional_synthesis(out, source, changed)
        save(out/'result.json', result)
        if result['synthesis'].get('completed'):
            result['functional_proofs'].append(mapped_public_interface_proof(out))
            save(out/'result.json', result)
    print(json.dumps(result, ensure_ascii=False), flush=True)


if __name__ == '__main__':
    main()
