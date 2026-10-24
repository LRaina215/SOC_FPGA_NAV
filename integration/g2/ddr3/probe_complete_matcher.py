"""Full SLF command-path numeric proof with a modeled DDR service envelope.

Uses the existing five frames/245 candidates/prior-map golden files. No joint
SYN/PNR or hardware action. Passing does not prove real DDR worst-case5ms.
"""
import argparse
from datetime import datetime
import json
from pathlib import Path
import re
import shutil
import subprocess
import xml.etree.ElementTree as ET

from derive_ddr_matcher import HERE, ROOT, derive, derive_store, derive_row_engine, derive_fast_cache
from derive_ddr_subsystem import derive as derive_subsystem, mmio_shell
from probe_row_update import run_syn, TOOL, validate, memory_status
from audit_netlist_memories import inventory as memory_inventory

FROZEN = Path('F:/nav_integration_g2_20261023/sfp_lane2_private_snapshot_syn11_retry1/nav')


def run(out, row_probe, quick=False, mmio=False, dirty=False):
    assert not out.exists()
    out.mkdir(parents=True)
    assert json.loads((row_probe/'functional_result.json').read_text())['passed']
    names = ['nav_ddr_native.v', 'nav_ddr_row_stream.v', 'nav_slf_row_update.v',
             'nav_ddr_struct_cache.v', 'nav_ddr_structure_store.v']
    for n in names:
        shutil.copyfile(HERE/n, out/n)
    (out/'nav_ddr_structure_store_full.v').write_text(derive_store((HERE/'nav_ddr_structure_store.v').read_text()),encoding='utf-8')
    (out/'nav_slf_row_update_full.v').write_text(derive_row_engine((HERE/'nav_slf_row_update.v').read_text()),encoding='utf-8')
    (out/'nav_ddr_struct_cache_fast.v').write_text(derive_fast_cache((HERE/'nav_ddr_struct_cache.v').read_text()),encoding='utf-8')
    names+=['nav_ddr_structure_store_full.v','nav_slf_row_update_full.v','nav_ddr_struct_cache_fast.v']
    shutil.copyfile(row_probe/'slf_lf_ce.v', out/'slf_lf_ce.v')
    original = (FROZEN/'rtl/match/slf_match.v').read_text(encoding='utf-8-sig')
    assert original == (ROOT/'rtl/match/slf_match.v').read_text(encoding='utf-8-sig')
    (out/'slf_match_ddr.v').write_text(derive(original), encoding='utf-8')
    if mmio:
        source=(FROZEN/'rtl/top/nav_subsystem.v').read_text(encoding='utf-8-sig')
        (out/'nav_subsystem_ddr.v').write_text(derive_subsystem(source),encoding='utf-8')
        (out/'nav_ddr_mmio_shell.v').write_text(mmio_shell(source),encoding='utf-8')
        for rel in ['rtl/regs/nav_regfile.v','rtl/regs/nav_regs_map.vh']:
            shutil.copyfile(FROZEN/rel,out/Path(rel).name)
        names+=['nav_regfile.v','nav_ddr_mmio_shell.v']
    for rel in ['rtl/match/slf_extract.v', 'rtl/common/nav_ram_sdp.v', 'rtl/plan/sin_lut.hex']:
        shutil.copyfile(FROZEN/rel, out/Path(rel).name)
    vectors = ROOT/'sim/build/tb_slf'
    for name in ['counts.txt', 'pts.hex', 'frm.hex', 'cand.hex', 'res.hex',
                 'sm.hex', 'smm.hex', 'prior.hex', 'pres.hex']:
        shutil.copyfile(vectors/name, out/name)
    if quick:
        counts = (out/'counts.txt').read_text().split()
        # Prior golden scores require the ENTIRE2618-word prior map even
        # when reducing the diagnostic frame/candidate count.
        counts[:2] = ['1', '5']
        (out/'counts.txt').write_text(' '.join(counts)+'\n')
    service = (HERE/'tb_slf_row_update.sv').read_text().split('module tb_slf_row_update;')[0]
    (out/'row_service_model.sv').write_text(service)
    tb = (ROOT/'rtl/match/tb/tb_slf.sv').read_text(encoding='utf-8-sig')
    def one(old, new):
        nonlocal tb
        assert tb.count(old) == 1, old
        tb = tb.replace(old, new, 1)
    one('    always #5 clk = ~clk;', '''    always #10 clk = ~clk;
    reg ui_clk=0;always #5 ui_clk=~ui_clk;
    wire fault,pw_ready;wire[2:0]dc;wire de,cr,we,wend,wr,rv,re;
    wire[27:0]da;wire[255:0]wd,rd;wire[31:0]wm;
    integer refresh_phase=389,ready_period=16,read_delay=20;
    row_service_model dram(ui_clk,rst,1'b1,dc,de,da,cr,wd,wm,we,wend,wr,rd,rv,re,
        refresh_phase,ready_period,read_delay);
    initial begin for(integer a=0;a<8192;a++)dram.mem[a]='1;end
    always@(posedge clk)if(!rst&&fault)$fatal(1,"unexpected storage fault");
    integer started_update,update_max=0,update_checks=0,clear_started=0;
    initial begin #(64'd10000000000);$fatal(1,"bounded functional simulation budget");end''')
    one('    slf_match #(.SIN_INIT("sin_lut.hex")) dut (',
        '    slf_match_ddr #(.SIN_INIT("sin_lut.hex")) dut (')
    one('        .busy(busy), .pts_ready(pts_ready), .n_pts(n_pts), .eval_done(eval_done));', '''        .busy(busy), .pts_ready(pts_ready), .n_pts(n_pts), .eval_done(eval_done),
        .pw_ready(pw_ready),.ui_clk(ui_clk),.calib_done(1'b1),.memory_fault(fault),
        .ddr_cmd(dc),.ddr_cmd_en(de),.ddr_addr(da),.ddr_cmd_ready(cr),
        .ddr_wr_data(wd),.ddr_wr_mask(wm),.ddr_wr_en(we),.ddr_wr_end(wend),
        .ddr_wr_ready(wr),.ddr_rd_data(rd),.ddr_rd_valid(rv),.ddr_rd_end(re));''')
    one('''            @(posedge clk); cmd_update <= 1; @(posedge clk); cmd_update <= 0;
            repeat (5) @(posedge clk);
            wait (!busy);''', '''            @(posedge clk);started_update=timing_cycle;cmd_update<=1;
            @(posedge clk);cmd_update<=0;
            repeat(5)@(posedge clk);wait(!busy);
            $display("FULL_UPDATE frame=%0d cycles=%0d us_at_50MHz=%0f",k,timing_cycle-started_update,(timing_cycle-started_update)/50.0);
            if(timing_cycle-started_update>update_max)update_max=timing_cycle-started_update;
            update_checks++;''')
    one('''        @(posedge clk); cmd_clear <= 1; @(posedge clk); cmd_clear <= 0;
        repeat (3) @(posedge clk);
        wait (!busy);''', '''        @(posedge clk);clear_started=timing_cycle;cmd_clear<=1;
        @(posedge clk);cmd_clear<=0;
        repeat(3)@(posedge clk);wait(!busy);
        $display("FULL_CLEAR cycles=%0d us_at_50MHz=%0f",timing_cycle-clear_started,(timing_cycle-clear_started)/50.0);''')
    one('''            @(posedge clk); pw_we <= 1; pw_addr <= prior[2*i][15:0]; pw_data <= prior[2*i+1][17:0];
            @(posedge clk); pw_we <= 0;
            @(posedge clk);''', '''            // Actual MMIO must backpressure on this handshake, not drop
            // single-cycle writes while the variable-latency map is stalled.
            @(negedge clk);pw_we=1;pw_addr=prior[2*i][15:0];pw_data=prior[2*i+1][17:0];
            do begin @(posedge clk);end while(!pw_ready);
            @(negedge clk);pw_we=0;
            wait(!busy);''')
    one('        if (errs == 0) $display("TEST PASSED");',
        '        if (errs == 0) $display("TEST PASSED frames=%0d candidates=%0d priors=%0d update_max_cycles=%0d",n_frm,n_cand,n_prior,update_max);')
    if mmio:
        one('    int n_frm, n_cand, n_prior, lfox, lfoy;',
            '    wire prior_valid;wire[15:0]prior_addr;wire[17:0]prior_data;\n    int n_frm, n_cand, n_prior, lfox, lfoy;')
        one('    int errs = 0;', '''    reg bus_req=0,bus_we=0;reg[15:0]bus_addr;reg[31:0]bus_wdata;
    wire bus_ack;wire[31:0]bus_rdata;
    integer bus_writes=0,prior_pulses=0,prior_max_wait=0;
    nav_ddr_mmio_shell shell(clk,rst,bus_req,bus_we,bus_addr,bus_wdata,
        bus_ack,bus_rdata,pw_ready,prior_valid,prior_addr,prior_data);
    always@(posedge clk)if(!rst&&prior_valid)begin
        if(!pw_ready)$fatal(1,"prior RF pulse while matcher not ready");
        prior_pulses++;
    end
    task automatic bus_write(input[15:0]a,input[31:0]d);
        integer waited;
        begin
            @(negedge clk);bus_req=1;bus_we=1;bus_addr=a;bus_wdata=d;waited=0;
            do begin @(posedge clk);#1;waited++;
                if(waited>10000)$fatal(1,"prior MMIO write timeout");end while(!bus_ack);
            if(waited>prior_max_wait)prior_max_wait=waited;
            @(negedge clk);bus_req=0;
            repeat(3)@(negedge clk);bus_writes++;
        end
    endtask
    int errs = 0;''')
        one('.pw_we(pw_we), .pw_addr(pw_addr), .pw_data(pw_data),',
            '.pw_we(prior_valid), .pw_addr(prior_addr), .pw_data(prior_data),')
        a=tb.index('            // Actual MMIO must backpressure')
        b=tb.index('        cfg_prior <= 1;',a)
        tb=tb[:a]+'''            bus_write(16'h7040,{16'd0,prior[2*i][15:0]});
            bus_write(16'h7044,prior[2*i+1]);
        end
        wait(!busy);
        if(prior_pulses!=n_prior||bus_writes!=2*n_prior)$fatal(1,"prior MMIO writes lost/duplicated");
        $display("MMIO_PROOF bus_writes=%0d prior_pulses=%0d longest_write_wait_cycles=%0d",bus_writes,prior_pulses,prior_max_wait);
''' + tb[b:]
    if dirty:
        assert mmio
        one('    int errs = 0;', '''    integer dirty_lines_checked=0,prep_prior_pulses=0;
    task automatic debug_word(input[15:0]a,output[17:0]d);
        begin
            @(negedge clk);dbg_req=1;dbg_addr=a;
            do begin @(posedge clk);#1;end while(!dbg_ack);
            d=dbg_q;@(negedge clk);dbg_req=0;
            wait(!busy);repeat(3)@(negedge clk);
        end
    endtask
    task automatic dirty_every_index;
        reg[17:0]old_word,ignored;integer a,conflict;
        begin
            for(integer l=0;l<256;l++)begin
                a=(l/19)*256+(l%19)*14;
                conflict=((l+256)/19)*256+((l+256)%19)*14;
                debug_word(a,old_word);
                debug_word(conflict,ignored);
                // Model setup: line l is now evicted. Seed a DIFFERENT old
                // backing value so lost dirty writes cannot pass as no-ops.
                dram.mem[l][0+:18]=old_word^18'h22222;
                bus_write(16'h7040,a);bus_write(16'h7044,old_word);
                wait(!busy);
            end
            for(integer l=0;l<256;l++)begin
                if(!dut.u_store.u_cache.meta[l][5])$fatal(1,"missing dirty index");
                dirty_lines_checked++;
            end
            prep_prior_pulses+=256;
        end
    endtask
    int errs = 0;''')
        one('            @(posedge clk);started_update=timing_cycle;cmd_update<=1;',
            '            dirty_every_index();\n            @(posedge clk);started_update=timing_cycle;cmd_update<=1;')
        one('if(prior_pulses!=n_prior||bus_writes!=2*n_prior)',
            'if(prior_pulses!=n_prior+prep_prior_pulses||bus_writes!=2*(n_prior+prep_prior_pulses))')
        one('        if (errs == 0) $display("TEST PASSED frames=',
            '        $display("DIRTY_COMBINED dirty_lines=%0d extra_prior_writes=%0d",dirty_lines_checked,prep_prior_pulses);\n        if (errs == 0) $display("TEST PASSED frames=')
    (out/'tb_complete_matcher.sv').write_text(tb, encoding='utf-8')
    viv = Path('E:/2026.1/Vivado/bin')
    commands = [('xvlog', names+['slf_lf_ce.v', 'slf_match_ddr.v', 'slf_extract.v', 'nav_ram_sdp.v']),
                ('xvlog', ['-sv', 'row_service_model.sv', 'tb_complete_matcher.sv']),
                ('xelab', ['-debug', 'off', '-timescale', '1ns/1ps', 'tb_slf', '-s', 'snap']),
                ('xsim', ['snap', '-runall'])]
    for i, (tool, args) in enumerate(commands):
        with (out/f'{i}_{tool}_stdout.log').open('w', encoding='utf-8') as log:
            subprocess.run(['cmd', '/c', str(viv/(tool+'.bat')), *args], cwd=out,
                stdout=log, stderr=subprocess.STDOUT, timeout=1800 if tool=='xsim' else 180,
                check=True, creationflags=subprocess.CREATE_NO_WINDOW)
    s = (out/'3_xsim_stdout.log').read_text(encoding='utf-8', errors='replace')
    assert 'TEST PASSED' in s and not re.search(r'TEST FAILED|FATAL|Fatal|ERROR:', s), s[-4000:]
    def lines(prefix):
        return [{k:float(v) if '.' in v else int(v) for k,v in re.findall(r'(\w+)=([\d.]+)', l)}
                for l in s.splitlines() if l.startswith(prefix)]
    result = dict(tested_at=datetime.now().astimezone().isoformat(), passed=True,
        quick=quick, update=lines('FULL_UPDATE'), evaluation=lines('SLF_TIMING'),
        mmio=lines('MMIO_PROOF'),
        dirty_combined=lines('DIRTY_COMBINED'),
        clear=lines('FULL_CLEAR'),
        metrics={k:int(v) for k,v in re.findall(r'(\w+)=(\d+)', re.search(r'TEST PASSED[^\r\n]*',s)[0])},
        scope='complete SLF command paths using original golden vectors and modeled native service',
        full245_candidates_tested=not quick, prior_write_backpressure_port_tested=True,
        original_point_transform_and_all_dirty_cache_combined=dirty,
        subsystem_MMIO_backpressure_connected=mmio, full_subsystem_simulated=False,
        real_worst_update_under_5ms_verified=False,
        elevation_map_integrated=False, full_joint_implementation=False, programmed=False)
    (out/'result.json').write_text(json.dumps(result,indent=2),encoding='utf-8')
    print(json.dumps(result,indent=2))


def synthesize(tested,out):
    proof=json.loads((tested/'result.json').read_text())
    assert proof['passed'] and not proof['quick'] and proof['subsystem_MMIO_backpressure_connected']
    expected={
        'slf_match_ddr.v':derive((FROZEN/'rtl/match/slf_match.v').read_text(encoding='utf-8-sig')),
        'nav_ddr_structure_store_full.v':derive_store((HERE/'nav_ddr_structure_store.v').read_text()),
        'nav_slf_row_update_full.v':derive_row_engine((HERE/'nav_slf_row_update.v').read_text()),
        'nav_ddr_struct_cache_fast.v':derive_fast_cache((HERE/'nav_ddr_struct_cache.v').read_text())}
    for n,s in expected.items():assert (tested/n).read_text(encoding='utf-8')==s,n
    for n in ['nav_ddr_native.v','nav_ddr_row_stream.v']:
        assert (tested/n).read_bytes()==(HERE/n).read_bytes(),n
    m=memory_status();assert m.avail_phys>=2*1024**3 and m.avail_page>=4*1024**3,'2/4GiB startup gate'
    assert not out.exists();out.mkdir()
    names=['slf_match_ddr.v','nav_ddr_structure_store_full.v','nav_slf_row_update_full.v',
        'nav_ddr_struct_cache_fast.v','nav_ddr_native.v','nav_ddr_row_stream.v',
        'slf_lf_ce.v','slf_extract.v','nav_ram_sdp.v']
    for n in names:
        s=(tested/n).read_text(encoding='utf-8-sig')
        if n=='slf_match_ddr.v':s=s.replace('module slf_match_ddr #(','module g2_top #(')
        (out/n).write_text(s,encoding='utf-8')
    shutil.copyfile(tested/'sin_lut.hex',out/'sin_lut.hex')
    (out/'clock.sdc').write_text('create_clock -name core -period 20 [get_ports {clk}]\ncreate_clock -name ui -period 10 [get_ports {ui_clk}]\n')
    settings=['set_device -device_version C GW5AT-LV138PG484AC1/I0']
    settings += [f'add_file -type verilog {{{(out/n).as_posix()}}}' for n in names]
    settings += [f'add_file -type sdc {{{(out/"clock.sdc").as_posix()}}}',
        'set_option -top_module g2_top','set_option -output_base_name g2_top',
        'set_option -global_freq 50','set_option -convert_sdp32_36_to_sdp16_18 0',
        'set_option -looplimit 100000']
    options=validate(TOOL,out,settings,50,0)
    (out/'run.tcl').write_text('\n'.join(settings+['run syn','exit'])+'\n')
    runtime=run_syn(out,scope='isolated complete SLF DDR adapter; no joint G2 SYN/PNR')
    inv=memory_inventory(out)
    assert not any('/u_sm/' in x['path'] or '/u_touch/' in x['path'] for x in inv['leaves']), 'old duplicate map/touch storage retained'
    cache=[x for x in inv['leaves'] if '/u_cache/' in x['path']]
    assert len(cache)==4 and all(x['parameters']['BIT_WIDTH_0']=='18' for x in cache)
    result=dict(tested=str(tested),runtime=runtime,effective_options=options,
        startup_available_physical=m.avail_phys,startup_available_commit=m.avail_page,
        resources=ET.parse(out/'impl/gwsynthesis/g2_top_syn_rsc.xml').getroot().attrib,
        inventory=inv,old_full_map_removed=True,cache_narrow_leaves=4,
        real_worst_update_under_5ms_verified=False,full_joint_implementation=False,
        physical_resources_verified=False,programmed=False)
    (out/'result.json').write_text(json.dumps(result,indent=2),encoding='utf-8')
    print(json.dumps({k:v for k,v in result.items() if k!='inventory'},indent=2))


if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--out',type=Path,required=True)
    p.add_argument('--row-probe',type=Path);p.add_argument('--quick',action='store_true')
    p.add_argument('--synthesis-after',type=Path)
    p.add_argument('--mmio',action='store_true')
    p.add_argument('--dirty-before-update',action='store_true')
    a=p.parse_args()
    if a.synthesis_after:synthesize(a.synthesis_after.resolve(),a.out.resolve())
    else:
        assert a.row_probe is not None
        run(a.out.resolve(),a.row_probe.resolve(),a.quick,a.mmio,a.dirty_before_update)
