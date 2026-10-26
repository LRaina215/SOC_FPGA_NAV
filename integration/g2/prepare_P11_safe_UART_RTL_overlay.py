"""P11 RX parser/quiesce overlay kept as Python strings until explicit admission.

The default writes review JSON and a proposed textual patch only, never Verilog.
--materialize produces a new overlay after both PNR54 and SYN44 naturally exit.
This entry never invokes an EDA tool, compiles firmware, or modifies a source job.
"""
from pathlib import Path
from datetime import datetime
import argparse,copy,difflib,importlib.util,json,re,shutil
import psutil,yaml
from prepare_baseline_fusion_retime import metadata,save
from prepare_P11_parser_offload import PARSER,unique
from prepare_FW_sin_ROM_RTL_overlay import gate54

BASE=Path('F:/nav_integration_g2_20261023')
REPO=Path(__file__).resolve().parents[2]
SOURCE=BASE/'SLF128_ring_stride32_SYN44_prepared1'
FW=BASE/'P11_quiesce_FW_sin_MMIO_prepared1'
SYN44=BASE/'SLF128_ring_stride32_syn44/job.json'

REGS=[
 dict(name='P11_CTRL',off=0x20,acc='rwe',reset=0,
      fields={'ENABLE':[0,0],'SOURCE_FLUSH':[1,1],'SOURCE_READY':[8,8]},
      desc='Write ENABLE bit0; bit1 pulse explicitly clears RX only. Read effective ENABLE bit0 and cleanup-completed READY bit8; not actuator authorization'),
 dict(name='P11_CMD',off=0x24,acc='ro',reset=0,
      desc='Atomic CRC/range accepted signed16 forward mmps low and yaw mradps high'),
 dict(name='P11_TS',off=0x28,acc='ro',reset=0,
      desc='Actual synchronized FIRST idle-burst first-character start time us; conservative if noise precedes frame'),
 dict(name='P11_SEQ',off=0x2c,acc='ro',reset=0,
      desc='Atomic generation;0 invalid,FFFFFFFF fail-closed saturation freezes CMD/TS too')]

# Same byte parser previously reviewed, with a private timestamp capture rather
# than depending on the tap timestamp remaining stable throughout a packet.
SNAPSHOT=PARSER.replace('reg [31:0] frame_age;', 'reg [31:0] frame_age,burst_ts;')
SNAPSHOT=SNAPSHOT.replace('pending_speed<=0;pending_yaw<=0;frame_age<=0;',
                        'pending_speed<=0;pending_yaw<=0;frame_age<=0;burst_ts<=0;')
SNAPSHOT=SNAPSHOT.replace('have_burst<=1;burst_used<=0;index<=',
                        'have_burst<=1;burst_used<=0;burst_ts<=first_start_us;index<=')
SNAPSHOT=SNAPSHOT.replace('command_ts<=first_start_us;', 'command_ts<=burst_ts;')
assert SNAPSHOT.count('command_ts<=burst_ts;')==2

QUIESCE=r'''
    // Administrative RX isolation only. The existing synchronizer and all TX
    // logic continue running. A normal ACTIVE start never clears READY.
    reg rx_quiescing;
    reg [10:0] rx_high_ticks;
    reg [15:0] source_cfg_div,source_cfg_idle;
    wire source_cfg_changed=(source_cfg_div!=cfg_div)||(source_cfg_idle!=cfg_idle_us);
    wire rx_fifo_reset=rst|source_flush|rx_quiescing;
    always @(posedge clk) begin
        if(rst) begin
            rx_quiescing<=1'b0;source_ready<=1'b1;rx_high_ticks<=0;
            source_cfg_div<=cfg_div;source_cfg_idle<=cfg_idle_us;
        end else begin
            source_cfg_div<=cfg_div;source_cfg_idle<=cfg_idle_us;
            if(source_flush) begin
                rx_quiescing<=1'b1;source_ready<=1'b0;rx_high_ticks<=0;
            end else if(rx_quiescing) begin
                if(source_cfg_changed||!rxs) rx_high_ticks<=0;
                else if(tick_us) begin
                    // 2001 HIGH observations span at least 2000us. LOW on any
                    // clk, not just a tick, restarts qualification.
                    if(rx_high_ticks==11'd2000) begin
                        rx_quiescing<=1'b0;source_ready<=1'b1;
                    end else rx_high_ticks<=rx_high_ticks+1'b1;
                end
            end
        end
    end
    assign tap_valid=push;
    assign tap_data=push_d[7:0];
    assign tap_first=push_d[8];
    assign tap_first_start_us=tpush_d;
'''

TOP=r'''
    // p_* is combinational wr/address decode. The flush bit MUST come from
    // this transaction's wd_q, not the previous r_IMU_P11_CTRL value.
    wire iu_source_flush=p_IMU_P11_CTRL & wd_q[1];
    assign i_IMU_P11_CTRL={23'd0,iu_source_ready,7'd0,
                           (r_IMU_P11_CTRL[0]&iu_source_ready)};
    p11_command_snapshot #(.FRAME_TIMEOUT_CLKS(500000)) u_p11_snapshot (
        .clk(clk),.rst(rst),.enable(r_IMU_P11_CTRL[0]&iu_source_ready),
        .flush(p_IMU_P11_CTRL|!iu_source_ready),
        .byte_valid(iu_tap_valid),.byte_data(iu_tap_data),.byte_first(iu_tap_first),
        .first_start_us(iu_tap_first_ts),.command_word(i_IMU_P11_CMD),
        .command_ts(i_IMU_P11_TS),.command_seq(i_IMU_P11_SEQ)
    );
'''

def text(p):return p.read_text(encoding='utf-8-sig')

def patch_uart(old):
    t=unique(old,'    output reg         irq,              // 单拍：突发首字节到达',
      '    input  wire        source_flush,\n    output reg         source_ready,\n'
      '    output wire        tap_valid,tap_first,\n    output wire [7:0]  tap_data,\n'
      '    output wire [31:0] tap_first_start_us,\n'
      '    output reg         irq,              // 单拍：突发首字节到达')
    t=unique(t,'    wire        rf_ready;','    wire        rf_ready;\n'+QUIESCE)
    t=unique(t,"            busy_r <= 1'b0; idle <= 16'hFFFF; st_ovf <= 0;\n        end else begin",
      "            busy_r <= 1'b0; idle <= 16'hFFFF; st_ovf <= 0;\n"
      "        end else if(source_flush||rx_quiescing) begin\n"
      "            busy_r<=0;bc<=0;bit_i<=0;sh<=0;idle<=16'hffff;\n"
      "            first_pend<=0;t_start<=0;push_d<=0;tpush_d<=0;\n"
      "            // st_ovf remains cumulative; only common NAV reset clears it.\n"
      "        end else begin")
    t=unique(t,'.clk(clk), .rst(rst), .s_valid(push),',
               '.clk(clk), .rst(rx_fifo_reset), .s_valid(push),')
    t=unique(t,'.clk(clk), .rst(rst), .s_valid(tpush),',
               '.clk(clk), .rst(rx_fifo_reset), .s_valid(tpush),')
    # Existing TX and real RX synchronizer remain literally unchanged.
    marker='    // ---------------- 发送 ----------------'
    assert t[t.index(marker):]==old[old.index(marker):]
    assert 'nav_sync2 #(.W(1)) u_s (.clk(clk), .d(uart_rx), .q(rxs));' in t
    return t

def patch_top(old):
    t=unique(old,'    wire [31:0] iu_ts, iu_ovf;',
      '    wire [31:0] iu_ts, iu_ovf;\n    wire iu_source_ready,iu_tap_valid,iu_tap_first;\n'
      '    wire [7:0] iu_tap_data;\n    wire [31:0] iu_tap_first_ts;\n'+TOP)
    return unique(t,'        .st_ovf(iu_ovf), .irq(iu_irq),',
      '        .st_ovf(iu_ovf), .irq(iu_irq),\n'
      '        .source_flush(iu_source_flush),.source_ready(iu_source_ready),\n'
      '        .tap_valid(iu_tap_valid),.tap_first(iu_tap_first),\n'
      '        .tap_data(iu_tap_data),.tap_first_start_us(iu_tap_first_ts),')

def gate44(required):
    r=json.loads(SYN44.read_text());assert r['tool_pid']==51268
    created=float(r['tool_created_epoch']);assert abs(created-1793028868.262156)<.02
    running=False
    try:running=abs(psutil.Process(51268).create_time()-created)<.02
    except psutil.NoSuchProcess:pass
    final=r.get('tool_exit') is not None and r['phase'] in ('synthesis_complete','synthesis_failed','failed')
    natural=final and not running and not r.get('cancelled',False) and r.get('tool_exit') in (0,1)
    info=dict(job=metadata(SYN44),pid=51268,created_epoch=created,phase=r['phase'],
      tool_exit=r.get('tool_exit'),original_alive=running,natural_complete=natural)
    if required:assert natural,'SYN44 original process is not naturally final; no Verilog may be produced'
    return info

def source_review(source):
    rels=['nav/rtl/io/imu_uart.v','nav/rtl/top/nav_subsystem.v',
          'nav/rtl/regs/nav_regfile.v','configuration_source/config/regmap.yaml',
          'g2_top.v','control4500_core.v','control/uart_speed_rx.v',
          'nav/rtl/common/nav_fifo.v','nav/rtl/common/nav_cdc.v',
          'nav/rtl/common/nav_fifo_ob.v','nav/rtl/common/nav_ram_sdp.v']
    # Locate named support modules exactly; source tree has historically moved
    # folders but their selected module definitions must not be guessed.
    for i,rel in enumerate(rels):
        p=source/rel
        if not p.is_file() and rel.endswith(('nav_fifo.v','nav_cdc.v','nav_fifo_ob.v','nav_ram_sdp.v')):
            name=Path(rel).name;found=list((source/'nav/rtl').rglob(name))
            assert len(found)==1,(name,found);rels[i]=found[0].relative_to(source).as_posix()
    files=[source/r for r in rels];assert all(p.is_file()for p in files),rels
    cfg=yaml.safe_load(text(source/'configuration_source/config/regmap.yaml'))
    occupied={b['base']+r['off']for b in cfg['blocks']for r in b['regs']}
    for r in REGS:
        assert 0x6000+r['off'] not in occupied
        assert not any(w['base']<=0x6000+r['off']<w['base']+w['size']for w in cfg['windows'])
    header=text(FW/'fw/include/nav_regs_map.h')
    for r in REGS:assert '#define NAV_IMU_'+r['name']+' 0x%04Xu'%(0x6000+r['off'])in header
    service=text(FW/'fw/nav/p11_snapshot_service.c')
    for s in('uint32_t a=p11_snapshot_hal_seq();','uint32_t word=p11_snapshot_hal_command();',
              'uint32_t stamp=p11_snapshot_hal_timestamp();','uint32_t b=p11_snapshot_hal_seq();',
              'if(a!=b)return P11_SNAPSHOT_UNSTABLE;','if(b==0xffffffffu)return P11_SNAPSHOT_LOST;'):
        assert s in service,s
    generator=text(REPO/'tools/gen_config.py')
    assert 'assign p_{n} = wr && (addr ==' in generator
    uart=text(files[0]);new_uart=patch_uart(uart);top=text(files[1]);new_top=patch_top(top)
    assert new_top.count('iu_source_flush=p_IMU_P11_CTRL & wd_q[1]')==1
    assert '.m_ready(rx_pop)'in new_uart and'.m_ready(ts_pop)'in new_uart
    return dict(files=[metadata(p)for p in files],support=rels[-4:],
      firmware=[metadata(FW/'fw/include/nav_regs_map.h'),metadata(FW/'fw/nav/p11_snapshot_service.c'),
                metadata(FW/'fw/drv/hal_fpga.c'),metadata(FW/'fw/app/nav_app.c')],
      old_addresses_unchanged=True,new_CSR_no_overlap=True,
      register_write_pulse='actual generated rwe p is current wr/address, so bit1 uses wd_q; stored r_CTRL is one edge too old for an action strobe',
      original_TX_literal_unchanged=True,original_sync_literal_unchanged=True),new_uart,new_top,cfg

TEST_HARNESS=r'''
// Proposed isolated CSR harness. Uses the actual generated nav_regfile; it is
// not a replacement register-map implementation or the full NAV bus wrapper.
module p11_csr_probe_harness(
 input wire clk,rst,tick_us,input wire[31:0] time_us,
 input wire uart_rx,input wire wr,rd,input wire[15:0] addr,input wire[31:0] wdata,
 output wire[31:0] rdata,output wire hit,output wire uart_tx,
 output wire source_ready,output wire[31:0] cmd,stamp,seq,
 output wire[31:0] ovf,output wire[9:0] rx_count,
 output wire rx_valid,rx_first,ts_valid,output wire[7:0] rx_data,output wire[31:0] ts_data
);
 wire[31:0] r_IMU_BAUD_DIV,r_IMU_IDLE_US,r_IMU_P11_CTRL;
 wire p_IMU_P11_CTRL,p_IMU_RX_DATA,p_IMU_RX_TS,p_IMU_TX_DATA;
 wire tx_busy,tx_full,irq,tap_valid,tap_first;wire[7:0] tap_data;wire[31:0] tap_ts;
 wire[31:0] ctrl={23'd0,source_ready,7'd0,(r_IMU_P11_CTRL[0]&source_ready)};
 nav_regfile rf(@RF_CONNECTIONS@);
 imu_uart uart(.clk(clk),.rst(rst),.time_us(time_us),.tick_us(tick_us),
   .cfg_div(r_IMU_BAUD_DIV[15:0]),.cfg_idle_us(r_IMU_IDLE_US[15:0]),
   .uart_rx(uart_rx),.uart_tx(uart_tx),
   .rx_valid(rx_valid),.rx_data(rx_data),.rx_first(rx_first),
   .rx_pop(p_IMU_RX_DATA&rx_valid),.rx_count(rx_count),
   .ts_valid(ts_valid),.ts_data(ts_data),.ts_pop(p_IMU_RX_TS&ts_valid),
   .st_ovf(ovf),.irq(irq),.tx_we(p_IMU_TX_DATA),.tx_data(wdata[7:0]),
   .tx_busy(tx_busy),.tx_full(tx_full),
   .source_flush(p_IMU_P11_CTRL&wdata[1]),.source_ready(source_ready),
   .tap_valid(tap_valid),.tap_first(tap_first),.tap_data(tap_data),.tap_first_start_us(tap_ts));
 p11_command_snapshot parser(.clk(clk),.rst(rst),
   .enable(r_IMU_P11_CTRL[0]&source_ready),.flush(p_IMU_P11_CTRL|!source_ready),
   .byte_valid(tap_valid),.byte_first(tap_first),.byte_data(tap_data),
   .first_start_us(tap_ts),.command_word(cmd),.command_ts(stamp),.command_seq(seq));
endmodule
'''

def harness(regfile):
    ports=re.findall(r'^\s*(input|output)\s+(?:wire|reg)\s*(\[[^\]]+\])?\s*(\w+)\s*[,)]',regfile,re.M)
    assert ports and len({n for _,_,n in ports})==len(ports)
    fixed={'clk':'clk','rst':'rst','wr':'wr','rd':'rd','addr':'addr','wdata':'wdata','rdata':'rdata','hit':'hit',
      'i_IMU_RX_DATA':"(rx_valid?{1'b1,22'd0,rx_first,rx_data}:32'd0)",
      'i_IMU_RX_COUNT':"{22'd0,rx_count}",'i_IMU_RX_TS':"(ts_valid?ts_data:32'd0)",
      'i_IMU_TX_STAT':"{30'd0,tx_full,tx_busy}",'i_IMU_STAT_OVF':'ovf',
      'i_IMU_P11_CTRL':'ctrl','i_IMU_P11_CMD':'cmd','i_IMU_P11_TS':'stamp','i_IMU_P11_SEQ':'seq'}
    same={'r_IMU_BAUD_DIV','r_IMU_IDLE_US','r_IMU_P11_CTRL','p_IMU_P11_CTRL','p_IMU_RX_DATA','p_IMU_RX_TS','p_IMU_TX_DATA'}
    connections=[]
    for direction,width,name in ports:
        value=fixed.get(name,name if name in same else ("32'd0"if width else"1'b0")if direction=='input'else'')
        connections.append('.%s(%s)'%(name,value))
    assert same<={n for _,_,n in ports}
    return TEST_HARNESS.replace('@RF_CONNECTIONS@',',\n    '.join(connections))

def materialize(source,out,review,new_uart,new_top,cfg):
    out.mkdir();rels=['nav/rtl/io/imu_uart.v','nav/rtl/top/nav_subsystem.v']
    for rel,value in zip(rels,[new_uart,new_top]):
        p=out/rel;p.parent.mkdir(parents=True,exist_ok=True);p.write_text(value,encoding='utf8')
    p=out/'nav/rtl/io/p11_command_snapshot.v';p.write_text(SNAPSHOT,encoding='utf8')
    # Regenerate only in the new overlay, using the original generator, never
    # source/current selected files. All old register addresses are compared.
    cfg=copy.deepcopy(cfg);next(b for b in cfg['blocks']if b['name']=='IMU')['regs']+=copy.deepcopy(REGS)
    cfgroot=out/'configuration_source';(cfgroot/'config').mkdir(parents=True)
    (cfgroot/'config/regmap.yaml').write_text(yaml.safe_dump(cfg,sort_keys=False,allow_unicode=True),encoding='utf8')
    spec=importlib.util.spec_from_file_location('P11_safe_cfg_generator',REPO/'tools/gen_config.py')
    mod=importlib.util.module_from_spec(spec);spec.loader.exec_module(mod);mod.ROOT=cfgroot
    mod.gen_regmap(cfg);mod.gen_regfile(cfg)
    for name in('nav_regfile.v','nav_regs_map.vh','nav_subsystem_rf.vh'):
        dest=out/'nav/rtl/regs'/name;dest.parent.mkdir(parents=True,exist_ok=True)
        shutil.copy2(cfgroot/'rtl/regs'/name,dest)
    regtext=text(out/'nav/rtl/regs/nav_regfile.v')
    olddefines=dict(re.findall(r'^`define (RA_\w+) (16\'h[\da-fA-F]+)',text(source/'nav/rtl/regs/nav_regs_map.vh'),re.M))
    newdefines=dict(re.findall(r'^`define (RA_\w+) (16\'h[\da-fA-F]+)',text(out/'nav/rtl/regs/nav_regs_map.vh'),re.M))
    assert olddefines and all(newdefines.get(k)==v for k,v in olddefines.items())
    tb=out/'test';tb.mkdir();(tb/'p11_csr_probe_harness.sv').write_text(harness(regtext),encoding='utf8')
    (tb/'imu_uart_reference.v').write_text(unique(text(source/'nav/rtl/io/imu_uart.v'),'module imu_uart #(','module imu_uart_reference #('),encoding='utf8')
    save(out/'overlay_binding.json',dict(source_review=review,all_old_register_addresses_preserved=True,
      selected_FW_BIN_INIT_not_changed=True,source_list_not_changed=True,not_complete_validation=True,
      outputs=[metadata(p)for p in sorted(out.rglob('*'))if p.is_file()]))

def main():
    ap=argparse.ArgumentParser(description=__doc__);ap.add_argument('--source',type=Path,default=SOURCE)
    ap.add_argument('--out',type=Path,required=True);ap.add_argument('--materialize',action='store_true')
    a=ap.parse_args();assert not a.out.exists();gates=dict(PNR54=gate54(a.materialize),SYN44=gate44(a.materialize))
    review,uart,top,cfg=source_review(a.source)
    if a.materialize:materialize(a.source,a.out,review,uart,top,cfg)
    else:a.out.mkdir()
    diff=[]
    for rel,new in [('nav/rtl/io/imu_uart.v',uart),('nav/rtl/top/nav_subsystem.v',top)]:
        diff+=difflib.unified_diff(text(a.source/rel).splitlines(True),new.splitlines(True),fromfile='source/'+rel,tofile='future/'+rel)
    (a.out/'proposed_UART_source.patch').write_text(''.join(diff),encoding='utf8')
    result=dict(at=datetime.now().astimezone().isoformat(),phase='independent overlay, still unvalidated'if a.materialize else'plan and Python strings only',
      baseline=str(a.source),selected_future_FW=str(FW),source_review=review,gates=gates,
      RTL_materialized=a.materialize,EDA_started=False,firmware_recompiled=False,current_jobs_modified=False,
      exact_registers=[dict(address=hex(0x6000+r['off']),**r)for r in REGS],
      snapshot_reads=['SEQ at602C','CMD at6024','TS at6028','SEQ at602C'],
      protocol=dict(extended='A5 5A 10 04 forward signed16 LE/yaw signed16 LE CRC16 LE',
        legacy='A5 5A 10 02 forward signed16 LE CRC16 LE; yaw0',
        CRC='CCITT-FALSE1021 initFFFF, version+length+payload only',ranges='forward±600mmps yaw±2900mradps',
        interbyte_timeout='500000 clk50 cycles=10ms; no movement TTL in RTL',
        timestamp='FIRST idle-burst actual synchronized first-character start, captured privately on its valid stop; not CPU dequeue or CRC completion',
        noise='noise before A5 shares earlier TS and therefore expires earlier',
        continuous='one successful frame per FIRST burst; no500us idle means no renewal; rawFIFOfull cannot backpressure tap',
        snapshot='CMD/TS/SEQ one clk publication; sequence0 invalid andFFFFFFFF fail-closed freezes all three',
        atomicity='four volatile reads same FW owner, no concurrent administrative flush writer; equalSEQ proves same generation under no wrap'),
      flush=dict(write='CTRL p&&wd_q[1] clears RX/FIFOs plus parser on that accepted edge; r_CTRL is previous value on same edge',
        scope='RX state andDATA/TS queues only; real nav_sync2 keeps observing RX; original TX literal unchanged',
        common_reset='preserve ordinary raw startup with READY1/ACTIVE; selectedFW BOOT or CPUHOLD init explicitly writesCTRL2 and remainsOFF/R0locked',
        qualify='READY0; every sampledLOW restarts;2001 HIGH tick_us observations span≥2000us; cfg changes restart only whileisolated',
        release='automatic ACTIVE+stickyREADY; releaseedge still resetsRXFIFO; nextclock resumesnormalRX; normalstart never clearsREADY',
        overflow='st_ovf retains original cumulative DATAoverflow evidence; administrativeflush doesnot zero it',
        config_owner='FW exclusive source-switch, rawTXidle+empty beforecfg_div change; unsupported concurrentdebugwrites are outsidecontract'),
      ownership='no physical command output added; all P11 proposal/R0/feedback/200ms/WAIT_ZERO/motion-support/HAL/SAFE/lease behavior stays in selectedfutureFW',
      actual_FW_budget='future quiesce+snapshot+sineMMIO ELF has1924B measuredspare; not final budget after any new sensorbridgeFW',
      planned_outputs=['modified imu_uart/nav_subsystem','new p11_command_snapshot','regmap.yaml+originalgeneratedmaps/regfile','isolatedactualregfile testharness+oldUARTreference'],
      tests_before_acceptance=[
        'Freeze overlay andtestTB, then only oneRTLtool at2GiBphys/4GiBcommit; retainactualPID/create/commands/log; PASS andnoFatalrequired',
        'Actual serial217clk/bit8N1 through realnav_sync2/RX, not directbyteinjection: extended/legacy/CRC/ranges, signedextremes, headerresync, badstop/glitch',
        'OriginalrawUART miter while noadministrativeflush, sameDATA/FIRST/TS/count/overflow/TX; explicitflush changes onlyRX andstopsoldproducer/FIFOdata',
        'Actual generatednav_regfile writes/readmapping; CTRL2 followedCTRL1 exerciseswd_qactionnotstalerCtrl; enableonlyflushesparsernotraw',
        'Quiesce fromeach0..9serialphase plusoldpush/tpushboundary; realrxsLOWforever, LOWglitchbetweenus ticks,2001HIGHticks, cfgchange, clkstop/restart, startaroundrelease; neverforceinternalstate',
        'Fullproduction256byte/32TSqueues andselectivepops: parserprogressindependentofbothfull; unrelatedFIFOorphanscannotfabricateP11TS',
        'Snapshotpublication between eachSEQ/CMD/TS/SEQ read boundary; mismatch/gap/flush/saturation and timestampwrap mustfailclosed; saturationcanuseparameterizedlogicalproofratherthanforcingSEQ',
        'Endtoend selectedFW/realCPU busy250/350ms/physical500ms remainsnewgate, not replacedbyCSRsubstitute or isolatedUARTtests'],
      missing_before_runtime=['finite serialstimulusTB andrunner not yet emitted/executed; generatedharnessalone is notverification',
        'fullNAVbus/CPUACKlatency miter beyondisolatedactualnav_regfile',
        'actualCmapping/clock/reset/STOP/CEB/INIT/IO/STA',
        'P11commandmodeNAVBMIattitudegap mustbeclosed separately; UARTparser doesnot manufactureWIT'],
      generator_materialization_gate='explicitflag AND PNR54naturalcompletion AND SYN44naturalcompletion/originalPIDexit; noautoEDA',
      final_sensor_contract='NAVstillreceivescontrolodo_vx/wz viaSAFE_ODO, butnofusiongravity/rawBMIpath. P11mode disablesWITfeed: pitch/roll/gyrotime/attjumpupdatesnotmaintained; do notacceptasfullnavigation',
      deployment_ready=False)
    save(a.out/'result.json',result)
    if not a.materialize:
        assert not list(a.out.rglob('*.v')) and not list(a.out.rglob('*.sv'))
        save(REPO/'docs/4_design/evidence/g2_20261023/P11_safe_UART_RTL_overlay_plan_20261026.json',result)
    print(json.dumps(dict(output=str(a.out),materialized=a.materialize,EDA=False,
      SYN44gate=gates['SYN44']['natural_complete'],PNR54gate=gates['PNR54']['natural_completion_gate'])))

if __name__=='__main__':main()
