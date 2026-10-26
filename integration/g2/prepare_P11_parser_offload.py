"""Prepare the P11 parser-offload contract; materialization is explicit and gated.

Default --plan writes JSON only. It does not write RTL, compile, or touch PNR
inputs. --materialize must be separately authorized after PNR53/54 naturally
finish; it creates a new, unaccepted tree and never invokes an EDA tool.
"""
from pathlib import Path
from datetime import datetime
import argparse,copy,difflib,importlib.util,json,re,shutil,sys
from prepare_baseline_fusion_retime import metadata,save

BASE=Path('F:/nav_integration_g2_20261023')
REPO=Path(__file__).resolve().parents[2]
SOURCE=BASE/'SLF128_storage_safeFW_SYN43_prepared1'
NEW_REGS=[
 dict(name='P11_CTRL',off=0x20,acc='rwe',reset=0,
      fields={'ENABLE':[0,0]},desc='P11 parser only; every write flushes old snapshot/partial packet, ENABLE0 keeps WIT/raw path; does not authorize motion'),
 dict(name='P11_CMD',off=0x24,acc='ro',reset=0,
      desc='Atomic valid-frame latch: signed16 forward mm/s low, signed16 yaw mrad/s high; no direct actuator path'),
 dict(name='P11_TS',off=0x28,acc='ro',reset=0,
      desc='Actual FIRST idle-burst first-character start time us, not CRC completion/read time; may conservatively precede A5 after noise'),
 dict(name='P11_SEQ',off=0x2c,acc='ro',reset=0,
      desc='Valid-frame sequence;0 invalid, increments atomically with CMD/TS;FFFFFFFF saturates and freezes all three until explicit flush')]

# Future output only. This string is not selected/compiled by any current job.
PARSER=r'''// Experimental P11 byte parser. NAV/FW remains the sole actuator owner.
// Factored CRC/range/legacy state machine from frozen uart_speed_rx.v.
module p11_command_snapshot #(
    parameter integer FRAME_TIMEOUT_CLKS=500000
)(
    input wire clk,rst,enable,flush,
    input wire byte_valid,byte_first,
    input wire [7:0] byte_data,
    input wire [31:0] first_start_us,
    output reg [31:0] command_word,command_ts,command_seq
);
    function [15:0] crc_byte;
        input [15:0] previous; input [7:0] value;
        reg [15:0] c;integer i;
        begin
            c=previous^{value,8'b0};
            for(i=0;i<8;i=i+1)c=c[15]?(c<<1)^16'h1021:(c<<1);
            crc_byte=c;
        end
    endfunction
    reg [3:0] index;
    reg extended_frame,have_burst,burst_used;
    reg [15:0] crc;
    reg [7:0] speed_low,yaw_low,crc_low;
    reg signed [15:0] pending_speed,pending_yaw;
    reg [31:0] frame_age;
    wire values_in_range=pending_speed>=-16'sd600&&pending_speed<=16'sd600&&
        pending_yaw>=-16'sd2900&&pending_yaw<=16'sd2900;
    always @(posedge clk)begin
        if(rst||flush||!enable)begin
            index<=0;extended_frame<=0;have_burst<=0;burst_used<=0;
            crc<=16'hffff;speed_low<=0;yaw_low<=0;crc_low<=0;
            pending_speed<=0;pending_yaw<=0;frame_age<=0;
            command_word<=0;command_ts<=0;command_seq<=0;
        end else begin
            if(index!=0)begin
                if(frame_age==FRAME_TIMEOUT_CLKS-1)begin index<=0;frame_age<=0;end
                else frame_age<=frame_age+1'b1;
            end
            if(byte_valid)begin
                frame_age<=0;
                if(byte_first)begin
                    // FIRST also discards any incomplete earlier burst.
                    have_burst<=1;burst_used<=0;index<=(byte_data==8'ha5)?1:0;
                end else if(have_burst&&!burst_used)begin
                    case(index)
                        0:if(byte_data==8'ha5)index<=1;
                        1:if(byte_data==8'h5a)index<=2;else index<=(byte_data==8'ha5)?1:0;
                        2:if(byte_data==8'h10)begin crc<=crc_byte(16'hffff,byte_data);index<=3;end
                          else index<=(byte_data==8'ha5)?1:0;
                        3:if(byte_data==8'h02||byte_data==8'h04)begin
                            extended_frame<=(byte_data==8'h04);pending_yaw<=0;
                            crc<=crc_byte(crc,byte_data);index<=4;
                          end else index<=(byte_data==8'ha5)?1:0;
                        4:begin speed_low<=byte_data;crc<=crc_byte(crc,byte_data);index<=5;end
                        5:begin pending_speed<={byte_data,speed_low};crc<=crc_byte(crc,byte_data);index<=6;end
                        6:if(extended_frame)begin yaw_low<=byte_data;crc<=crc_byte(crc,byte_data);index<=7;end
                          else begin crc_low<=byte_data;index<=7;end
                        7:if(extended_frame)begin pending_yaw<={byte_data,yaw_low};crc<=crc_byte(crc,byte_data);index<=8;end
                          else begin
                            index<=0;
                            if({byte_data,crc_low}==crc&&values_in_range)begin
                                burst_used<=1;
                                if(command_seq!=32'hffffffff)begin
                                    command_word<={16'd0,pending_speed};command_ts<=first_start_us;
                                    command_seq<=command_seq+1'b1;
                                end
                            end
                          end
                        8:begin crc_low<=byte_data;index<=9;end
                        9:begin
                            index<=0;
                            if({byte_data,crc_low}==crc&&values_in_range)begin
                                burst_used<=1;
                                if(command_seq!=32'hffffffff)begin
                                    command_word<={pending_yaw,pending_speed};command_ts<=first_start_us;
                                    command_seq<=command_seq+1'b1;
                                end
                            end
                          end
                        default:index<=0;
                    endcase
                end
            end
        end
    end
endmodule
'''

TOP=r'''
    // Parsing only: no connection to SAFE/lease/control command inputs.
    assign i_IMU_P11_CTRL={31'd0,r_IMU_P11_CTRL[0]};
    p11_command_snapshot #(.FRAME_TIMEOUT_CLKS(500000)) u_p11_snapshot (
        .clk(clk),.rst(rst),.enable(r_IMU_P11_CTRL[0]),.flush(p_IMU_P11_CTRL),
        .byte_valid(iu_tap_valid),.byte_data(iu_tap_data),.byte_first(iu_tap_first),
        .first_start_us(iu_tap_first_ts),.command_word(i_IMU_P11_CMD),
        .command_ts(i_IMU_P11_TS),.command_seq(i_IMU_P11_SEQ)
    );
'''

# This adapter is a future isolated component, not linked into selected FW.
C_SERVICE=r'''#include "p11_snapshot_service.h"
// Exactly four real MMIO reads, no retry loop or queue/arrival-time synthesis.
p11_snapshot_result_t p11_snapshot_read(uint32_t previous,p11_snapshot_t *out)
{
    uint32_t a=p11_snapshot_hal_seq();
    uint32_t word=p11_snapshot_hal_command();
    uint32_t stamp=p11_snapshot_hal_timestamp();
    uint32_t b=p11_snapshot_hal_seq();
    if(a!=b)return P11_SNAPSHOT_UNSTABLE;
    if(!b||b==previous)return P11_SNAPSHOT_NONE;
    if(b==0xffffffffu||(uint32_t)(b-previous)!=1u)return P11_SNAPSHOT_LOST;
    out->forward_mmps=(int16_t)word;out->yaw_mradps=(int16_t)(word>>16);
    out->start_us=stamp;out->sequence=b;return P11_SNAPSHOT_NEW;
}
'''
C_HEADER=r'''#ifndef P11_SNAPSHOT_SERVICE_H
#define P11_SNAPSHOT_SERVICE_H
#include <stdint.h>
typedef struct{int16_t forward_mmps,yaw_mradps;uint32_t start_us,sequence;}p11_snapshot_t;
typedef enum{P11_SNAPSHOT_NONE,P11_SNAPSHOT_NEW,P11_SNAPSHOT_UNSTABLE,P11_SNAPSHOT_LOST}p11_snapshot_result_t;
uint32_t p11_snapshot_hal_seq(void);
uint32_t p11_snapshot_hal_command(void);
uint32_t p11_snapshot_hal_timestamp(void);
p11_snapshot_result_t p11_snapshot_read(uint32_t previous,p11_snapshot_t *out);
#endif
'''

def read_text(p):return p.read_text(encoding='utf-8-sig')
def unique(t,a,b):
    assert t.count(a)==1,(a,t.count(a));return t.replace(a,b)

def source_contract(source):
    import yaml
    files=[source/'nav/rtl/io/imu_uart.v',source/'nav/rtl/top/nav_subsystem.v',
           source/'control/uart_speed_rx.v',source/'configuration_source/config/regmap.yaml',
           source/'control4500_core.v',source/'g2_top.v',source/'g2.cst',
           source/'fw/include/nav_regs_map.h',source/'run.tcl',REPO/'tools/gen_config.py',
           source/'nav/rtl/soc/nav_soc.v']
    assert all(p.is_file()for p in files),[str(p)for p in files if not p.is_file()]
    uart=read_text(files[0]);top=read_text(files[1]);parser=read_text(files[2])
    anchors=["t_start <= time_us;","first_pend <= (idle >= cfg_idle_us);",
             "push <= 1'b1; push_d <= {first_pend, sh};",
             "if (first_pend) begin tpush <= 1'b1; tpush_d <= t_start; irq <= 1'b1; end",
             '.s_valid(push), .s_data(push_d), .s_ready(rf_ready)']
    assert all(a in uart for a in anchors),[a for a in anchors if a not in uart]
    assert "if (push && !rf_ready) st_ovf <= st_ovf + 1'b1;"in uart
    assert 'wire signed [31:0] converted=rounded/32\'sd1000;'in parser
    assert '.rx_count(iu_cnt), .ts_valid(iu_tsv), .ts_data(iu_ts), .ts_pop(p_IMU_RX_TS & iu_tsv),'in top
    g2=read_text(source/'g2_top.v');soc=read_text(source/'nav/rtl/soc/nav_soc.v')
    assert 'nav_rst_sync u_rs (.clk(clk), .arst(~key_rst_n | ~pll_lock), .rst(rst));'in g2
    assert '.clk(clk), .rst(rst),'in g2 and'wire         cpu_rst = rst | hold;'in soc and'.resetn(~cpu_rst)'in soc
    cfg=yaml.safe_load(read_text(files[3]));imu=next(b for b in cfg['blocks']if b['name']=='IMU')
    # Frozen YAML quotes off, unlike YAML1.1's boolean interpretation.
    assert imu['base']==0x6000
    addresses={b['base']+r['off']:b['name']+'_'+r['name']for b in cfg['blocks']for r in b['regs']}
    for reg in NEW_REGS:
        addr=0x6000+reg['off'];assert addr not in addresses,(hex(addr),addresses.get(addr))
        assert all(not(w['base']<=addr<w['base']+w['size'])for w in cfg['windows']),hex(addr)
    return dict(inputs=[metadata(p)for p in files],source=str(source),checks_passed=True,
        source_anchors=[dict(text=a,line=uart[:uart.index(a)].count('\n')+1)for a in anchors],
        t_start='updated for EACH attempted character start (synchronized falling start); push only after valid stop',
        tpush_d='updated ONLY for valid stop with first_pend: original idle-burst FIRST character t_start',
        tap='push/push_d/tpush_d at pre-FIFO registered producer, independent of rf_ready; parser observes all on next clk',
        fixed_500us_idle=True,byte_timestamp_not_CPU_dequeue=True,new_CSRs=[dict(name='IMU_'+r['name'],address=0x6000+r['off'],access=r['acc'])for r in NEW_REGS])

def make_plan(source,out):
    assert not out.exists(),out;contract=source_contract(source);out.mkdir()
    plan=dict(at=datetime.now().astimezone().isoformat(),phase='plan_only_no_RTL_written',
        source_contract=contract,implementation_generator=str(Path(__file__).resolve()),
        normal_commands='A5 5A 10 04 signed16 forward/yaw LE CRC16LE; legacy length02 yaw0; original CRC/ranges',
        arrival_semantics=dict(source='existing FIRST idle-burst first character synchronized start time',
            noise='if noise precedes A5 in same burst, TS precedes actual frame; conservative earlier expiry',
            continuous='only first valid packet per idle-separated burst; no>=500us idle means no further seq/time renewal and FW expires',
            parser_timeout='original500000 clk50 clocks=10ms interbyte incomplete-frame discard, not movement TTL',
            t_start_per_character=True,tpush_d_per_burst=True,zero_timestamp='FW rejects0 conservatively including exact32-bit time wrap'),
        atomic_snapshot=dict(registers='CMD32,TS32,SEQ32 update in one clk50 always block only on valid CRC+range',
            read_order=['SEQ a','CMD','TS','SEQ b'],bounded_MMIO_reads=4,retries=0,
            stable='accept only a==b and1<=b<FFFFFFFF andb==previous+1; use capturedTS, never read time',
            mismatch='no proposal renewal; deadline/feedback/R0 polls still execute; retry only on next bounded service call',
            gaps='sequence gap or saturation => FW LOCKED/zero, explicit trusted rearm only',
            consumed_sequence='advance previous on every coherent NEW snapshot, even when WAIT_ZERO rejects its nonzero command; never renew accepted command timestamp on rejection',
            saturation='SEQ saturates FFFFFFFF and freezes CMD/TS too; no wrap ABA',
            exclusivity='Only same no-IRQ FW owner writes CTRL; arbitrary simultaneous debug writes are outside this owner contract'),
        owner_switch=dict(management='existing trusted NV45 only, IDLE/actual still500ms/no pending matcher; state OFF/WAIT_ZERO/ARMED/LOCKED in FW',
            sequence=['zero via original common HAL','writeCTRL0 =>disable+flush; raw/WIT queues retained',
                      'configure actual baud217/idle500 for P11 or868 for WIT','writeCTRL1 =>flush+enable/wait newFIRST',
                      'sample authorization time AFTER enable ACK; previous_seq0; WAIT_ZERO',
                      'require post-authorization TS, consecutive new sequence, fresh actual feedback and newzero before ARMED'],
            new_snapshot_before_authorization='rejected by timestamp/sessionbaseline; does not auto-arm',
            reset='common NAV reset resets parser snapshot AND CPU/owner; CPUbootslocked; independent CTRL change flushes only parser and FWmust be managing/locked',
            R0_timeout_us=250000,P11_timeout_us=200000,feedback_timeout_us=50000,
            original_command_age350ms_and_physical500ms_not_relaxed=True,
            hardware_owner_remains_NAV=True,actuator_route='FW proposal before motion/support→R0/P11 final zero filters→SAFE→existing lease→4500',
            no_P11_to_control_UART_or_postlease_mux=True),
        WIT_and_raw=dict(old_FIFO_and_TS_read_semantics_unchanged=True,parser_tap_never_reads_FIFO=True,
            parser_ready_not_connected_to_RF_ready=True,full_FIFO_does_not_block_parser=True,
            parser_disable_in_WIT=True,legacy_WIT_parser_and_rawdebug_kept=True,
            BMI088_and_control_fusion_unchanged=True,telemetry_A16_V2_unchanged=True),
        planned_files=dict(RTL_modified=['nav/rtl/io/imu_uart.v','nav/rtl/top/nav_subsystem.v'],
            RTL_new=['nav/rtl/io/p11_command_snapshot.v'],
            config='configuration_source/config/regmap.yaml',
            generated=['nav/rtl/regs/nav_regfile.v','nav/rtl/regs/nav_regs_map.vh','nav/rtl/regs/nav_subsystem_rf.vh','fw/include/nav_regs_map.h','configuration_source/model/nav_regs_map.py','configuration_source/docs/2_interface/regmap.md'],
            FW_experiment_only=['experimental_fw/p11_snapshot_service.c','experimental_fw/p11_snapshot_service.h'],
            selected_FW_not_integrated_by_generator=True),
        source_budget=dict(new_parser_state_bits=111,snapshot_bits=96,enable_control_bits=1,
            total_new_source_register_bits=208,new_RAM_target=0,new_DSP_target=0,new_clock_domains=0,
            new_UART_bit_receiver=False,new_hardware_snapshot_copy=False,
            actual_LUT_FF_DSP_and_routing_unknown=True,
            FW='softwareraw_work898B includes parser/service; replace with4MMIOread adapter and retained FWowner/guards; saving>=716B and final margin NOT proven until same originalLTO link',
            previous_full_C_link_over_bytes=716),
        validation_plan=['Freeze newRTL/TB and exact generated constants after current53/54 natural completion and root admission',
            'Small byte-stream original uart_speed_rx CRC/range/legacy output comparison, full signed16range and80singlebitCRC cases',
            'Bit receiver end-to-end FIRST/t_start timing: leadingnoise, back-to-backbytes, idle499/500/501us, noidlecontinuous, badstop/glitch, reset/CTRL atpartial/CRCedge',
            'Arbitrarily fill/stop both rawFIFOs and prove parserSEQ/CMD/TSindependent; WIT/rawbehavior miter unchanged',
            'Publish under every cut of fourMMIOreads; equal-seq generation consistency, mismatch/gap/MAXfreeze/flushreset rejects',
            'Same actualCguard finite contract but snapshotservice source; R0dead with continuing validP11 stream remainszero/locked',
            'RealC small mapping only after admission: modes/FF/reset/noBSRAM/noDSP/50MHz estimate and exact original flags fullFWLTO/stack/BSS',
            'Re-run selected completeCPU busy/350mspublication and actualphysical500msstop beforedeployment'],
        no_EDA_started=True,RTL_materialized=False,automatic_SYN_PNR=False,deployment_ready=False)
    save(out/'result.json',plan)
    save(REPO/'docs/4_design/evidence/g2_20261023/P11_parser_offload_contract_plan_20261026.json',plan)
    print(json.dumps(dict(output=str(out),CSR=[hex(0x6000+r['off'])for r in NEW_REGS],RTL_materialized=False)))

def require_completed_routes():
    import psutil
    # A source-generation gate, not permission to start an EDA process.
    for name in('SLF128_storage_place1_pnr53','SLF128_storage_place2_pnr54'):
        d=BASE/name;runtime=d/'tool_runtime.json'
        assert runtime.is_file(),str(runtime)+' missing; do not materialize yet'
        r=json.loads(runtime.read_text());assert r.get('tool_exit')is not None and r.get('ended_at'),(name,'not final')
        assert r.get('cancelled')is False,(name,'not naturally complete')
        pid=r.get('pid')or r.get('tool_pid');created=r.get('created_epoch')or r.get('tool_created_epoch')
        if created is None:created=datetime.fromisoformat(r['started_at']).timestamp()
        if pid and psutil.pid_exists(int(pid)):
            p=psutil.Process(int(pid));assert abs(p.create_time()-float(created))>1.0,(name,'original process still alive')

def materialize(source,out,acknowledged):
    assert acknowledged,'root authorization for post-route materialization required'
    require_completed_routes();contract=source_contract(source);assert not out.exists();shutil.copytree(source,out)
    before={str(p.relative_to(out)):p.read_bytes()for p in out.rglob('*')if p.is_file()}
    def patch(rel,fn):
        p=out/rel;t=read_text(p);n=fn(t);assert t!=n,rel;p.write_text(n,encoding='utf8')
    def imu(t):
        t=unique(t,'    output reg         irq,              // 单拍：突发首字节到达',
            '    output wire        tap_valid, tap_first,\n    output wire [7:0]  tap_data,\n    output wire [31:0] tap_first_start_us,\n    output reg         irq,              // 单拍：突发首字节到达')
        return unique(t,'    wire        rf_ready;',
            '    wire        rf_ready;\n    assign tap_valid=push;\n    assign tap_data=push_d[7:0];\n    assign tap_first=push_d[8];\n    assign tap_first_start_us=tpush_d;')
    patch('nav/rtl/io/imu_uart.v',imu)
    def top(t):
        t=unique(t,'    wire [31:0] iu_ts, iu_ovf;',
            '    wire [31:0] iu_ts, iu_ovf;\n    wire iu_tap_valid,iu_tap_first;\n    wire [7:0] iu_tap_data;\n    wire [31:0] iu_tap_first_ts;')
        t=unique(t,'        .st_ovf(iu_ovf), .irq(iu_irq),',
            '        .st_ovf(iu_ovf), .irq(iu_irq),\n        .tap_valid(iu_tap_valid),.tap_first(iu_tap_first),.tap_data(iu_tap_data),.tap_first_start_us(iu_tap_first_ts),')
        return unique(t,'    assign i_IMU_STAT_OVF = iu_ovf;','    assign i_IMU_STAT_OVF = iu_ovf;\n'+TOP)
    patch('nav/rtl/top/nav_subsystem.v',top)
    new=out/'nav/rtl/io/p11_command_snapshot.v';new.write_text(PARSER,encoding='utf8')
    import yaml
    cfgroot=out/'configuration_source';cfgpath=cfgroot/'config/regmap.yaml'
    cfg=yaml.safe_load(read_text(cfgpath));next(b for b in cfg['blocks']if b['name']=='IMU')['regs'].extend(copy.deepcopy(NEW_REGS))
    cfgpath.write_text(yaml.safe_dump(cfg,sort_keys=False,allow_unicode=True),encoding='utf8')
    spec=importlib.util.spec_from_file_location('P11_frozen_config_generator',REPO/'tools/gen_config.py');mod=importlib.util.module_from_spec(spec);spec.loader.exec_module(mod)
    mod.ROOT=cfgroot;mod.gen_regmap(cfg);mod.gen_regfile(cfg)
    for name in('nav_regfile.v','nav_regs_map.vh','nav_subsystem_rf.vh'):
        shutil.copyfile(cfgroot/'rtl/regs'/name,out/'nav/rtl/regs'/name)
    shutil.copyfile(cfgroot/'fw/include/nav_regs_map.h',out/'fw/include/nav_regs_map.h')
    # Generated old register addresses must be invariant, including SLF128 additions.
    def defines(t):return {n:int(v[:-1],16)for n,v in re.findall(r'^#define (NAV_[A-Z0-9_]+) (0x[0-9A-Fa-f]+u)$',t,re.M)}
    old=defines(read_text(source/'fw/include/nav_regs_map.h'));now=defines(read_text(out/'fw/include/nav_regs_map.h'))
    assert all(now.get(k)==v for k,v in old.items()),'old generated constants changed'
    experimental=out/'experimental_fw';experimental.mkdir();(experimental/'p11_snapshot_service.h').write_text(C_HEADER,encoding='utf8');(experimental/'p11_snapshot_service.c').write_text(C_SERVICE,encoding='utf8')
    t=read_text(out/'run.tcl').replace(source.as_posix(),out.as_posix()).replace(str(source),str(out))
    marker='add_file -type verilog {'+(out/'nav/rtl/io/imu_uart.v').as_posix()+'}'
    t=unique(t,marker,marker+'\nadd_file -type verilog {'+new.as_posix()+'}')
    (out/'run.tcl').write_text(t,encoding='utf8')
    changed=[];diff=[]
    for p in sorted(out.rglob('*')):
        if not p.is_file():continue
        rel=str(p.relative_to(out));original=before.get(rel)
        if original!=p.read_bytes():
            changed.append(rel)
            if original is not None:
                try:diff.extend(difflib.unified_diff(original.decode('utf-8-sig').splitlines(True),read_text(p).splitlines(True),fromfile='base/'+rel,tofile='candidate/'+rel))
                except UnicodeDecodeError:pass
    (out/'P11_offload.patch').write_text(''.join(diff),encoding='utf8')
    # Old provenance documents are retained, never reinterpreted as proof of this candidate.
    save(out/'P11_offload_preparation.json',dict(at=datetime.now().astimezone().isoformat(),source=str(source),
         changed_files=changed,original_source_contract=contract,
         candidate_inputs=[metadata(p)for p in sorted(out.rglob('*'))if p.is_file()],
         inherited_proof_documents_not_validating_new_candidate=True,
         parser_RTL_unvalidated=True,experimental_FW_not_selected=True,selected_CPU_BIN_INIT_unchanged=True,
         SYN_started=False,PNR_started=False,automatic_SYN_PNR=False,deployment_ready=False))
    print(json.dumps(dict(output=str(out),changed_files=changed,EDA_started=False),ensure_ascii=False))

if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--source',type=Path,default=SOURCE)
    p.add_argument('--out',type=Path,required=True);g=p.add_mutually_exclusive_group(required=True)
    g.add_argument('--plan',action='store_true');g.add_argument('--materialize',action='store_true')
    p.add_argument('--root-authorized-after-current-PNR',action='store_true');a=p.parse_args()
    if a.plan:make_plan(a.source,a.out)
    else:materialize(a.source,a.out,a.root_authorized_after_current_PNR)
