"""Freeze separate TX-shadow RDEN and console posted-write candidates.

No simulator/EDA is launched. The console CPU-facing memory boundary is exposed
only in a test copy; production PicoRV32, CPU INIT and both bus FSMs stay intact.
"""
from pathlib import Path
from datetime import datetime
import argparse,json,re,shutil,difflib
from prepare_baseline_fusion_retime import metadata,same_bytes

BASE=Path('F:/nav_integration_g2_20261023')
SOURCE=BASE/'stage2B_bitmap_mailbox_slist_trig_console_bias_prepared1'
OUT=dict(txshadow=BASE/'stage2B_TXshadow_RDEN_prepared2',console=BASE/'stage2B_console_posted_prepared1')
HERE=Path(__file__).parent
def load(p):return json.loads(p.read_text(encoding='utf-8-sig'))
def save(p,x):p.write_text(json.dumps(x,ensure_ascii=False,indent=2)+'\n',encoding='utf8')
def once(t,a,b):assert t.count(a)==1,(a,t.count(a));return t.replace(a,b,1)

def txshadow(t):
 t=once(t,'    reg         mb_rd;','    reg         mb_rd;\n    wire        mb_shadow_read;')
 t=once(t,".txb_re(1'b1), .txb_addr",'.txb_re(mb_shadow_read), .txb_addr')
 t=once(t,'    reg [3:0] bs;', '''    reg [3:0] bs;
    // Demand read at acceptance E0; original B_RF consumes/ACKs at E1.
    // TX_DATA writes originate from the separate B_RF phase, never E0.
    assign mb_shadow_read = !rst && (bs == B_IDLE) && bus_req &&
        ({bus_addr[15:2],2'b00} == RA_MBX_TXB_Q);''')
 return t

def console(t):
 t=once(t,'    reg  [9:0]  con_ra;', '''    reg  [9:0]  con_ra;
    wire        con_read_needed;
    //21bits: accepted byte8 + ring byte address12 +valid1. INIT0 only;
    // protocol resets must not cancel an already-acknowledged byte.
    reg         con_post_valid=1'b0;
    reg [11:0]  con_post_addr;
    reg [7:0]   con_post_data;
    wire        con_commit = con_post_valid || (con_we && !con_read_needed);
    wire [11:0] con_commit_addr = con_post_valid ? con_post_addr : con_wp[11:0];
    wire [7:0]  con_commit_data = con_post_valid ? con_post_data : mem_wdata[7:0];
    always @(posedge clk) begin
        if (con_post_valid) con_post_valid <= 1'b0;
        if (con_we && con_read_needed) begin
            con_post_valid <= 1'b1;
            con_post_addr <= con_wp[11:0];
            con_post_data <= mem_wdata[7:0];
        end
    end''')
 t=once(t,'.we(con_we && con_wp[1:0] == b), .waddr(con_wp[11:2]), .wdata(mem_wdata[7:0]),',
             '.we(con_commit && con_commit_addr[1:0] == b), .waddr(con_commit_addr[11:2]), .wdata(con_commit_data),')
 t=once(t,".re(1'b1), .raddr(con_ra)",'.re(con_read_needed), .raddr(con_ra)')
 t=once(t,"    wire      d_con = (d_addr[15:12] == 4'hE);",'''    wire      d_con = (d_addr[15:12] == 4'hE);
    //E0 captures address, E1 reads, E2 captures data and original ACK.
    assign con_read_needed = !rst && d_con && (d_st == 2'd1);''')
 return t

def console_fixture(source,new,folder):
 # Actual complete nav_soc logic; only the CPU module boundary is abstracted.
 # This is a bus contract proof, not a substituted-CPU firmware test.
 for label,text in [('reference',source),('candidate',new)]:
  text=once(text,'module nav_soc #(','module console_'+label+' #(')
  text=once(text,'    input  wire        rst,','''    input  wire        rst,
    input wire proof_mem_valid,
    input wire [31:0] proof_mem_addr,proof_mem_wdata,
    input wire [3:0] proof_mem_wstrb,
    output wire proof_mem_ready,
    output wire [31:0] proof_mem_rdata,
    output wire proof_cpu_rst,''')
  block=re.search(r'    picorv32 #\(.*?\n    \);',text,re.S)[0]
  replacement='''    assign mem_valid=proof_mem_valid;
    assign mem_instr=1'b0;
    assign mem_addr=proof_mem_addr;
    assign mem_wdata=proof_mem_wdata;
    assign mem_wstrb=proof_mem_wstrb;
    assign proof_mem_ready=mem_ready;
    assign proof_mem_rdata=mem_rdata;
    assign proof_cpu_rst=cpu_rst;
    assign trap=1'b0;'''
  text=once(text,block,replacement)
  (folder/(label+'.v')).write_text(text,encoding='utf8')
 for name in ('nav_ram_sp.v','nav_ram_sdp.v'):
  shutil.copyfile(SOURCE/'nav/rtl/common'/name,folder/name)
 shutil.copyfile(HERE/'tb_stage2b_console_bus_miter.sv',folder/'tb.sv')
 return dict(files=['reference.v','candidate.v','nav_ram_sp.v','nav_ram_sdp.v','tb.sv'],top='tb_stage2b_console_bus_miter',
  cut='Only original PicoRV32 memory master is replaced by explicit test input ports in simulation copies. All actual nav_soc debug/ready/console/RAM/hold FSMs are retained. Inputs obey CPU reset and held-request handshakes; no force/internal-state initialization.',
  pending_proofs=['Complete actual CPU-boundary/debug bus miter, including reset/stopped clock','Reachable assertion: posted_valid implies no new CPU console request at its commit edge','Actual physical per-lane no CEA/CEB conflict, INIT and unchanged read/ACK edge'])

def regfile_instance(text):
 body=re.search(r'module nav_regfile\s*\((.*?)\n\);',text,re.S)[1]
 ports=[]
 connected={'clk':'clk','rst':'rst','wr':'rf_wr','rd':'rf_rd','addr':'a_q','wdata':'wd_q','rdata':'rf_rdata','hit':'rf_hit',
  'p_MBX_TX_DATA':'tx_we','p_MBX_TX_GO':'tx_go','r_MBX_TX_LEN':'tx_len','r_MBX_TXB_ADDR':'txb_addr','i_MBX_TXB_Q':'txb_q','i_MBX_TX_STAT':'{16\'d0,2\'d0,tx_words,3\'d0,1\'b0}', 'i_MBX_TX_SEQ':'tx_seq'}
 for line in body.splitlines():
  m=re.match(r'\s*(input|output)\s+(?:wire|reg)\s+(?:\[[^]]+\]\s+)?(\w+)\s*,?',line)
  if not m:continue
  direction,name=m.groups();ports.append('.'+name+'('+connected.get(name,"'0"if direction=='input'else'')+')')
 assert len(ports)>150
 return 'nav_regfile rf('+','.join(ports)+');'

def tx_fixture(folder,prepared):
 regfile=(SOURCE/'nav/rtl/regs/nav_regfile.v').read_text();inst=regfile_instance(regfile)
 # The RF sub-flow is copied for addresses that actually take it. Unrelated
 # long read windows are outside this cut; the new read gate is false there.
 controller='''
localparam B_IDLE=0,B_RF=1,B_END=5;reg[3:0]bs;
reg rf_wr,rf_rd;reg[15:0]a_q;reg[31:0]wd_q;wire[31:0]rf_rdata;wire rf_hit;
always@(posedge clk)begin
 bus_ack<=0;rf_wr<=0;rf_rd<=0;
 if(rst)begin bs<=B_IDLE;end
 else case(bs)
 B_IDLE:if(bus_req)begin
  a_q<={bus_addr[15:2],2'b00};wd_q<=bus_wdata;
  if(bus_we)rf_wr<=1;else rf_rd<=1;bs<=B_RF;
 end
 B_RF:begin bus_rdata<=rf_rdata;bus_ack<=1;bs<=B_END;end
 B_END:bs<=B_IDLE;
 default:bs<=B_IDLE;
 endcase
end
'''
 for label,read in [('reference',"1'b1"),('candidate',"!rst && bs==B_IDLE && bus_req && {bus_addr[15:2],2'b00}==16'h1240")]:
  wrapper='''module shadow_'''+label+'''(input clk,rst,
input m0_req,m0_we,input[15:0]m0_addr,input[31:0]m0_wdata,output m0_ack,output[31:0]m0_rdata,
input m1_req,m1_we,input[15:0]m1_addr,input[31:0]m1_wdata,output m1_ack,output[31:0]m1_rdata);
wire bus_req,bus_we;wire[15:0]bus_addr;wire[31:0]bus_wdata;reg bus_ack;reg[31:0]bus_rdata;
nav_bus_arb arb(clk,rst,m0_req,m0_we,m0_addr,m0_wdata,m0_ack,m0_rdata,m1_req,m1_we,m1_addr,m1_wdata,m1_ack,m1_rdata,bus_req,bus_we,bus_addr,bus_wdata,bus_ack,bus_rdata);
'''+controller+'''
wire tx_we,tx_go;wire[31:0]tx_len,txb_addr,txb_q,tx_seq;wire[9:0]tx_words;
wire shadow_read='''+read+''';
'''+inst+'''
host_mbx #(.RX_AW(12),.DQ_AW(4),.TX_WAW(9)) mbx(
 .clk(clk),.rst(rst),.e_valid(1'b0),.e_data(8'd0),.e_last(1'b0),.e_err(1'b0),.e_src_ip(32'd0),.e_src_port(16'd0),.e_len(16'd0),.e_ts(32'd0),
 .inj_we(1'b0),.inj_wdata(32'd0),.inj_go(1'b0),.inj_len(16'd0),.now_us(32'd0),
 .txb_re(shadow_read),.txb_addr(txb_addr[8:0]),.txb_q(txb_q),.tx_seq(tx_seq),
 .rx_avail(),.rx_len(),.rx_pkts(),.rx_ts(),.rx_src_ip(),.rx_src_port(),.rx_rd(1'b0),.rx_rd_ack(),.rx_word(),.rx_pop(1'b0),.rx_irq(),.st_drop(),
 .tx_we(tx_we),.tx_wdata(wd_q),.tx_go_in(tx_go),.tx_len_in(tx_len[15:0]),.tx_words(tx_words),.tx_go(),.tx_len(),.tx_busy(1'b0),.pl_re(1'b0),.pl_addr(11'd0),.pl_q());
endmodule
'''
  (folder/(label+'.v')).write_text(wrapper,encoding='utf8')
 for rel in ['nav/rtl/io/nav_bus_arb.v','nav/rtl/regs/nav_regfile.v','nav/rtl/net/host_mbx.v','nav/rtl/common/nav_ram_sdp.v']:
  shutil.copyfile(prepared/rel,folder/Path(rel).name)
 shutil.copyfile(HERE/'tb_stage2b_TXshadow_bus_miter.sv',folder/'tb.sv')
 return dict(files=['reference.v','candidate.v','nav_bus_arb.v','nav_regfile.v','host_mbx.v','nav_ram_sdp.v','tb.sv'],top='tb_stage2b_TXshadow_bus_miter',
  cut='Actual two-master arbiter, complete regfile and complete host_mbx are used. Only the ordinary RF bus subflow is extracted; other read-window state branches are outside this cut, with the new read gate false. This is not whole-nav or UART physical throughput proof.',
  pending_proofs=['RF branch cut/bus-ack/data miter and demanded read timing','Actual full SYN after merge: top read decode plus no enabled write/read on all physical shadow leaves','Other bus-window/IRQ/reset full-consumer regressions'])

def prepare(profile):
 out=OUT[profile];assert not out.exists();m=load(SOURCE/'manifest.json')
 names={Path(x).relative_to(SOURCE).as_posix()for x in m['source_files']};assert len(names)==63
 shutil.copytree(SOURCE,out,ignore=shutil.ignore_patterns('validation','__pycache__'))
 rel='nav/rtl/top/nav_subsystem.v'if profile=='txshadow'else'nav/rtl/soc/nav_soc.v'
 old=(SOURCE/rel).read_text();new=(txshadow if profile=='txshadow'else console)(old)
 (out/rel).write_text(new,encoding='utf8');(out/'consumer.delta.patch').write_text(''.join(difflib.unified_diff(old.splitlines(True),new.splitlines(True),fromfile='stage2B/'+rel,tofile=profile+'/'+rel)),encoding='utf8')
 inj='nav/rtl/net/host_mbx.v'
 inj_old=(SOURCE/inj).read_text();inj_new=once(inj_old,'nav_ram_sdp #(.DW(32), .AW(TX_WAW), .STAGE2B_BLOCK(1)) u_injb','nav_ram_sdp #(.DW(32), .AW(TX_WAW)) u_injb')
 (out/inj).write_text(inj_new,encoding='utf8')
 (out/'keep_injection_distributed.delta.patch').write_text(''.join(difflib.unified_diff(inj_old.splitlines(True),inj_new.splitlines(True),fromfile='stage2B/'+inj,tofile='keep_distributed/'+inj)),encoding='utf8')
 changed=sorted(n for n in names if not same_bytes(SOURCE/n,out/n));assert changed==sorted([rel,inj])
 assets=[p.relative_to(SOURCE)for p in SOURCE.rglob('*')if p.is_file()and'validation'not in p.relative_to(SOURCE).parts and p.suffix.lower()in{'.hex','.mem','.mif','.bin','.h','.vh'}]
 assert all(same_bytes(SOURCE/n,out/n)for n in assets)
 m.update(source_files=[str(out/Path(x).relative_to(SOURCE))for x in m['source_files']],frozen_source_build=str(SOURCE),candidate='stage2B_'+profile+'_collision_exclusion',automatic_PNR=False,source_preparation_only=True,synthesis_complete=False,programmed=False)
 save(out/'manifest.json',m);val=out/'validation';val.mkdir()
 spec=tx_fixture(val,out)if profile=='txshadow'else console_fixture(old,new,val)
 contract=dict(profile=profile,source=str(SOURCE),candidate=str(out),changed_RTL=changed,unchanged_other61_RTL=True,original_Firmware_assets_parameters_clocks_weights_125Hz_capacity_unchanged=True,
  architecture='Same50MHz clock. No derived/half-cycle clock, byte/word capacity or INIT change. No unsupported Read-before-Write mode selected.',
  observable_timing='TXB_Q E0 demand/E1 original ACK, including data returned by writes to the read-only Q address'if profile=='txshadow'else'Console debug E0 address/E1 demand/E2 original ACK; original CPU per_ready/con_wp unchanged',
  write_commit_change='None'if profile=='txshadow'else'CPU log byte accepted at demanded-read E1 is queued21bits and committed at next available edgeE2, including rst/HOLD or clock resume; con_post_valid has only INIT0 and no protocol reset.',
  conflict_granularity='Exclusion by no simultaneous physical read/write enables; does not rely on logical-address inequality. Future mapping must check actual14bit AD[13:4] and all split lanes.',
  input_contract='CPU/debug/nav request and addresses held until ACK; CPU master respects reset/HOLD. Console pending->no next CPU write requires precise req/per_ready/sel_per reachable-state proof, including reset transition; not yet proved.',
  injection_retained_distributed=True,injection_GO_length_ACK_or_capacity_changed=False,combined_B_route_BSRAM_prediction=333,prediction_not_actual_measurement=True,
  consumer_bus_miter=spec,RTL_not_compiled=True,RTL_not_simulated=True,actual_mapping_not_verified=True,physical_collision_contract_closed=False,all_tools_started=False,programmed=False)
 save(out/'consumer_contract.json',contract)
 spec.update(candidate=str(out),RTL_simulator='xvlog/xelab/xsim only after explicit root coordination',minimum_physical_bytes=2*1024**3,minimum_commit_bytes=4*1024**3,automatic_run=False)
 spec['commands']=[['xvlog',['-sv',*spec['files']]],['xelab',['-debug','off','-timescale','1ns/1ps',spec['top'],'-s','consumer_miter']],['xsim',['consumer_miter','-runall']]]
 save(val/'entry.json',spec)
 r=dict(prepared=True,profile=profile,source=str(SOURCE),candidate=str(out),changed_RTL=changed,all_other_source_files_byte_identical=True,contract=str(out/'consumer_contract.json'),validation_entry=str(val/'entry.json'),
  hardware_bindings=[metadata(out/n)for n in sorted(names)],validation_bindings=[metadata(p)for p in sorted(val.rglob('*'))if p.is_file()],simulation_started=False,Gowin_started=False,PNR_started=False,programmed=False)
 save(out/'prepared_source_review.json',r);return r

if __name__=='__main__':
 p=argparse.ArgumentParser();p.add_argument('--profile',choices=list(OUT),required=True);p.add_argument('--prepare',action='store_true');a=p.parse_args();assert a.prepare
 r=prepare(a.profile);print(json.dumps({k:r[k]for k in ['prepared','profile','candidate','changed_RTL','contract','validation_entry','simulation_started','Gowin_started','PNR_started']},ensure_ascii=False))
