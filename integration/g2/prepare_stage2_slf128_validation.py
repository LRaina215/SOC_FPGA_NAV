"""Prepare frozen targeted SLF fixtures, no vector generator or RTL tool run."""
from pathlib import Path
from datetime import datetime
import argparse,importlib.util,json,re,shutil
from prepare_baseline_fusion_retime import metadata
from prepare_stage2_slf128_forward import ROOT,BASE,OUT,once,load,save

def prepare():
 val=OUT/'validation';assert not val.exists();val.mkdir()
 # A literal source cut of the new frontend control, with arbitrary external
 # matcher ACK/busy sequences. It proves the local ownership/exclusion contract,
 # not complete NAV bus reachability or firmware execution.
 top=(OUT/'nav/rtl/top/nav_subsystem.v').read_text()
 logic=re.search(r'    always @\(posedge clk\) begin\n        if \(rst\) begin mt_eval_st.*?\n    end\n    assign i_MATCH_STAT',top,re.S)[0].rsplit('\n    assign i_MATCH_STAT',1)[0]
 cut='''module slf_frontend_cut(input clk,rst,input[31:0]r_MATCH_SM_ADDR,r_MATCH_DBG_ADDR,wd_q,
input p_MATCH_SM_WDATA,p_MATCH_DBG_CMD,p_MATCH_CMD,input mt_busy,mt_dbg_ack,mt_eval_done,input[17:0]mt_dbg_q,
output reg mt_dbg_req,mt_dbg_done,output ram_we,output[13:0]mt_dbg_address,output reg[17:0]mt_dbg_word,
output[31:0]s_MATCH_ADDR_ERR,i_MATCH_PROFILE);
localparam SLF_LOG2N=7;reg mt_eval_st,mt_dbg_pending;
wire mt_pw_valid=(r_MATCH_SM_ADDR>>(2*SLF_LOG2N))==0;
wire mt_dbg_valid=(r_MATCH_DBG_ADDR>>(2*SLF_LOG2N))==0;
assign ram_we=p_MATCH_SM_WDATA&&mt_pw_valid;
assign s_MATCH_ADDR_ERR={30'd0,(p_MATCH_DBG_CMD&&wd_q[0]&&!mt_dbg_valid),(p_MATCH_SM_WDATA&&!mt_pw_valid)};
assign i_MATCH_PROFILE=32'h534c0100|SLF_LOG2N;
'''
 # Output address is the actual production snapshot register.
 cut=cut.replace('output[13:0]mt_dbg_address','output reg[13:0]mt_dbg_address')+logic+'\nendmodule\n'
 (val/'frontend_cut.v').write_text(cut,encoding='utf8')
 shutil.copyfile(Path(__file__).with_name('tb_stage2_slf_frontend.sv'),val/'tb_frontend.sv')
 tb=(ROOT/'rtl/match/tb/tb_slf.sv').read_text()
 tb=once(tb,'module tb_slf;','module tb_slf #(parameter integer SLF_LOG2N=7);')
 tb=once(tb,'always #5 clk = ~clk;','always #10 clk = ~clk;')
 tb=once(tb,'NSLOT = 65536','NSLOT = (1<<(2*SLF_LOG2N))')
 tb=tb.replace('0:2*65536-1','0:2*NSLOT-1').replace('logic [15:0] pw_addr','logic [2*SLF_LOG2N-1:0] pw_addr').replace('logic [15:0] dbg_addr','logic [2*SLF_LOG2N-1:0] dbg_addr')
 tb=once(tb,'slf_match #(.SIN_INIT','slf_match #(.LOG2N(SLF_LOG2N), .SIN_INIT')
 tb=tb.replace("[15:0]; pw_data", "[2*SLF_LOG2N-1:0]; pw_data").replace('dbg_addr <= a[15:0]','dbg_addr <= a[2*SLF_LOG2N-1:0]').replace('dbg_addr <= prior[2*i][15:0]','dbg_addr <= prior[2*i][2*SLF_LOG2N-1:0]')
 tb=once(tb,'        int fd;','        int fd;\n        int update_cx,update_cy;')
 tb=once(tb,'            // 并图 + 似然场','            update_cx=lcx;update_cy=lcy;\n            // 并图 + 似然场')
 tb=once(tb,'            repeat (5) @(posedge clk);','            #1;lcx=32\'h12345678;lcy=-987654; // changes after accepted UPDATE must not redirect LF\n            repeat (5) @(posedge clk);')
 tb=once(tb,'            wait (!busy);','            wait (!busy);\n            lcx=update_cx;lcy=update_cy;')
 # Case inequality explicitly rejects unknowns in compared masked data.
 tb=tb.replace(' & smm[k*NSLOT + a]) != 0)',' & smm[k*NSLOT + a]) !== 24\'d0)')
 clearcheck='''        // A complete CLEAR includes highest slot and every tagged wrap slot.
        for(int slot=0;slot<NSLOT;slot++)begin
            dbg_req<=1;dbg_addr<=slot[2*SLF_LOG2N-1:0];
            do begin @(posedge clk);#1;end while(!dbg_ack);
            dbg_req<=0;
            if(dbg_q!==18'd0)$fatal(1,"clear slot%0d not zero",slot);
        end
'''
 tb=once(tb,'        for (int i = 0; i < n_prior; i++) begin',clearcheck+'        for (int i = 0; i < n_prior; i++) begin')
 tb=once(tb,'        else $display("TEST FAILED errs=%0d", errs);','        else $fatal(1,"TEST FAILED errs=%0d", errs);')
 tb=once(tb,'endmodule','    initial begin #2000000000;$fatal(1,"finite SLF input watchdog");end\nendmodule')
 (val/'tb_slf128.sv').write_text(tb,encoding='utf8')
 # Freeze a separate complete model-source tree for deterministic input creation.
 tree=val/'execution_tree'
 for folder in ['model','sim/closed_loop','sim/gen']:(tree/folder).mkdir(parents=True)
 for folder in ['model','sim/closed_loop']:
  for p in (ROOT/folder).glob('*.py'):shutil.copyfile(p,tree/folder/p.name)
 for p in (OUT/'host_profile/model').glob('*.py'):shutil.copyfile(p,tree/'model'/p.name)
 gen=(ROOT/'sim/gen/gen_slf.py').read_text()
 gen=once(gen,'MOUNT_Z = 200','MOUNT_Z = 200\nFORWARD_MM=1500\ndef forward_cells(yaw_deg):\n    a=int(round(yaw_deg*65536/360))&65535\n    den=16384*65536\n    def rnd(v):return (1 if v>=0 else -1)*((abs(v)+den//2)//den)\n    return rnd(FORWARD_MM*INV_RES*int(nf.SIN[((a>>6)+256)&1023])),rnd(FORWARD_MM*INV_RES*int(nf.SIN[a>>6]))')
 gen=once(gen,'        smap.build_lf(lcx, lcy)','        fx,fy=forward_cells(yaw)\n        lcx+=fx;lcy+=fy\n        smap.build_lf(lcx, lcy)')
 (tree/'sim/gen/gen_slf128_forward.py').write_text(gen,encoding='utf8')
 entries=dict(candidate=str(OUT),automatic_run=False,no_tools_started=True,minimum_physical_bytes=2*1024**3,minimum_commit_bytes=4*1024**3,profiles=[dict(name='frontend',files=['frontend_cut.v','tb_frontend.sv'],top='tb_stage2_slf_frontend',scope='Literal actual frontend source cut; arbitrary old ACK alignments, command snapshot and prior highbit exclusion. Not full NAV bus proof.'),dict(name='SLF128',model_env=dict(SLF_SN_LOG2='7',SLF_W18='1'),top='tb_slf',top_parameter='SLF_LOG2N=7',scope='Five original actual point frames/245 candidates/all16384 words, forwarded LF center, post-command CSR change, complete clear, prior scoring/hold; expected from frozen integer model.'),dict(name='SLF256_control_only',model_env=dict(SLF_SN_LOG2='8',SLF_W18='1'),top='tb_slf',top_parameter='SLF_LOG2N=8',scope='Explicit validation-only256 geometry, never a production/build alternative.')],vector_generator=str(tree/'sim/gen/gen_slf128_forward.py'),RTL_files=[str(OUT/('nav/rtl/'+p)) for p in ['common/nav_ram_sdp.v','match/slf_extract.v','match/slf_lf.v','match/slf_match.v']],sin_lut=str(OUT/'sin_lut.hex'),run_order='Freeze all candidates before any RTL tool, then one profile at a time after2/4GiB admission; no automatic retry',pending=['RV safe/profileFW compile and full busy host-loss verification','whole NAV CSR/stop/radar regressions','actual combined storage/netlist/timing/board acceptance'])
 save(val/'entry.json',entries)
 save(val/'frozen_bindings.json',dict(inputs=[metadata(p) for p in sorted(val.rglob('*')) if p.is_file()],hardware_inputs=load(OUT/'prepared_source_review.json')['hardware_bindings']))
 return dict(candidate=str(OUT),validation=str(val),files=len(load(val/'frozen_bindings.json')['inputs']),RTL_TB_frozen_together=True,tools_started=False)

if __name__=='__main__':
 p=argparse.ArgumentParser();p.add_argument('--prepare',action='store_true');a=p.parse_args();assert a.prepare;print(json.dumps(prepare(),ensure_ascii=False))
