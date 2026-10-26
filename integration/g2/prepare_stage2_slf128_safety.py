"""Replay the three already-tested safety repairs on SLF128 prepared7.

Preparation only: preserve the ancestor and all proof runs, copy no old hardware
ancestor wholesale, and prevent this directory from launching an EDA build.
"""
from pathlib import Path
from datetime import datetime
import argparse,difflib,json,shutil
import psutil
from prepare_baseline_fusion_retime import metadata,relocate,same_bytes,save
from prepare_fifo_async_assert import transform_fifo,transform_ob
from prepare_mac_rx_boundary import OVERRIDE

BASE=Path('F:/nav_integration_g2_20261023')
SRC=BASE/'stage2_SLF128_forward_safeFW_prepared7'
OUT=BASE/'stage2_SLF128_forward_safeFW_prepared8'
SYN40=BASE/'baseline_fusion50_ring_MAX_syn40'
FIFO='nav/rtl/common/nav_async_fifo.v'
OB='nav/rtl/common/nav_fifo_ob.v'
MAC='nav/rtl/net/rgmii_mac100.v'
SUB='nav/rtl/top/nav_subsystem.v'
def load(p):return json.loads(p.read_text(encoding='utf-8-sig'))
def once(t,a,b):assert t.count(a)==1,(a,t.count(a));return t.replace(a,b,1)

def prepare():
 assert not OUT.exists(),OUT
 active=[]
 for p in psutil.process_iter(['pid','name','create_time','cmdline']):
  if (p.info['name']or'').lower() in {'xsim.exe','xsimk.exe','xelab.exe','xvlog.exe','vsim.exe','vsimk.exe'}:active.append(p.info)
 assert not active,('RTL inputs must not change while RTL tools run',active)
 before=load(SRC/'prepared_source_review.json');m=load(SRC/'manifest.json')
 for b in before['hardware_bindings']+before['all_FW_bindings']:assert metadata(Path(b['path']))==b,b
 names=[Path(p).relative_to(SRC)for p in m['source_files']];assert len(names)==63
 # The old validation copy remains in prepared7, with its actual source paths.
 shutil.copytree(SRC,OUT,ignore=shutil.ignore_patterns('validation','__pycache__'))
 old={n:(SRC/n).read_text(encoding='utf-8-sig')for n in [FIFO,OB,MAC,SUB,'g2_top.v']}
 prior_fifo=BASE/'fifo_async_assert_candidate3'
 prior_mac=BASE/'mac_rx_boundary_candidate2'
 proof_fifo=load(prior_fifo/'result.json');proof_mac=load(prior_mac/'result.json')
 assert proof_fifo['passed']and proof_mac['passed']
 for rel in [FIFO,OB,MAC]:
  assert same_bytes(SRC/rel,BASE/'baseline_fusion_only_syn23'/rel),('old proof input differs',rel)
 private=transform_ob(old[OB]).replace('module nav_fifo_ob #(','module nav_fifo_ob_async_assert #(',1)
 fifo=transform_fifo(old[FIFO]).replace('nav_fifo_ob #(','nav_fifo_ob_async_assert #(',1)+'\n'+private
 assert fifo==(prior_fifo/'nav_async_fifo.v').read_text(encoding='utf-8-sig')
 (OUT/FIFO).write_text(fifo,encoding='utf8')
 mac=once(old[MAC],'    input  wire       rst,\n','    input  wire       rst,\n    input  wire       rx_rst,       // existing RX-domain reset from top\n')
 marker='''        default: if (!dv_s) rs <= R_IDLE;
        endcase
    end
    initial begin rs = R_IDLE;'''
 mac=once(mac,marker,marker.replace('    end\n    initial',OVERRIDE+'    end\n    initial'))
 assert mac==(prior_mac/'rgmii_mac100.v').read_text(encoding='utf-8-sig')
 (OUT/MAC).write_text(mac,encoding='utf8')
 top=old['g2_top.v'];start=top.index('    rgmii_mac100 #(.CLK_HZ(CLK_HZ)) u_lmac (');end=top.index('    );',start)+len('    );')
 block=once(top[start:end],'.clk(clk), .rst(rst),','.clk(clk), .rst(rst), .rx_rst(l_rx_rst),')
 top=top[:start]+block+top[end:]
 # Net declaration only: the existing synchronized reset is unchanged.
 top=once(top,'    wire l_rx_rst, l_tx_rst;\n','')
 top=once(top,'    rgmii_mac100 #(.CLK_HZ(CLK_HZ)) u_lmac (','    wire l_rx_rst, l_tx_rst;\n    rgmii_mac100 #(.CLK_HZ(CLK_HZ)) u_lmac (')
 assert 'nav_rst_sync u_lrr (.clk(l_rx_clk), .arst(rst), .rst(l_rx_rst));'in top
 (OUT/'g2_top.v').write_text(top,encoding='utf8')
 sub=once(old[SUB],"assign i_IMU_RX_DATA  = {iu_valid, 22'd0, iu_first, iu_data};", "assign i_IMU_RX_DATA  = iu_valid ? {1'b1, 22'd0, iu_first, iu_data} : 32'd0;")
 (OUT/SUB).write_text(sub,encoding='utf8')
 changed=[str(n).replace('\\','/')for n in names if not same_bytes(SRC/n,OUT/n)]
 assert set(changed)=={FIFO,MAC,SUB,'g2_top.v'},changed
 assert same_bytes(SRC/OB,OUT/OB)
 fw=[p.relative_to(SRC)for p in sorted((SRC/'fw').rglob('*'))if p.is_file()]
 assert all(same_bytes(SRC/n,OUT/n)for n in fw)
 patch=''.join(''.join(difflib.unified_diff((SRC/n).read_text().splitlines(True),(OUT/n).read_text().splitlines(True),fromfile='prepared7/'+str(n),tofile='prepared8/'+str(n)))for n in names if not same_bytes(SRC/n,OUT/n))
 (OUT/'safety_delta.patch').write_text(patch,encoding='utf8')
 # Ancestor run.tcl was still bound to SYN40.  Rebind then disable explicitly;
 # the sole future combined build will regenerate all real inputs/constraints.
 original_tcl=(SRC/'run.tcl').read_text()
 rebound=original_tcl.replace(SYN40.as_posix(),OUT.as_posix()).replace(str(SYN40),str(OUT)).replace(SRC.as_posix(),OUT.as_posix()).replace(str(SRC),str(OUT))
 (OUT/'run.tcl').write_text('error {PREPARATION ONLY: CPU INIT/storage and actual constraints are not bound; no SYN/PNR authorized}\n'+rebound,encoding='utf8')
 m=relocate(m,SRC,OUT);m.update(safety_repairs=['invalid_IMU_returns_zero','common_reset_FIFO3','MAC_RX_wait_real_idle'],source_preparation_only=True,automatic_SYN=False,automatic_PNR=False,firmware_rebuild_pending=True,inherited_CPU_INIT_not_new_FW=True)
 save(OUT/'manifest.json',m)
 contract=load(SRC/'implementation_contract.json');contract.update(safety_repairs=dict(IMU='invalid read returns32\'d0; valid payload/first/valid unchanged; no pop or timestamp change',FIFO='async assertion of both endpoint pointer/synchronizer/FWFT validity; local release from existing nav_rst_sync; RAM and common nav_fifo_ob unchanged; common reset only',MAC='existing l_rx_rst drives synchronous last-priority R_BAD/hold_v0/rx_valid0; after a stopped RX clock it is held until local edges; wait real DV idle before next preamble',root_reset='u_rs from ~key_rst_n|~pll_lock; u_lrr RX and u_ltr TX from rst, host reset also from rst; CPU HOLD resets CPU only',source_proof_reuse='FIFO/MAC text exactly equals archived tested candidate3/candidate2, with unchanged immediate net_port/nav_cdc inputs; new g2_top only connects same reset plus declaration relocation',remaining=['new joint primitive reset types/INIT/CE and all new reset loads','physical reset assertion/release recovery-removal, pulse width, CDC/MTBF and IO','new firmware busy host-loss/real RK stop and physical stop latency']),selected_FW_unchanged_from_prepared7=True,RTL_validation_of_safety_rebase_not_yet_run=True,automatic_SYN=False,automatic_PNR=False)
 save(OUT/'implementation_contract.json',contract)
 review=relocate(before,SRC,OUT);review.update(candidate=str(OUT),ancestor=str(SRC),changed_RTL_from_prepared7=changed,changed_RTL=[str(n).replace('\\','/')for n in names if not same_bytes(SYN40/n,OUT/n)],hardware_bindings=[metadata(OUT/n)for n in names],all_FW_bindings=[metadata(OUT/n)for n in fw],source_frozen=True,RTL_TB_not_yet_frozen_together=True,RTL_validation_started=False,SYN_started=False,PNR_started=False,programmed=False)
 save(OUT/'prepared_source_review.json',review)
 result=dict(prepared=True,recorded_at=datetime.now().astimezone().isoformat(),source=str(SRC),candidate=str(OUT),changed_RTL=changed,source63_count=63,unchanged_other59_RTL=True,unchanged_all_FW=True,public_nav_fifo_ob_unchanged=True,FIFO_text_equals_proved_candidate3=True,MAC_text_equals_proved_candidate2=True,root_reset_sources_preserved=True,proof_paths=[str(prior_fifo/'result.json'),str(prior_mac/'result.json'),str(BASE/'preview_MAC_UDP_XT_reset_probe2/result.json'),str(BASE/'preview_full_G2_UART_CPU250_probe2_emptyIMU_modelsim/result.json')],prior_SLF_validation=str(BASE/'stage2_SLF128_targeted_validation2/job.json'),no_qualified_RTL_tools_active_at_prepare=True,original_prepared7_unchanged=True,selected_FW_source=str(BASE/'CPU250_host_drain_isolated_validation1/isolated/fw'),new_FW_compile_same_bytes_as_prepared7=True,tool_launched=False,SYN_started=False,PNR_started=False,programmed=False,remaining=contract['safety_repairs']['remaining'])
 for b in before['hardware_bindings']+before['all_FW_bindings']:assert metadata(Path(b['path']))==b,b
 save(OUT/'safety_rebase_review.json',result);return result

if __name__=='__main__':
 p=argparse.ArgumentParser();p.add_argument('--prepare',action='store_true');a=p.parse_args();assert a.prepare;print(json.dumps(prepare(),ensure_ascii=False,indent=2))
