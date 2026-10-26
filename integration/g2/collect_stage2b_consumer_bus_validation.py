"""Collect the two frozen consumer bus miters; never launch tools."""
from pathlib import Path
from datetime import datetime
import argparse,json,re,psutil
from prepare_baseline_fusion_retime import metadata

BASE=Path('F:/nav_integration_g2_20261023')
GIB=1024**3
def load(p):return json.loads(p.read_text(encoding='utf-8-sig'))
def identity(pid,created):
 try:
  q=psutil.Process(pid);actual=q.create_time();same=abs(actual-created)<.01
 except(psutil.NoSuchProcess,psutil.AccessDenied):actual=None;same=False
 return dict(pid=pid,original_created_epoch=created,current_created_epoch=actual,original_identity_exited=not same)

def collect(profile,candidate_name,probe):
 candidate=BASE/candidate_name;folder=BASE/probe
 r=load(folder/'result.json');prepared=load(candidate/'prepared_source_review.json')
 assert r['passed'] and r['phase']=='finite_bus_miter_complete'
 assert all(x['exit_code']==0 for x in r['runtimes'])
 bindings=prepared['hardware_bindings']+prepared['validation_bindings']
 if profile=='txshadow':bindings+=load(Path(r['validation'])/'input_bindings.json')['inputs']
 for row in bindings:assert metadata(Path(row['path']))==row,('frozen input changed',row)
 text=(folder/'xsim.log').read_text(errors='replace')
 marker='TEST PASSED TXSHADOW BUS' if profile=='txshadow' else 'TEST PASSED CONSOLE BUS'
 assert marker in text and not re.search(r'\bFatal:',text)
 processes=[identity(r['controller_pid'],r['controller_created_epoch'])]
 for t in r['runtimes']:
  assert t['admission_physical_bytes']>=2*GIB and t['admission_commit_bytes']>=4*GIB
  processes.append(identity(t['wrapper_pid'],t['wrapper_created_epoch']))
  processes.extend(identity(q['pid'],q['created_epoch']) for q in t.get('processes',[]))
 assert all(x['original_identity_exited'] for x in processes)
 warnings=[]
 for p in sorted(folder.glob('*.log')):
  warnings.extend(dict(log=str(p),line=line) for line in p.read_text(errors='replace').splitlines() if 'WARNING:' in line)
 return dict(profile=profile,passed=True,candidate=str(candidate),changed_RTL=prepared['changed_RTL'],actual_result=str(folder/'result.json'),actual_xsim_log=str(folder/'xsim.log'),counts=r['counts'],actual_PASS_line=r['actual_PASS_line'],finish_lines=[x for x in text.splitlines() if x.startswith('$finish called')],tools=r['runtimes'],original_process_identities=processes,warnings=warnings,frozen_hardware_and_fixture_unchanged=True,cut=r['cut'],observed_physical_enable_exclusion_assertion='No demanded read and accepted/posted write simultaneously, independent of AD[13:4]',actual_mapped_CEA_CEB_and_OCE_not_yet_verified=True)

def main(out):
 rows=[collect('console','stage2B_console_posted_prepared1','stage2B_console_bus_probe1'),collect('txshadow','stage2B_TXshadow_RDEN_prepared2','stage2B_TXshadow_bus_probe2')]
 assert rows[0]['tools'][-1]['ended_at']<rows[1]['tools'][0]['started_at']
 first=load(BASE/'stage2B_TXshadow_bus_probe1/result.json')
 assert not first['passed'] and first['tool_runtime']['tool']=='xelab' and first['tool_runtime']['exit_code']==1
 failed_text=(BASE/'stage2B_TXshadow_bus_probe1/1_xelab_stdout.log').read_text(errors='replace')
 assert '[VRFC 10-2063]' in failed_text and 'nav_fifo' in failed_text
 dependency=load(BASE/'stage2B_TXshadow_validation_dependency2/preparation.json')
 assert not dependency['source_RTL_or_TB_expressions_modified']
 result=dict(recorded_at=datetime.now().astimezone().isoformat(),passed=True,profiles=rows,tools_sequential_one_at_a_time=True,all_original_tool_identities_exited=True,first_TX_probe_preserved=dict(path=str(BASE/'stage2B_TXshadow_bus_probe1'),classification='elaboration dependency missing nav_fifo; no functional simulation entered',error_lines=[x for x in failed_text.splitlines() if '[VRFC 10-2063]' in x]),validation_dependency_correction=dependency,TX_full_512_words_and_post_reset_readback=True,TX_extra_full_write_ignored_and_RO_Q_write_return_preserved=True,TX_runtime_reset_not_clock_stop_tested=True,console_full4096_byte_wrap_and1024_word_readback=True,console_posted_byte_preserved_across_runtime_reset_and_stopped_clock=True,console_ACK_and_independent_prewrite_data_scoreboard=True,small_Boolean_control_obligations=str(BASE/'stage2B_BSRAM_mode_collision_review1/consumer_control_obligations2.json'),injection_buffer_retained_distributed=True,combined_B_BSRAM333_is_prediction=True,no_Gowin_synthesis_or_PNR_started=True,no_programming=True,no_SHA=True,not_verified=['actual C primitive CEA/CEB/OCE and per-lane AD[13:4] exclusion','actual post-merge storage modes and INIT','whole G2/Pico firmware/UART physical throughput','setup/hold/reset recovery/removal/physical collision electrical behavior'])
 out.write_text(json.dumps(result,ensure_ascii=False,indent=2)+'\n',encoding='utf8')
 print(json.dumps(dict(passed=True,output=str(out),all_original_identities_exited=True,profiles=[dict(profile=x['profile'],counts=x['counts'],warnings=len(x['warnings'])) for x in rows]),ensure_ascii=False))

if __name__=='__main__':
 p=argparse.ArgumentParser();p.add_argument('--out',type=Path,required=True);a=p.parse_args();main(a.out)
