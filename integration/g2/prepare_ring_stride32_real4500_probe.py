"""Plan now; materialize and run are separate explicit actions.

No Verilog is written by --plan. Later --materialize requires the exact new
stride32 source directory and natural completion of PNR53/54. --start accepts
only that frozen validation directory and never starts SYN/PNR.
"""
from datetime import datetime
from pathlib import Path
import argparse
import difflib
import json
import re
import shutil
import subprocess
import sys
import time
import traceback

REPO_TOOLS = Path('D:/Desktop/SOC FPGA/nav/integration/g2')
if str(REPO_TOOLS) not in sys.path:
    sys.path.insert(0, str(REPO_TOOLS))
import prepare_ring_stride32_private_reader as candidate_generator
from prepare_baseline_fusion_retime import metadata, same_bytes


BASE = Path('F:/nav_integration_g2_20261023')
SOURCE = BASE / 'SLF128_storage_safeFW_syn43'
OLD_PROBE = BASE / 'observation_ring_real4500_probe1'
OLD_TB = OLD_PROBE / 'tb_model25_observation_ring_real4500.sv'
VIV = Path('E:/2026.1/Vivado/bin')
PYTHON = Path('D:/Anaconda/python.exe')
TOP = 'tb_observation_ring_real4500'
SNAP = 'ring_stride32_real4500_snap'
MATERIALIZE_TOKEN = 'freeze-only-after-natural-PNR53-PNR54-completion'
START_TOKEN = 'run-one-frozen-stride32-real4500-comparison'
CORE = ('fc_v4_top.v', 'fixed4500_weight_rom.v', 'control/fc_v4_round_sat.v')
TB_NAME = 'tb_ring_stride32_real4500.sv'


def read(path):
    return json.loads(path.read_text(encoding='utf-8-sig'))


def save(path, value):
    temp = path.with_suffix(path.suffix + '.tmp')
    temp.write_text(json.dumps(value, ensure_ascii=False, indent=2) + '\n', encoding='utf8')
    temp.replace(path)


def once(text, before, after):
    assert text.count(before) == 1, (before, text.count(before))
    return text.replace(before, after, 1)


def rename(text, module, suffix):
    text = re.sub(r'\b' + module + r'\b', module + '_' + suffix, text)
    return text.replace(module.upper() + '_V_INCLUDED', module.upper() + '_V_INCLUDED_' + suffix.upper())


# Stored Python strings only until the explicit materialization action.
EXTRA_MONITORS = r'''
 // Valid-slot ownership proves that accepted reads never depend on naturalX
 // RAM startup contents. Padding stays X; no TB assignment targets DUT RAM.
 integer normal_weight_reads=0,weight_frames=0,load_stops=0,clip_cases=0,deadline_hits=0;
 reg[40703:0]weight_seen=0;
 reg[15:0]gold_weights[0:40703];
 reg weight_pending=0;reg[15:0]weight_pending_addr=0;
 integer pad_slot,pad_lane,clip_j;
 initial $readmemh("src/mem/weights.mem",gold_weights);
 always @(posedge clk)begin
  if(!rst_n || !enable)begin weight_pending=0;normal_weight_reads=0;weight_seen=0;end
  else begin
   if(`RC.start)begin normal_weight_reads=0;weight_seen=0;end
   if({`RC.weights.en,`RC.weight_address}!=={`CC.weights.en,`CC.weight_address})
    $fatal(1,"stride32 real weight request/edge differs");
   weight_pending=`RC.weights.en;
   weight_pending_addr=`RC.weight_address;
   if(weight_pending)begin
    if($isunknown(weight_pending_addr)||weight_pending_addr>=40704)$fatal(1,"weight address out of real model");
    normal_weight_reads=normal_weight_reads+1;weight_seen[weight_pending_addr]=1'b1;
   end
   if(c_obs.g_ring.lane_write &&
      (c_obs.g_ring.write_address[4:0]>27 || c_obs.g_ring.write_address[7:5]>5))
    $fatal(1,"stride32 producer touched hole/outside slot");
   if(freeze || read_advance)begin
    if(c_obs.g_ring.read_address[4:0]>27 || c_obs.g_ring.read_address[7:5]>5)
     $fatal(1,"stride32 consumed hole/outside slot");
    if($isunknown(read_data))$fatal(1,"accepted reader consumed uninitialized RAM");
    if(r_read_data!==read_data)$fatal(1,"accepted old/new paired reader word differs");
   end
   if(freeze!==r_freeze)$fatal(1,"old/new freeze edge moved");
   if(read_advance && !(c_policy.state==2 && c_policy.load_index!=167 &&
      enable && sensor_ok && !stop_latched && c_fault==0 && !c_policy.deadline_missed))
    $fatal(1,"private cursor advanced outside original accepted LOAD");
  end
 end
 always @(negedge clk)if(rst_n&&enable)begin
  if(weight_pending)begin
   if($isunknown({`RC.weight_q,`CC.weight_q}) ||
      `RC.weight_q!==`CC.weight_q || `RC.weight_q!==gold_weights[weight_pending_addr])
    $fatal(1,"actual packed weight value/hold edge differs from40704 gold");
  end
  if(`RC.done)begin
   if(normal_weight_reads!=40704 || weight_seen!=={40704{1'b1}})
    $fatal(1,"inference did not visit all40704 real weights exactly once");
   weight_frames=weight_frames+1;
  end
 end
 task stop_load(input integer kind,input integer index);begin
  wait(r_policy.state==2 && r_policy.load_index==index);
  @(negedge clk);
  if(kind==0)begin
   rst_n=0;repeat(10)@(negedge clk);sensor_ok=1;enable=1;epoch=epoch+1;rst_n=1;
  end else if(kind==1)rearm();
  else begin
   sensor_ok=0;repeat(10)@(negedge clk);
   if(r_av||c_av||r_fault[0]!==1||c_fault[0]!==1)$fatal(1,"LOAD sensor stop failed");
   rearm();
  end
  load_stops=load_stops+1;
  $display("STRIDE32_LOAD_STOP kind=%0d index=%0d",kind,index);
  wait(c_frames>=1);
 end endtask
 task clip_load(input integer mode);begin
  @(negedge clk);enable=0;manual_mode=1;
  if(mode==0)manual_actions={-24'sd409601,-24'sd409600,-24'sd409599,
                            24'sd409601,24'sd409600,24'sd409599};
  else manual_actions={24'sh800000,24'sh7fffff,-24'sd1,24'sd1,24'sd0,24'sd500000};
  repeat(10)@(negedge clk);sensor_ok=1;epoch=epoch+1;enable=1;
  // All168 real core writes are compared, then abort before START. Extreme
  // clipping vectors are not asserted to be numerically admissible inference.
  wait(r_policy.state==3);@(negedge clk);
  if(load_count!=168)$fatal(1,"clip fixture missing complete168word LOAD");
  for(clip_j=0;clip_j<6;clip_j=clip_j+1)begin
   if(loaded[22+clip_j]!==clip(manual_actions[clip_j*24+:24]) ||
      loaded[162+clip_j]!==manual_actions[clip_j*24+:24])
    $fatal(1,"current-only clip/raw newest history boundary");
  end
  clip_cases=clip_cases+1;enable=0;manual_mode=0;
  repeat(10)@(negedge clk);sensor_ok=1;epoch=epoch+1;enable=1;
  wait(c_frames>=1);
 end endtask
'''


DEADLINE_PAIR = r'''
 // Independent real old/new pair: only the instance PERIOD is shortened.
 // 10 divided lanes*66 +18 direct lanes =678 producer clocks. From capture
 // to LOAD0 is682 clocks; PERIOD709 naturally expires at LOAD27.
 // Normal numerical pair above remains50MHz/400000clocks/125Hz throughout.
 wire drcap,dccap,drv,dcv,dri,dci,drfreeze,dcfreeze,dcpeek,dcadvance;
 wire[7:0]drindex,dcindex;wire signed[23:0]drdata,dcdata;
 wire[671:0]drcur,dccur;wire[3359:0]drhist,dchist;
 wire drav,dcav;wire[143:0]dra,dca;wire[31:0]drframes,dcframes,drcycles,dccycles;
 wire[2:0]drfault,dcfault;
 wire dreset=rst_n&&enable;
 rl_observation_v5_ref #(.OBS_RING_STORAGE(1))dr_obs(.clk(clk),.rst_n(dreset),.capture(drcap),
  .frame_freeze(drfreeze),.frame_read_index(drindex),.frame_read_data(drdata),
  .angular_velocity_q12(72'd0),.gravity_q12(72'd0),.left_delta(16'd0),.right_delta(16'd0),
  .linear_command_q12(24'd0),.yaw_command_q12(24'd0),.height_command_q12(24'd0),
  .last_executed_actions(144'd0),.valid(drv),.initialized(dri),.current_flat(drcur),.history_flat(drhist));
 rl_observation_v5_candidate #(.OBS_RING_STORAGE(1))dc_obs(.clk(clk),.rst_n(dreset),.capture(dccap),
  .frame_freeze(dcfreeze),.frame_read_index(dcindex),.frame_read_data(dcdata),
  .frame_read_peek_first(dcpeek),.frame_read_advance(dcadvance),
  .angular_velocity_q12(72'd0),.gravity_q12(72'd0),.left_delta(16'd0),.right_delta(16'd0),
  .linear_command_q12(24'd0),.yaw_command_q12(24'd0),.height_command_q12(24'd0),
  .last_executed_actions(144'd0),.valid(dcv),.initialized(dci),.current_flat(dccur),.history_flat(dchist));
 rl_policy_tick_v5_ref #(.OBS_RING_STORAGE(1),.PERIOD_CLKS(709))dr_policy(
  .clk(clk),.rst_n(rst_n),.enable(enable),.sensor_ok(1'b1),.frame_valid(drv),
  .current_flat(drcur),.history_flat(drhist),.frame_freeze(drfreeze),.frame_read_index(drindex),.frame_read_data(drdata),
  .capture(drcap),.action_valid(drav),.actions(dra),.completed_frames(drframes),.inference_cycles(drcycles),.fault(drfault),
  .diag_last_fault(),.diag_core_detail(),.diag_fault_count(),.diag_retry_count(),.diag_expected_id(),.diag_result_id());
 rl_policy_tick_v5_candidate #(.OBS_RING_STORAGE(1),.PERIOD_CLKS(709))dc_policy(
  .clk(clk),.rst_n(rst_n),.enable(enable),.sensor_ok(1'b1),.frame_valid(dcv),
  .current_flat(dccur),.history_flat(dchist),.frame_freeze(dcfreeze),.frame_read_index(dcindex),.frame_read_data(dcdata),
  .frame_read_peek_first(dcpeek),.frame_read_advance(dcadvance),
  .capture(dccap),.action_valid(dcav),.actions(dca),.completed_frames(dcframes),.inference_cycles(dccycles),.fault(dcfault),
  .diag_last_fault(),.diag_core_detail(),.diag_fault_count(),.diag_retry_count(),.diag_expected_id(),.diag_result_id());
 always @(posedge clk)if(rst_n&&enable)begin
  if({dr_policy.core.wr_en,dr_policy.core.wr_history,dr_policy.core.wr_addr,
      dr_policy.core.start,dr_policy.core.frame_id}!==
     {dc_policy.core.wr_en,dc_policy.core.wr_history,dc_policy.core.wr_addr,
      dc_policy.core.start,dc_policy.core.frame_id})$fatal(1,"deadline real core protocol");
  if(dr_policy.core.wr_en && ($isunknown({dr_policy.core.wr_data,dc_policy.core.wr_data}) ||
     dr_policy.core.wr_data!==dc_policy.core.wr_data))$fatal(1,"deadline input word");
  if(dr_policy.deadline_missed && dr_policy.state==2)begin
   if(dr_policy.load_index!=27 || dc_policy.load_index!=27 || dcadvance)
    $fatal(1,"deadline LOAD27/cursor hold was not naturally exercised");
   deadline_hits=deadline_hits+1;
   $display("STRIDE32_DEADLINE_LOAD index=%0d period=709",dr_policy.load_index);
  end
 end
 always @(negedge clk)if(rst_n)begin
  if({dr_policy.state,drfault,drav,dra,drframes,drcycles}!==
     {dc_policy.state,dcfault,dcav,dca,dcframes,dccycles})$fatal(1,"deadline STOP/publication differs");
  if(drav||dcav||dr_policy.core.start||dc_policy.core.start)$fatal(1,"partial deadline frame escaped START/publication");
 end
'''


def fixture():
    old = OLD_TB.read_text(encoding='utf8')
    text = old
    for before, after in (
        ('Both model clocks are physical25MHz;', 'Both model clocks are physical50MHz;'),
        ('both periods are200000 cycles/8ms.', 'both normal periods are400000 cycles/8ms.'),
        ('always #20 clk=~clk;', 'always #10 clk=~clk;'),
        ('clock_MHz=25 period_us=8000', 'clock_MHz=50 period_us=8000'),
        ('#400000000;', '#1000000000;'),
        ('finite400ms real-time stimulus watchdog', 'finite1000ms real-time stimulus watchdog')):
        text = once(text, before, after)
    assert text.count('.PERIOD_CLKS(200000)') == 2
    text = text.replace('.PERIOD_CLKS(200000)', '.OBS_RING_STORAGE(1),.PERIOD_CLKS(400000)', 1)
    text = text.replace('.PERIOD_CLKS(200000)', '.PERIOD_CLKS(400000)', 1)
    text = once(text, 'rl_observation_v5_ref r_obs(', 'rl_observation_v5_ref #(.OBS_RING_STORAGE(1))r_obs(')
    text = once(text, '.rst_n(obs_rst),.capture(r_capture),',
                '.rst_n(obs_rst),.capture(r_capture),\n  .frame_freeze(r_freeze),.frame_read_index(r_read_index),.frame_read_data(r_read_data),')
    text = once(text, '.frame_valid(r_valid),.current_flat(r_cur)',
                '.frame_valid(r_valid),.frame_freeze(r_freeze),.frame_read_index(r_read_index),.frame_read_data(r_read_data),.current_flat(r_cur)')
    text = once(text, 'wire obs_rst=rst_n&&enable&&!stop_latched;',
                'wire obs_rst=rst_n&&enable&&!stop_latched;\n wire r_freeze,peek_first,read_advance;wire[7:0]r_read_index;wire signed[23:0]r_read_data;\n'
                ' reg manual_mode=0;reg[143:0]manual_actions=0;wire[143:0]observed_actions=manual_mode?manual_actions:last_actions;')
    assert text.count('.last_executed_actions(last_actions)') == 2
    text = text.replace('.last_executed_actions(last_actions)', '.last_executed_actions(observed_actions)')
    assert text.count('.frame_read_index(read_index),.frame_read_data(read_data),') == 2
    text = text.replace('.frame_read_index(read_index),.frame_read_data(read_data),',
                        '.frame_read_index(read_index),.frame_read_data(read_data),\n  .frame_read_peek_first(peek_first),.frame_read_advance(read_advance),')
    text = once(text, ' task rearm;begin', EXTRA_MONITORS + '\n task rearm;begin')
    text = once(text, '  if(first_repeats<4)',
                '  stop_load(0,0);stop_load(0,26);stop_load(0,167);\n'
                '  stop_load(1,27);stop_load(1,82);stop_load(1,138);\n'
                '  stop_load(2,54);stop_load(2,139);stop_load(2,166);\n'
                '  clip_load(0);clip_load(1);\n'
                '  if(deadline_hits<1||load_stops!=9||clip_cases!=2||weight_frames<20)$fatal(1,"missing stride32 targeted coverage");\n'
                '  for(pad_slot=0;pad_slot<6;pad_slot=pad_slot+1)for(pad_lane=28;pad_lane<32;pad_lane=pad_lane+1)\n'
                '   if(c_obs.g_ring.frame_mem[pad_slot*32+pad_lane]!==24\'hxxxxxx)$fatal(1,"padding was initialized/written");\n'
                '  $display("STRIDE32_TARGETED_PASS load_stops=%0d clip_cases=%0d deadline_hits=%0d weight_frames=%0d",load_stops,clip_cases,deadline_hits,weight_frames);\n'
                '  if(first_repeats<4)')
    text = once(text, 'endmodule', DEADLINE_PAIR + '\nendmodule')
    assert not re.search(r'\b(force|release)\b', re.sub(r'//[^\n]*', '', text))
    assert not re.search(r'frame_mem\s*\[[^\]]+\]\s*(?:<=|=(?!=))', text)
    assert text.count('always #10 clk=~clk;') == 1 and '#20 clk' not in text
    assert '.PERIOD_CLKS(400000)' in text and text.count('.PERIOD_CLKS(709)') == 2
    diff = list(difflib.unified_diff(old.splitlines(), text.splitlines()))
    return text, dict(reference=metadata(OLD_TB), normal_clock_MHz=50, normal_period_clocks=400000,
                     normal_period_us=8000, separate_deadline_period=709, normal_timing_not_accelerated=True,
                     added_lines=sum(x.startswith('+') and not x.startswith('+++') for x in diff),
                     removed_lines=sum(x.startswith('-') and not x.startswith('---') for x in diff))


def source_inventory():
    manifest, inputs, files, assets, packed, rendered, consumers, changes = candidate_generator.render(SOURCE)
    old_result = read(OLD_PROBE / 'result.json')
    assert old_result['passed'] and 'clock_MHz=25' in old_result['summary']
    tb, delta = fixture()
    all_inputs = list(inputs.values()) + [files[r] for r in assets] + [OLD_TB, OLD_PROBE / 'result.json',
                  Path(candidate_generator.__file__).resolve(), Path(__file__).resolve()]
    return inputs, files, assets, rendered, tb, delta, [metadata(p) for p in all_inputs]


def plan(out):
    assert not out.exists()
    inputs, files, assets, rendered, tb, delta, bindings = source_inventory()
    out.mkdir(parents=True)
    shutil.copy2(Path(__file__).resolve(), out / 'prepare_and_run.py')
    receipt = dict(phase='plan_only_not_materialized', recorded_at=datetime.now().astimezone().isoformat(),
        generator=metadata(Path(__file__).resolve()), frozen_generator=metadata(out / 'prepare_and_run.py'),
        candidate_required=True, actual_candidate=None, source=str(SOURCE), input_bindings=bindings,
        source_RTL=63, unchanged_RTL=60, changed_RTL=list(candidate_generator.CHANGED), assets=104, packed_weight_banks=35,
        fixture_change_review=delta, fixture_rendered_in_memory_utf8_bytes=len(tb.encode('utf8')),
        RTL_generated=False, simulation_started=False, automatic_run=False, automatic_SYN=False, automatic_PNR=False,
        commands=dict(materialize=[str(PYTHON), '-B', '-X', 'utf8', str(Path(__file__).resolve()), '--materialize',
                     '--candidate', '<ACTUAL_FROZEN_STRIDE32_CANDIDATE>', '--out', '<UNIQUE_VALIDATION_DIRECTORY>',
                     '--authorization', MATERIALIZE_TOKEN],
                     run=[str(PYTHON), '-B', '-X', 'utf8', '<UNIQUE_VALIDATION_DIRECTORY>/runner.py', '--start',
                          '--out', '<UNIQUE_VALIDATION_DIRECTORY>', '--authorization', START_TOKEN, '--wait-for-memory']),
        normal_comparison=['Both observer/policy use ring branch; old exactSYN43 vs new stride32',
             '50MHz physical simulation clock,400000 clocks per8ms/125Hz; no accelerated normal time',
             'Every168 input write/address/data/START,425 layer/index/data/ID trace records,six actions/result/frameID/publication cycle',
             'Each completed normal core inference reads every40704 gold weight address exactly once, actual packed35-bank values checked',
             'At least two six-slot wraps and first-history repetition; normal core and all inherited inputs/INIT remain exact source'],
        added_finite_cases=dict(LOAD_stops={'external_reset':[0,26,167], 'disable':[27,82,138], 'sensor':[54,139,166]},
            deadline='Separate original/new real pair PERIOD709 naturally expires atLOAD27; no force/counter assignment. NoSTART/publication allowed.',
            clipping='Two completed168word LOADs: +/-409600 and adjacent values, signed24bit extremes,0,+/-1; current clips but same newest history remains raw; disable beforeSTART, not an inference acceptance claim',
            RAM='NaturalX startup retained; only original DUT producer writes; valid lanes0..27 only, padding28..31 remainsX; accepted reads must be known and old/new equal'),
        arbitrary_initial_RAM_scope='Finite4-state simulation detects X leakage; it is not exhaustive quantification over all initial24bit values. Memory-bijection/cursor induction remains a separate source proof obligation.',
        preserved_abort_edge='At a missed-deadline LOAD edge original core.wr_en may still be high; compare and preserve it, while snapshot/cursor hold and subsequent reset preventSTART/publication.',
        scope_excludes=['WholeG2 CPU/UART/RK/real sensors/fusion calibration', 'Actuator/lease/IR/CDC and physical stopping',
                        'Physical timing/resource improvements', 'Arbitrary independent frame_read_index/freezing callers', 'Default flat-branch execution'],
        gates=['No.v before both frozenPNR53/54 natural completion and original tool identity gone',
               'Explicit actual new candidate;63sources and104assets match the exact current stride32 generator',
               'Explicit materialize token; new unique directory; no active RTL tool before writing validation sources',
               'Separate explicit start token;2GiB physical/4GiB commit before each child; one immutable run/no retry',
               'All tools exit0 AND both requiredPASS markers and noFatal/error; preserve warnings and failed receipts'],
        expected_simulation_limit_ms=1000, wall_timeout_s=0, no_progress_timeout_s=0,
        current_numerical_or_physical_pass=False)
    save(out / 'plan.json', receipt)
    save(out / 'job.json', dict(phase='prepared_plan_only', RTL_generated=False, actual_child_started=False))
    return receipt


def check_bindings(rows):
    for row in rows:
        assert metadata(Path(row['path'])) == row, ('frozen input changed', row['path'])


def no_active_rtl_tool():
    import psutil
    names = {'xvlog.exe', 'xelab.exe', 'xsim.exe', 'xsimk.exe', 'vsim.exe', 'vsimk.exe', 'vish.exe'}
    live = [dict(pid=p.pid, name=p.info['name']) for p in psutil.process_iter(['name'])
            if (p.info['name'] or '').lower() in names]
    assert not live, ('RTL inputs may only be generated while prior RTL tools are stopped', live)


def materialize(candidate, out, authorization):
    assert authorization == MATERIALIZE_TOKEN
    pair = candidate_generator.natural_pair_complete()
    no_active_rtl_tool()
    candidate, out = candidate.resolve(), out.resolve()
    assert candidate != SOURCE.resolve() and not out.exists()
    assert out.parent == BASE.resolve() and candidate.parent == BASE.resolve()
    assert not out.is_relative_to(candidate) and not candidate.is_relative_to(out)
    inputs, files, assets, rendered, tb, delta, old_bindings = source_inventory()
    receipt = read(candidate / 'ring_stride32_preparation_review.json')
    assert receipt['phase'] == 'source_materialized_unvalidated' and receipt['RTL_generated'] is True
    assert Path(receipt['source']).resolve() == SOURCE.resolve()
    cm, ci = candidate_generator.paths_in_manifest(candidate)
    assert set(ci) == set(inputs)
    for rel, source in inputs.items():
        if rel in rendered:
            assert ci[rel].read_text(encoding='utf8') == rendered[rel], ('candidate differs from selected generator', rel)
        else:
            assert same_bytes(source, ci[rel]), ('unrelated candidate RTL changed', rel)
    for rel in assets:
        assert same_bytes(SOURCE / rel, candidate / rel), ('asset changed', rel)
    check_bindings(old_bindings)
    candidate_bindings = [metadata(p) for p in ci.values()] + [metadata(candidate / p) for p in assets]
    candidate_bindings += [metadata(candidate / 'ring_stride32_preparation_review.json')]
    out.mkdir()
    simfiles = []
    for root, suffix in ((SOURCE, 'ref'), (candidate, 'candidate')):
        for module in ('rl_observation_v5', 'rl_policy_tick_v5'):
            path = out / (module + '_' + suffix + '.v')
            path.write_text(rename((root / 'control' / (module + '.v')).read_text(encoding='utf8'), module, suffix),
                            encoding='utf8', newline='\n')
            simfiles.append(str(path))
    for rel in CORE:
        dest = out / rel
        dest.parent.mkdir(parents=True, exist_ok=True)
        shutil.copy2(candidate / rel, dest)
        assert same_bytes(candidate / rel, dest)
        simfiles.append(str(dest))
    for rel in assets:
        dest = out / rel
        dest.parent.mkdir(parents=True, exist_ok=True)
        shutil.copy2(candidate / rel, dest)
        assert same_bytes(candidate / rel, dest)
    (out / TB_NAME).write_text(tb, encoding='utf8', newline='\n')
    shutil.copy2(Path(__file__).resolve(), out / 'runner.py')
    commands = [('xvlog', simfiles), ('xvlog', ['-sv', str(out / TB_NAME)]),
                ('xelab', [TOP, '-s', SNAP]), ('xsim', [SNAP, '-runall'])]
    frozen = [metadata(p) for p in out.rglob('*') if p.is_file()]
    cfg = dict(phase='frozen_validation_not_run', recorded_at=datetime.now().astimezone().isoformat(),
               candidate=str(candidate), source=str(SOURCE), natural_PNR_completion=pair,
               source_bindings=old_bindings + candidate_bindings, frozen_inputs=frozen,
               fixture_change_review=delta, commands=commands, normal_clock_MHz=50, normal_period_clocks=400000,
               normal_period_us=8000, real_core=True, real_weights=40704, packed_banks=35,
               no_forced_or_initialized_private_RAM=True, automatic_run=False, automatic_SYN=False, automatic_PNR=False,
               immutable_once_run=True, simulation_limit_ms=1000)
    save(out / 'preparation.json', cfg)
    save(out / 'job.json', dict(phase='frozen_prepared_not_started', actual_child_started=False))
    return cfg


def start(out, authorization, wait):
    assert authorization == START_TOKEN
    import psutil
    from retry_frozen_synthesis import memory_status
    cfg = read(out / 'preparation.json')
    check_bindings(cfg['source_bindings'] + cfg['frozen_inputs'])
    candidate_generator.natural_pair_complete()
    no_active_rtl_tool()
    me = psutil.Process()
    me.nice(psutil.BELOW_NORMAL_PRIORITY_CLASS)
    identity = dict(controller_pid=me.pid, controller_created_epoch=me.create_time(), parent_pid=me.ppid())
    with (out / 'run_claim.json').open('x', encoding='utf8') as f:
        json.dump(identity, f, indent=2)
    def status(phase, **kw):
        save(out / 'job.json', dict(identity, phase=phase, updated_at=datetime.now().astimezone().isoformat(), **kw))
    def admission(tool):
        while True:
            m = memory_status()
            row = dict(physical=m.avail_phys, commit=m.avail_page, minimum_physical=2 * 1024**3, minimum_commit=4 * 1024**3)
            if m.avail_phys >= row['minimum_physical'] and m.avail_page >= row['minimum_commit']:
                return row
            status('waiting_memory', next_tool=tool, memory=row)
            assert wait, ('2/4GiB memory gate; claim retained, do not relaunch same run', row)
            time.sleep(15)
    try:
        for index, (tool, args) in enumerate(cfg['commands']):
            check_bindings(cfg['source_bindings'] + cfg['frozen_inputs'])
            entered = admission(tool)
            argv = ['cmd', '/c', str(VIV / (tool + '.bat')), *args]
            logpath = out / f'{index}_{tool}.log'
            with logpath.open('w', encoding='utf8') as log:
                p = subprocess.Popen(argv, cwd=out, stdout=log, stderr=subprocess.STDOUT,
                                     creationflags=subprocess.CREATE_NO_WINDOW | subprocess.BELOW_NORMAL_PRIORITY_CLASS)
                proc = psutil.Process(p.pid)
                created = proc.create_time()
                runtime = dict(command=argv, wrapper_pid=p.pid, wrapper_created_epoch=created,
                               started_at=datetime.now().astimezone().isoformat(), admission=entered,
                               max_runtime_s=0, max_no_observable_progress_s=0, processes=[])
                seen = {}
                while p.poll() is None:
                    try:
                        for child in [proc] + proc.children(recursive=True):
                            try:
                                row = dict(pid=child.pid, created_epoch=child.create_time(), name=child.name(),
                                           parent_pid=child.ppid(), command=child.cmdline())
                                seen[(row['pid'], row['created_epoch'])] = row
                            except (psutil.NoSuchProcess, psutil.AccessDenied):
                                pass
                    except psutil.NoSuchProcess:
                        pass
                    runtime.update(processes=list(seen.values()), updated_at=datetime.now().astimezone().isoformat())
                    save(out / 'tool_runtime.json', runtime)
                    status('running_' + tool, runtime=runtime)
                    time.sleep(3)
                runtime.update(tool_exit=p.returncode, ended_at=datetime.now().astimezone().isoformat())
                save(out / f'{index}_{tool}_runtime.json', runtime)
                save(out / 'tool_runtime.json', runtime)
            assert p.returncode == 0, (tool, p.returncode)
        check_bindings(cfg['source_bindings'] + cfg['frozen_inputs'])
        log = (out / 'xsim.log').read_text(errors='replace')
        normal = re.search(r'OBSERVATION_RING_REAL4500_PASS[^\r\n]+', log)
        targeted = re.search(r'STRIDE32_TARGETED_PASS[^\r\n]+', log)
        assert normal and targeted and 'clock_MHz=50 period_us=8000' in normal[0]
        assert not re.search(r'(?:Fatal:|FATAL_ERROR|ERROR:|\$fatal)', log, flags=re.I)
        result = dict(passed=True, normal=normal[0], targeted=targeted[0],
                      numeric_frames=re.findall(r'RING_REAL4500_FRAME_MATCH[^\r\n]+', log),
                      scope='Finite paired observer/policy plus real4500 RTL and all40704 gold weights; not wholeG2 or physical acceptance',
                      arbitrary_initial_RAM_exhaustive_proof=False, SYN_PNR_started=False,
                      completed_at=datetime.now().astimezone().isoformat())
        save(out / 'result.json', result)
        status('completed_finite_comparison', passed=True, result=str(out / 'result.json'))
        return result
    except BaseException:
        error = traceback.format_exc()
        save(out / 'result.json', dict(passed=False, error=error))
        status('failed_preserved_no_retry', passed=False, error=error)
        raise


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    mode = ap.add_mutually_exclusive_group(required=True)
    mode.add_argument('--plan', action='store_true')
    mode.add_argument('--materialize', action='store_true')
    mode.add_argument('--start', action='store_true')
    ap.add_argument('--out', type=Path, required=True)
    ap.add_argument('--candidate', type=Path)
    ap.add_argument('--authorization', default='')
    ap.add_argument('--wait-for-memory', action='store_true')
    a = ap.parse_args()
    if a.plan:
        assert a.candidate is None and not a.authorization and not a.wait_for_memory
        r = plan(a.out)
    elif a.materialize:
        assert a.candidate is not None and not a.wait_for_memory
        r = materialize(a.candidate, a.out, a.authorization)
    else:
        assert a.candidate is None
        r = start(a.out.resolve(), a.authorization, a.wait_for_memory)
    print(json.dumps(dict(phase=r.get('phase'), passed=r.get('passed'), output=str(a.out),
                         automatic_run=False, automatic_SYN=False, automatic_PNR=False), ensure_ascii=False))


if __name__ == '__main__':
    main()
