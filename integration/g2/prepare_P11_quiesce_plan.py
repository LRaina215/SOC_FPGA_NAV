"""JSON-only plan/generator recipe for explicit UART source quiesce.

No RTL generation option is supplied here. The proposed ready/flush fields do
not exist in the current PNR53/54 input and are not claimed as hardware proof.
"""
from pathlib import Path
from datetime import datetime
import argparse,json
from prepare_baseline_fusion_retime import metadata,save
BASE=Path('F:/nav_integration_g2_20261023');REPO=Path(__file__).resolve().parents[2]

def plan1(out):
    assert not out.exists();out.mkdir();source=BASE/'SLF128_storage_safeFW_SYN43_prepared1'
    uart=source/'nav/rtl/io/imu_uart.v';t=uart.read_text()
    for a in('reg  [31:0] t_start;','t_start <= time_us;','if (first_pend) begin tpush <= 1\'b1; tpush_d <= t_start;',
              '.s_valid(push), .s_data(push_d), .s_ready(rf_ready)',
              '.s_valid(tpush), .s_data(tpush_d), .s_ready()',
              '.clk(clk), .rst(rst), .s_valid(tx_we)'):
        assert a in t,a
    result=dict(at=datetime.now().astimezone().isoformat(),phase='plan only; proposed hardware not implemented',
        baseline_files=[metadata(uart),metadata(source/'nav/rtl/top/nav_subsystem.v'),metadata(source/'configuration_source/config/regmap.yaml')],
        exact_CSR=dict(address=0x6020,name='IMU_P11_CTRL',access='rwe',
            write_bits={'0':'PARSER_ENABLE request','1':'SOURCE_FLUSH strobe only on this write; ordinary parser CTRL writes withoutbit1 do not reset rawUART/FIFOs'},
            read_bits={'0':'effectivePARSER_ENABLE','8':'SOURCE_READY; zero from SOURCE_FLUSH through continuous-high qualification'},
            reserved_read_zero=True,
            existing_snapshot_words={'0x6024':'CMD32','0x6028':'actualFIRST TS32','0x602C':'SEQ32'},
            current_selected_RTL_has_these_fields=False),
        recipe=[
            'Add source_flush input/source_ready output and one quiescing FF to imu_uart; retain its current clock/rst and all ordinary receive logic',
            'Explicit SOURCE_FLUSH wins over current byte completion: clear receivebusy/counters/partialshift/FIRST/t_start/push/tpush state, assert shared RXbyte+TSFIFO reset',
            'Keep BOTH RXFIFO resets asserted whilequiescing; set producerpush/tpush0 andholdreceiveridle; do not clear or reset TXFIFO/TXFSM',
            'Reuse16bit idle counter duringquiescing: every observed rxsLOW clearsit; only tick_us with rxsHIGH increments; release aftercfg_idle_us+1 observed high ticks (500us default ->501ticks, ensuring>=500us betweenfirst/last)',
            'Drive ready=!quiescing&&!source_flush; at release edge oldquiescing stillholdsFIFOreset, so ordinary reception resumes onnextclk withemptyqueues andfirst_pend reset',
            'Parser isreset/flush while source_ready0; it still requires freshFIRST afterready and validCRC/range beforeSEQ/CMD/TSpublication',
            'Expose readCTRLbit8 from actual source_ready; FW writes2 todisable+flush beforebaud/sourcechange, then optional1 toenable parser without resettingrawagain',
            'CommonNAV reset preserves original normal rawUART startup behavior and emptiesqueues; CPUHOLD alone doesnotresetNAV, so FWinit explicitly writes2 and keepsownerOFF/R0locked'],
        ordering=dict(priority='commonrst > explicitSOURCE_FLUSH >quiescing >normalRX',
            queued_old_bytes='administratively discarded only at explicit source switch/boot; normal WIT/raw operation, badCRC and FIFO-full never triggerflush',
            same_edge_old_character='RXFIFOreset and parserflush win; oldpush cannot enter eitherqueue or snapshot',
            line_low='ready stays0 indefinitely; FW never spins and still checksR0/P11/feedbackdeadline andforceszero',
            new_start_afterready='normal receiver capturesrealstart, FIRSTtimestamp from existingtpush_d; WAIT_ZERO still checksauthorizationtimestamp',
            TX='TXFIFO and TXFSM unchanged; source changes require rawport owner exclusive and no concurrent rawTX. CurrentFW doesnot transmit onthisrawport; concurrent debugTX isoutside source-switch contract'),
        FW_contract=dict(one_ready_read_per_service=True,no_busy_wait=True,
            source_pending='generic boot/P11/WIT handover flag; finalHALzero and NVmotion rejected untilactualREADY',
            WIT_halfpacket='clearonce when starting explicit switch; no WITfeed whilepending, so no repeated cleanup needed atready',
            oldFIFO_service_removed='only proposed HWSOURCE_FLUSH replacesFW32byte/34TSdrain, not ordinaryraw/WITuse',
            ownership='FWalone OFF/WAIT_ZERO/ARMED/LOCKED; no command output muxadded',
            deadlines_us={'P11':200000,'R0':250000,'feedback':50000},physical500ms_not_relaxed=True,
            parser_boot_disabled=True),
        required_later_proofs=['RTLserial/pin+FIFO quiesce miter; no oldDATA/TS or producerpush survives explicitflush at anyreceiverphase',
            'Continuous-high exacttick qualification, LOWglitch restart, clockstop/restart and arbitrarylineLOW preserve ready0',
            'Normal WIT/raw data/TS/FIFO behavior miter whennoSOURCE_FLUSH; TXlogic/ports identical',
            'ActualCmode/FFreset/CPUINIT/clock and CSR bus ACK/effectivewrite ordering',
            'AtomicSEQ proof still depends on serialized soleFWCTRL; externalconcurrentCTRL ABA remainsoutsidecontract',
            'Only future RTL smallmapping/fullLTO and actualbusyCPU/physicalstop can accept deployment'],
        current_validation_scope='proposed CSR substitute only; neither actualhardware nor physicalhighduration tested',
        RTL_materialized=False,EDA_started=False,selected_image_changed=False,deployment_ready=False)
    save(out/'result.json',result);save(REPO/'docs/4_design/evidence/g2_20261023/P11_quiesce_flush_plan_20261026.json',result)
    print(str(out))

def plan2(out):
    """Refine completion semantics without materializing RTL or changing FW."""
    assert not out.exists();out.mkdir()
    old=BASE/'P11_quiesce_flush_plan1/result.json'
    result=json.loads(old.read_text(encoding='utf8'))
    result['at']=datetime.now().astimezone().isoformat()
    result['revision']=2
    result['preserved_first_plan']=metadata(old)
    uart=BASE/'SLF128_storage_safeFW_SYN43_prepared1/nav/rtl/io/imu_uart.v'
    t=uart.read_text()
    assert 'nav_sync2 #(.W(1)) u_s (.clk(clk), .d(uart_rx), .q(rxs));' in t
    assert '.clk(clk), .rst(rst), .s_valid(tx_we)' in t
    result['actual_sync_and_reset_source']=dict(source=metadata(uart),
        rxs='u_s is the existing two-flop synchronizer without reset; it must keep running while the receiver and RX FIFOs are isolated',
        current_common_reset='RX state/u_rf/u_tf and TX state/u_txf currently share rst',
        proposed_administrative_reset='RX state and u_rf/u_tf only; u_s and every TX state/port retain their old reset and clock')
    result['exact_CSR']['read_bits']={'0':'effective PARSER_ENABLE (requested enable AND ACTIVE)',
        '8':'SOURCE_READY completion latch: cleared by explicit FLUSH, set once the selected 2ms qualification has completed; ordinary ACTIVE data never clear it'}
    result['exact_CSR']['RESUME_bit_added']=False
    result['recipe']=[
        'Keep common NAV reset behavior for ordinary raw startup; software BOOT/HOLD initialization explicitly writes SOURCE_FLUSH and owner remains OFF/locked',
        'Explicit SOURCE_FLUSH wins over RX completion/push and clears RX in-flight state, DATA/TS queues, FIRST/t_start/push/tpush and snapshot parser/SEQ; never reset u_s or TX',
        'While QUALIFY hold RX state and both RX FIFO resets asserted, suppress all producers, and observe only continuously updated real synchronized rxs',
        'Use a counter independent of receiver busy: every sampled LOW clears qualification immediately; at HIGH tick_us count 2001 observations so elapsed first-to-last HIGH tick is at least 2000us',
        'After qualification automatically enter ACTIVE and latch READY=1; the qualification transition edge still holds RX FIFOs in reset, ordinary reception starts on the following clock',
        'READY records completed administrative cleanup, not instantaneous line idle; ordinary ACTIVE LOW/start/data never re-clear READY or re-quiesce RX',
        'All queued DATA/TS after READY belong to the post-cleanup boundary; WIT may parse them when FW observes READY and is no longer source_pending',
        'Effective P11 parser enable is requested enable AND ACTIVE; normal snapshot CTRL writes may clear parser/SEQ but do not flush raw RX unless SOURCE_FLUSH is set',
        'No RESUME write or READY_HELD state is added; the abandoned stronger-handshake proposal has no validation or measured code cost']
    result['ordering'].update(dict(
        priority='common rst > explicit SOURCE_FLUSH > configuration-change qualification restart > QUALIFY > normal ACTIVE RX',
        line_low='during QUALIFY any observed LOW restarts the 2ms qualification; line permanently LOW keeps READY=0; ACTIVE normal starts retain READY=1',
        new_start_afterready='ordinary RX captures the real synchronized start and existing t_start; P11 still requires a fresh FIRST/valid frame and WAIT_ZERO post-authorization timestamp',
        TX='SOURCE_FLUSH does not clear u_txf/tx_act or any TX state; cfg_div is shared with TX so changing it requires current TX idle+FIFO empty and an exclusive source-switch owner'))
    result['configuration_order']=dict(
        sequence=['put selected source owner OFF/locked and publish controlled zero through existing HAL',
                  'write CTRL=SOURCE_FLUSH with PARSER_ENABLE=0 and obtain its normal bus ACK',
                  'only with raw TX idle+empty write cfg_div/cfg_idle_us while RX is isolated; each config write restarts the qualification count',
                  'optionally write PARSER_ENABLE=1 for P11 without SOURCE_FLUSH; this does not bypass QUALIFY',
                  'bounded FW service reads READY once; until READY, WIT parsing/NV motion remain blocked and R0/P11/feedback checks continue',
                  'when READY is observed, clear only source_pending; WIT half-packet was cleared once at switch entry, and newly queued bytes remain available'],
        raw_TX_in_current_FW='current selected firmware does not call raw UART TX; this source binding is not a hardware interlock',
        unsupported_concurrent_change='another MMIO/debug writer changing cfg_div/CTRL or sending raw TX during this exclusive switch is outside the contract and must be excluded or explicitly interlocked before deployment')
    result['FW_contract']['READY_meaning']='latched cleanup completion, not an idle sample; no repeated raw drain/quiet loop in FW'
    result['qualification_us']=2000
    result['test_scope_difference']=dict(
        tested_C='P11_quiesce_FW_C_probe1 only supplies a manual READY CSR substitute; it never implements or times actual rxs qualification',
        original_plan='plan1 suggested cfg_idle_us+1 high ticks (500us default) and immediate ready; preserved unchanged',
        selected_plan2='fixed 2ms continuous-high qualification followed by sticky completion READY; same simple C status interface, no claim that 2ms hardware exists',
        no_new_compiler_run_for_plan2=True)
    result['required_later_proofs']+=['New selected 2ms qualification and READY completion latch must be verified against actual synchronized serial RX, every receiver phase, config writes and same-edge FIFO producers',
        'Source-switch serial sender may be continuous/low: safety must remain locked if qualification never completes; sustained stream is not assumed to become quiet',
        'Check synchronization latency and a new start immediately around automatic ACTIVE transition; do not claim external pin waveform equality from the CSR substitute']
    save(out/'result.json',result)
    save(REPO/'docs/4_design/evidence/g2_20261023/P11_quiesce_flush_plan_20261026.json',result)
    print(str(out))
if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--revision',type=int,choices=(1,2),default=2);p.add_argument('--out',type=Path)
    a=p.parse_args();(plan1 if a.revision==1 else plan2)(a.out or BASE/('P11_quiesce_flush_plan'+str(a.revision)))
