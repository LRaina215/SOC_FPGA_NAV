"""Read-only source review of injection-RAM migration; no simulation or EDA."""
from pathlib import Path
import argparse,json

SOURCE=Path('F:/nav_integration_g2_20261023/baseline_fusion50_ring_MAX_syn40')


def review(out):
    assert not out.exists(),out
    p=SOURCE/'nav/rtl/net/host_mbx.v';text=p.read_text(encoding='utf-8')
    guards=[
        'input  wire [15:0] inj_len,',
        "if (inj_go) begin ig_pend <= 1'b1; ilen <= inj_len; its <= now_us; end",
        "if (ia) begin\n                i_re <= 1'b1; i_ra <= ioff[TX_WAW+1:2];",
        "if (ioff + 1'b1 >= ilen) begin ia <= 1'b0; iwp <= 0; end",
        'wire ibusy   = ia | iv1 | iv2;',
        'wire fits  = ({1\'b0, u_len} <= free) && dq_s_ready;',
    ]
    assert all(s in text for s in guards)
    lines=text.splitlines();excerpts=[]
    for a,b in [(27,32),(70,128),(130,174)]:
        excerpts.append(dict(path=str(p),first_line=a,last_line=b,text='\n'.join(lines[a-1:b])))
    r=dict(decision='Retain u_injb distributed; do not promote or change protocol',
        source=str(p),method='Static RTL source and nonblocking edge ordering; no simulator or new synthesis',
        fact_status='Confirmed source behavior; counterexamples are legal RTL-interface schedules, not executed Pico/UART timing traces',
        ready_valid_fact='No injection per-byte ready port exists. ia/i_re/iv1/iv2 issue automatically, u_valid can contain gaps, and dq_s_ready is only the packet descriptor/first-byte capacity check.',
        unchanged_contracts=['INJ_DATA/GO register writes and original bus ACK','16bit injection length with no <=2048 or <=written-byte admission restriction','Original single ig_pend and live ilen/its overwrite by busy GO','iwp saturation and restart at final issued read','Byte order and RX descriptor/drop semantics'],
        rejection_of_simple_read_pause=[
            'Gating the actual RAM re during effective inj_we and holding its request inserts bubbles. The current read request/ib1/il1 must all stall together; advancing any of them loses/reorders bytes.',
            'Even perfectly aligned stalls change data versions: the original logical same-edge read obtains pre-write data, whereas a delayed physical read obtains the new data. Read-first simulation does not license a prohibited physical collision.',
            'Stalls extend ibusy. A network first byte that arrived after old ibusy cleared may now be dropped; this changes packet arbitration and st_drop, not just latency.',
            'A busy GO changes live ilen/its and future last-byte tests. Moving ioff progress relative to the unchanged GO event can change truncation/extension/descriptor timing.',
        ],
        counterexamples=[
            dict(name='first read of newly overwritten word',preconditions='TX_WAW9, iwp1, an active injection whose length includes word1; first pending read has i_ra1 and prior iq is word0. INJ_GO does not require every requested word already written.',
                 edge0='Effective inj_we writes word1=B while i_re reads word1, whose pre-edge initialized or earlier value is A.',
                 original='RTL read q becomes A; the corresponding byte is emitted from A.',
                 paused_read='RAM read is suppressed; retry after the write obtains B and emits that byte from B.',
                 conclusion='Choosing A!=B changes payload. Existing iq cannot supply A on a first read of this word.'),
            dict(name='length beyond physical buffer wraps into restarted writes',preconditions='Length2049 is accepted unchanged; byte2048 addresses word0 because i_ra uses only ioff[TX_WAW+1:2].',
                 edge0='ia issues final read of word0 and resets iwp0 immediately.',
                 edge1='That registered read samples while a new effective INJ_DATA writes word0; both signals are allowed by source.',
                 conclusion='Same old/new data-version conflict; adding len<=2048 would be a protocol change. This is a source-interface schedule, not a claim a specific serial command delay was measured.'),
        ],
        latency_contract_if_pause_were_selected='Each denied read adds at least1 control clock; sustained writes can stall without a finite bound. RX IRQ/descriptor availability and network exclusion window move. GO/ACK events are unchanged, so payload overwrite outcomes can also move. This fails the requested full old contract and is not adopted.',
        alternatives_not_adopted=['A new busy-write/GO restriction or delayed bus ACK changes the protocol/timing contract','A full distributed version mirror defeats the LUT-storage saving','A new faster/phase-shifted memory clock requires a new clock/timing design and is outside this lossless migration'],
        budget=dict(injection_BSRAM_added=0,injection_LUT_recovery_credited=0,not_migrated_physical_blocks_relative_to_old_promotion_plan=2,logical_capacity_words=512,bits_per_word=32),
        source_contract='The combined source keeps the exact proven TX/console host_mbx body: u_injb has no STAGE2B_BLOCK opt-in; only TXshadow is promoted. No GO, ACK, length, write pointer or stream expression is changed for injection.',
        physical_pending='Other selected stores still require actual primitive CEB/BLKSEL/OCE/AD review at stage3; this review claims no silicon collision guarantee.',
        excerpts=excerpts)
    out.parent.mkdir(parents=True,exist_ok=True);out.write_text(json.dumps(r,ensure_ascii=False,indent=2)+'\n',encoding='utf-8')
    print(json.dumps(dict(out=str(out),decision=r['decision']),ensure_ascii=False))
    return r


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--out',type=Path,required=True);review(p.parse_args().out)
