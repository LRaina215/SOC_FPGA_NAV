"""Remove only the private frame shift-buffer reset, after a write-before-read proof.

SNAPSHOT fills all4032 bits before entering LOAD. Any reset/fault/disable aborts
the transaction and resets or disables the consumer. Every later LOAD again
follows a full capture. State, faults, actions, core reset and all enables stay.
"""
from pathlib import Path

OLD='''    always @(posedge clk or negedge rst_n) begin
        if(!rst_n) snapshot<=0;
        else if(snapshot_update)
            snapshot<=snapshot_shift_select ? {24'd0,snapshot[4031:24]} : {history_flat,current_flat};
    end'''
NEW='''    // Private frame data is wholly overwritten in SNAPSHOT before any LOAD.
    // Reset still aborts the scheduler and resets the core; stale private data
    // cannot be consumed after reset, disable, sensor failure or deadline abort.
    always @(posedge clk) begin
        if(snapshot_update)
            snapshot<=snapshot_shift_select ? {24'd0,snapshot[4031:24]} : {history_flat,current_flat};
    end'''


def transform(text):
    assert text.count(OLD)==1 and 'snapshot_shift_select /* synthesis syn_maxfan = 128 */' in text
    assert 'wire snapshot_update = run_ok && !deadline_missed' in text
    changed=text.replace(OLD,NEW,1)
    assert changed.replace(NEW,OLD,1)==text
    return changed


def generate(path):
    path=Path(path);path.write_text(transform(path.read_text(encoding='utf-8')),encoding='utf-8',newline='\n')
