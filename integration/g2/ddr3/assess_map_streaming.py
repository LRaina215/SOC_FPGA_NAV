"""Arithmetic proof of first-update row bucketing and a conditional budget.

This is an admission model, not an implemented DMA or cycle-accurate RTL.
It intentionally leaves vendor/service/adapter gates false.
"""
import argparse
from datetime import datetime
import json
from pathlib import Path
import sys

import numpy as np

ROOT=Path(__file__).resolve().parents[3]
sys.path.insert(0,str(ROOT/'model'))
import slf_match as slf


def update_word(w,tag,mx,my):
    old_tag,hit=(w>>16)&3,(w>>14)&3
    x,y=(w>>11)&7,(w>>8)&7
    fresh=tag!=old_tag or hit==0
    n0=0 if fresh else hit
    def coordinate(old,new):
        if fresh:return new
        if n0>=3:return old
        d=new-old;n=n0+1
        # Independent scalar expression for the RTL's ties-toward-zero rule.
        magnitude=(2*abs(d)+n-1)//(2*n)
        return old+(-magnitude if d<0 else magnitude)
    return (tag<<16)|(min(n0+1,3)<<14)|(coordinate(x,mx)<<11)|\
        (coordinate(y,my)<<8)|(w&255)


def evaluate(out):
    assert not out.exists()
    rng=np.random.default_rng(4500125)
    batches=32;records_total=0;max_row=0
    for case in range(batches):
        ref=slf.StructMap(1311)
        ref.hit=rng.integers(0,4,size=65536,dtype=np.int64)
        ref.tag=rng.integers(0,4,size=65536,dtype=np.int64)
        ref.cxs=2*rng.integers(0,8,size=65536,dtype=np.int64)+1
        ref.cys=2*rng.integers(0,8,size=65536,dtype=np.int64)+1
        low=rng.integers(0,256,size=65536,dtype=np.int64)
        original=slf.pack_hi(ref.tag,ref.hit,ref.cxs,ref.cys)|low
        pts=rng.integers(-120000,120001,size=(1024,2),dtype=np.int64)
        if case%4==0:pts[:,1]=25  # all first-updates in one physical map row
        if case%4==1:pts[::2]=pts[1::2]  # repeated slots must retain first
        fx,fy=ref.cell16(pts[:,0]),ref.cell16(pts[:,1])
        ix,iy=fx>>4,fy>>4
        addresses=((iy&255)<<8)|(ix&255)
        tags=slf.tag_of(ix,iy)
        heads=[None]*256;nodes=[];seen=set()
        for i,address in enumerate(addresses):
            address=int(address)
            if address in seen:continue
            seen.add(address);row=address>>8
            metadata=(int(tags[i])<<6)|(((int(fx[i])&15)>>1)<<3)|((int(fy[i])&15)>>1)
            nodes.append((address&255,metadata,heads[row]));heads[row]=len(nodes)-1
        assert len(nodes)<=1024
        actual=original.copy();visited=set()
        for row,pointer in enumerate(heads):
            count=0
            while pointer is not None:
                assert pointer not in visited
                visited.add(pointer);count+=1
                col,metadata,pointer=nodes[pointer]
                address=(row<<8)|col
                actual[address]=update_word(int(actual[address]),metadata>>6,
                                            (metadata>>3)&7,metadata&7)
            max_row=max(max_row,count)
        assert len(visited)==len(nodes)
        ref.update(pts)
        expected=slf.pack_hi(ref.tag,ref.hit,ref.cxs,ref.cys)|low
        assert np.array_equal(actual,expected),(case,'changed original map values')
        records_total+=len(nodes)
    # Full 64K structure map, 32-bit DDR slots, read once and write once.
    # At one transferred map word/core cycle, copy cost is 131072 cycles.
    core_hz=50_000_000
    lf_cycles=258*258+64
    original_point_update_cycles=14*1024
    copy_cycles=2*65536
    fixed_us=(lf_cycles+original_point_update_cycles+copy_cycles)/50
    # Component datasheet:4Gb needs260ns tRFC; >85C needs3.9us tREFI.
    # This only reserves configured refresh occupancy, not unverified
    # controller queuing, precharge, temperature/voltage or arbitration.
    max_refreshes=(5_000_000+3899)//3900+1
    refresh_reserve_us=max_refreshes*260/1000
    result=dict(evaluated_at=datetime.now().astimezone().isoformat(),
        arithmetic=dict(batches=batches,samples_per_batch=1024,full_map_words_compared=batches*65536,
            unique_records_checked=records_total,maximum_records_in_one_row=max_row,
            matches_existing_StructMap_update=True,first_point_rule_preserved=True,
            all_original_18bits_preserved=True,scope='row bucketing/update arithmetic only, not full SLF or RTL/physical equivalence'),
        proposed_storage=dict(node_capacity=1024,node_bits=27,explicit_18bit_planes_target_blocks=2,
            node_RTL_synthesis_verified=False,head_bits=256*11,head_target='distributed',
            original_touch_bitmap_and_4096x36_point_buffer_preserved=True),
        conditional_schedule=dict(map_words=65536,bytes_read_plus_written=2*65536*4,
            full_read_row_batches=256,full_write_row_batches=256,
            copy_core_cycles=copy_cycles,existing_compute_core_cycles=lf_cycles+original_point_update_cycles,
            serial_copy_plus_existing_compute_us=fixed_us,software_update_wait_us=5000,
            remaining_all_added_overheads_us=5000-fixed_us,
            assumed_refresh_interval_ns=3900,assumed_refresh_busy_ns=260,
            refresh_reserve_us=refresh_reserve_us,
            remaining_after_refresh_for_all_DMA_CDC_arbitration_and_bucket_us=5000-fixed_us-refresh_reserve_us,
            additional_bucket_and_adapter_cycles_measured=False,
            copy_word_per_cycle_architecture_implemented=False,
            refresh_reserve_is_only_configured_nominal_not_system_WCET=True,
            ordinary_request_ack_cache_min_read_interval_cycles=4,
            ordinary_full_map_read_lower_bound_us=4*65536/50,
            ordinary_cache_scan_meets_5ms=False),
        verdict='resource-promising; ordinary request/ACK cache fails fixed scan budget; streaming implementation and measured bounded service still required',
        end_to_end_worst_latency_accepted=False,automatic_joint_synthesis=False,
        automatic_pnr=False,programmed=False)
    out.mkdir(parents=True)
    (out/'result.json').write_text(json.dumps(result,indent=2),encoding='utf-8')
    print(json.dumps(result,indent=2))


if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--out',type=Path,required=True)
    evaluate(p.parse_args().out.resolve())
