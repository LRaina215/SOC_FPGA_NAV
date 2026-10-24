"""Map-offload admission model, using existing RTL-checked SLF vectors.

No hardware or joint build. Latency sweeps are conditional on a supplied
32-byte transaction bound; none of the sweeps establishes a vendor bound.
"""
import argparse
from collections import OrderedDict
from datetime import datetime
import json
from pathlib import Path
import sys

import numpy as np

ROOT = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(ROOT / 'model'))
import nav_fixed as nf
import slf_match as slf
from slf_timing import evaluation_cycles


def hex_words(path):
    return np.array([int(x, 16) for x in path.read_text().split()], dtype=np.int64)


def cache_misses(addresses, entries, associative=False):
    """Clean cache, one eight-slot/32-byte DDR line; no dirty writeback cost."""
    hits = misses = 0
    if associative:
        tags = OrderedDict()
        for address in addresses:
            line = int(address) >> 3
            if line in tags:
                hits += 1
                tags.move_to_end(line)
            else:
                misses += 1
                if len(tags) == entries:
                    tags.popitem(last=False)
                tags[line] = None
    else:
        tags = [-1] * entries
        for address in addresses:
            line = int(address) >> 3
            index = line & (entries - 1)
            if tags[index] == line:
                hits += 1
            else:
                misses += 1
                tags[index] = line
    return dict(accesses=hits + misses, hits=hits, clean_read_misses=misses)


def transform(points, candidates, cx, cy):
    px = points[None, :, 0]
    py = points[None, :, 1]
    angle = ((candidates[:, 2] + 32) & 65535) >> 6
    sn = np.array(nf.SIN, dtype=np.int64)[angle, None]
    cs = np.array(nf.SIN, dtype=np.int64)[(angle + 256) & 1023, None]
    # Vectors are inside the signed 18-bit point-relative range. Do not use
    # this model to admit overflowing configurations that truncate in RTL.
    assert np.all((px - cx >= -131072) & (px - cx < 131072))
    assert np.all((py - cy >= -131072) & (py - cy < 131072))
    x = cx + (((px - cx) * cs - (py - cy) * sn) >> 14) + candidates[:, 0, None]
    y = cy + (((px - cx) * sn + (py - cy) * cs) >> 14) + candidates[:, 1, None]
    return (np.clip(x, -(1 << 25), (1 << 25) - 1) * 1311) >> 12, \
        (np.clip(y, -(1 << 25), (1 << 25) - 1) * 1311) >> 12


def scoring_accesses(points, candidates, cx, cy, words, ox, oy, prior, nogate):
    fx0, fy0 = transform(points, np.zeros((1, 3), dtype=np.int64), cx, cy)
    gx0, gy0 = fx0[0] >> 4, fy0[0] >> 4
    gate_addresses = ((gy0 & 255) << 8) | (gx0 & 255)
    gate_words = words[gate_addresses]
    margin = 0 if prior else 8
    gate = (gx0 - ox >= margin) & (gx0 - ox < 256 - margin) & \
        (gy0 - oy >= margin) & (gy0 - oy < 256 - margin)
    gate &= nogate | ((gate_words & 1) != 0)
    if prior and not nogate:
        gate &= (gate_words >> 16) == ((((gy0 >> 8) & 1) << 1) | ((gx0 >> 8) & 1))
    fx, fy = transform(points[gate], candidates, cx, cy)
    gx, gy, sx, sy = fx >> 4, fy >> 4, fx & 15, fy & 15
    best = np.full(fx.shape, 255, dtype=np.int64)
    any_occ = np.zeros(fx.shape, dtype=bool)
    addresses = []
    normals = np.array(slf.NVEC, dtype=np.int64)
    for dy in (-1, 0, 1):
        for dx in (-1, 0, 1):
            xx, yy = gx + dx, gy + dy
            address = ((yy & 255) << 8) | (xx & 255)
            addresses.append(address)
            word = words[address]
            occ = (word & 128) != 0
            if prior:
                occ &= (word >> 16) == ((((yy >> 8) & 1) << 1) | ((xx >> 8) & 1))
            ex = 32 * dx + 4 * ((word >> 11) & 7) + 1 - 2 * sx
            ey = 32 * dy + 4 * ((word >> 8) & 7) + 1 - 2 * sy
            q = (word >> 2) & 31
            normal = np.abs(ex * normals[q, 0] + ey * normals[q, 1]) >> 8
            l1 = (np.abs(ex) + np.abs(ey)) >> 1
            distance = np.minimum(255, np.where((word & 2) != 0, normal, l1))
            best = np.where(occ, np.minimum(best, distance), best)
            any_occ |= occ
    costs = np.minimum(best, 40)
    near = any_occ & (best <= 6)
    result = np.stack([costs.sum(axis=1), near.sum(axis=1)], axis=1)
    # Loop interchange is only an arithmetic/access-model experiment. It does
    # not establish RTL scheduling, pending write or external read equivalence.
    point_cost = np.zeros_like(result)
    for p in range(costs.shape[1]):
        point_cost[:, 0] += costs[:, p]
        point_cost[:, 1] += near[:, p]
    assert np.array_equal(result, point_cost)
    return gate_addresses, np.stack(addresses, axis=2), result, int(gate.sum())


def adversarial_cases():
    rng = np.random.default_rng(4500)
    # All samples are in the legal prior window. The prior-write interface
    # accepts complete 18-bit words, including gate=1 with no occupancy.
    words = np.ones(65536, dtype=np.int64)
    tracking = np.array([(x*25,y*25,a*128) for a in range(-2,3)
                         for y in range(-3,4) for x in range(-3,4)])
    spread = rng.integers(-110,111,size=(1024,2))*50+25
    conflicts = np.array([((i%13-6)*800+25,(-4800 if i%2 else 4800)+25)
                          for i in range(1024)],dtype=np.int64)
    alignment = np.array([((i%21-10)*50,((i//21)%21-10)*50,-1820)
                          for i in range(256)],dtype=np.int64)
    arbitrary = rng.integers(-32768,32768,size=(256,3),dtype=np.int64)
    rows=[]
    for name,points,candidates in [('tracking_spread',spread,tracking),
        ('tracking_cache_conflicts',conflicts,tracking),
        ('alignment_first_batch',spread,alignment),
        ('legal_arbitrary_candidate_interface',spread,arbitrary)]:
        gate,address,_,valid=scoring_accesses(points,candidates,0,0,words,-128,-128,True,True)
        assert valid==1024
        row=dict(name=name,samples=1024,valid=valid,candidates=len(candidates),
                 original_ms_at_50mhz=evaluation_cycles(1024,valid,len(candidates))/50000,
                 constructed_not_field_trace=True,cache_models=[])
        for order,trace in [('candidate_major',address.reshape(-1)),
                            ('point_major',address.transpose(1,0,2).reshape(-1))]:
            counts=cache_misses(np.concatenate([gate,trace]),512)
            counts.update(order=order,entries=512,
                miss_only_added_ms_at_1us_per_line=counts['clean_read_misses']/1000)
            row['cache_models'].append(counts)
        print('adversarial',name,[m['clean_read_misses'] for m in row['cache_models']],flush=True)
        rows.append(row)
    return rows


def evaluate(vectors, frozen, out):
    assert not out.exists()
    # Current storage/control transformations do not alter the matching FSM.
    assert (frozen / 'nav/rtl/match/slf_match.v').read_bytes() == \
        (ROOT / 'rtl/match/slf_match.v').read_bytes()
    assert slf.W18 and slf.NBR == '3x3' and slf.SN == 256
    observed = json.loads((vectors / 'timing.json').read_text())
    assert observed['pass_']
    n, k, prior_words, ox, oy = map(int, (vectors / 'counts.txt').read_text().split())
    frames = hex_words(vectors / 'frm.hex').astype(np.uint32).view(np.int32).reshape(n, 12)
    pixels = hex_words(vectors / 'pts.hex').astype(np.uint32).view(np.int32).reshape(n, 9600, 4)
    maps = hex_words(vectors / 'sm.hex').reshape(n, 65536)
    c = hex_words(vectors / 'cand.hex').reshape(k, 2)
    signed16 = lambda x: ((x & 65535) ^ 32768) - 32768
    candidates = np.stack([signed16(c[:, 0]), signed16(c[:, 0] >> 16), signed16(c[:, 1])], axis=1)
    expected = hex_words(vectors / 'res.hex').reshape(n, 512)
    pexpected = hex_words(vectors / 'pres.hex').reshape(-1, 512)
    prior_data = hex_words(vectors / 'prior.hex').reshape(prior_words, 2)
    prior = np.zeros(65536, dtype=np.int64)
    prior[prior_data[:, 0]] = prior_data[:, 1]
    points = []
    for i in range(n):
        mask = slf.structure_mask(pixels[i, :, :3], (pixels[i, :, 3] & 1) != 0,
                                  0, int(frames[i, 0]), int(frames[i, 1]))
        index = np.flatnonzero(mask)
        assert min(len(index), 4096) == frames[i, 11]
        points.append(pixels[i, slf.subsample(index), :2].astype(np.int64))
    tests = []
    for i in range(n):
        words = maps[i - 1] if i else np.zeros(65536, dtype=np.int64)
        lfo = frames[i - 1, 6:8] - 128 if i else (-128, -128)
        tests.append((f'online_{i}', i, words, int(lfo[0]), int(lfo[1]), False, False, expected[i]))
    for j, (i, nogate) in enumerate(((3, True), (4, False))):
        tests.append((f'prior_{i}', i, prior, ox, oy, True, nogate, pexpected[j]))
    rows = []
    for index, (name, i, words, x0, y0, use_prior, nogate, gold) in enumerate(tests):
        gate, address, result, valid = scoring_accesses(points[i], candidates,
            int(frames[i, 4]), int(frames[i, 5]), words, x0, y0, use_prior, nogate)
        expected_result = np.stack([gold[0:2*k:2], gold[1:2*k:2] >> 16], axis=1)
        assert np.array_equal(result, expected_result), (name, 'score mismatch', result[:3], expected_result[:3])
        assert np.all((gold[1:2*k:2] & 65535) == valid)
        cycles = evaluation_cycles(len(points[i]), valid, k)
        assert cycles == observed['rows'][index]['observed_cycles']
        row = dict(name=name, samples=len(points[i]), valid=valid, candidates=k,
                   scores_equal_existing_RTL_checked_vectors=True,
                   loop_interchange_arithmetic_equal=True, original_cycles=cycles,
                   original_ms_at_50mhz=cycles/50000, cache_models=[])
        for order, trace in [('candidate_major', address.reshape(-1)),
                             ('point_major', address.transpose(1, 0, 2).reshape(-1))]:
            all_accesses = np.concatenate([gate, trace])
            for entries in (128, 512):
                counts = cache_misses(all_accesses, entries)
                counts.update(order=order, entries=entries, data_RAM_18k_blocks=entries//128,
                    conditional_ms={str(ns):cycles/50000+counts['clean_read_misses']*ns/1e6
                                    for ns in (100, 250, 500, 1000)})
                row['cache_models'].append(counts)
        rows.append(row)
        print(name, 'valid', valid, 'ms', row['original_ms_at_50mhz'],
              'misses', [x['clean_read_misses'] for x in row['cache_models']], flush=True)
    worst_samples=1024
    update_core_bound=11*worst_samples+3*worst_samples+258*258+64
    update_line_bound=2*8192+2*worst_samples
    result = dict(evaluated_at=datetime.now().astimezone().isoformat(), vectors=str(vectors),
        frozen_source=str(frozen), original_RTL_bytes_match=True, rows=rows,
        adversarial=adversarial_cases(),
        static_bounds=dict(maximum_evaluation_cycles=evaluation_cycles(1024,1024,256),
            maximum_evaluation_ms_at_50mhz=evaluation_cycles(1024,1024,256)/50000,
            maximum_matching_read_accesses=1024+9*1024*256,
            clean_read_line_miss_upper_bound=1024+6*1024*256,
            map_update_software_wait_us=5000,
            conservative_original_update_core_cycles=update_core_bound,
            conservative_original_update_core_us=update_core_bound/50,
            serialized_update_line_transactions_upper_bound=update_line_bound,
            serialized_update_mean_transaction_budget_ns=(5000-update_core_bound/50)*1000/update_line_bound,
            transaction_budget_includes_all_buffering_CDC_and_arbitration=True,
            unchanged_interfaces_allow_variable_latency=False,
            vendor_cmd_ready_and_read_completion_have_no_verified_upper_bound=True),
        latency_scope='illustrative baseline plus clean-miss cost only; NOT an end-to-end upper bound',
        hit_interface_assumption='original one-word-per-cycle RAM; isolated request/ACK cache cannot supply this rate',
        candidate_scope='loop interchange arithmetic only; no changed matching RTL or joint build',
        unconditional_vendor_service_bound_available=False, physical_accepted=False,
        programmed=False)
    out.mkdir(parents=True)
    (out/'result.json').write_text(json.dumps(result, indent=2), encoding='utf-8')
    return result


if __name__ == '__main__':
    ap = argparse.ArgumentParser()
    ap.add_argument('--vectors', type=Path, default=ROOT/'sim/build/tb_slf')
    ap.add_argument('--frozen', type=Path, required=True)
    ap.add_argument('--out', type=Path, required=True)
    a = ap.parse_args()
    evaluate(a.vectors.resolve(), a.frozen.resolve(), a.out.resolve())
