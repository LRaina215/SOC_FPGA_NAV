"""Read-only PNR50 BSRAM geometry/explicit INIT and split reconstruction audit.

Uses selected memory hierarchy expansion, never a full-design connectivity graph.
No EDA, hashes, or source writes. Output is evidence only.
"""
import argparse
from collections import Counter
import json
from pathlib import Path
import re

from audit_netlist_memories import BSRAM, PARAM, MODULE, inventory
from revalidate_lidar_cdc import definitions
from review_small_block_stores import parameter_reader
from review_baseline_single_factor_syn import cpu_image
from synthesis_connectivity import read, PORT, TOKEN, DECL

LIB = Path('D:/Gowin2/Gowin_V1.9.12.04_x64/IDE/simlib/gw5a/prim_sim.v')


def save(path, obj):
    path.write_text(json.dumps(obj, ensure_ascii=False, indent=2)+'\n', encoding='utf-8')


def metadata(path):
    s = path.stat()
    return dict(path=str(path), bytes=s.st_size, mtime_ns=s.st_mtime_ns)


def value(raw):
    m = re.fullmatch(r"(\d+)'([bhd])([0-9a-fA-F]+)", raw)
    if m:
        return int(m[3], {'b': 2, 'h': 16, 'd': 10}[m[2]])
    if re.fullmatch(r'\d+', raw):
        return int(raw)
    if raw.startswith('"') and raw.endswith('"'):
        return raw[1:-1]
    raise ValueError(('unsupported parameter literal', raw))


def physical_memories(path):
    text = path.read_text(encoding='utf-8')
    kinds = '|'.join(sorted(BSRAM, key=len, reverse=True))
    # RAM port closure may share the last port line. Do not require ^);.
    cell = re.compile(r'^('+kinds+r')\s+(\\\S+|[^\s(]+)\s*\(\s*\n(.*?)\)\s*;', re.M|re.S)
    decl = {n.lstrip('\\'): (int(hi), int(lo)) for d, hi, lo, n in DECL.findall(text) if hi}
    def expand(expr):
        result = []; pos = 0
        while expr[pos:].strip():
            m = TOKEN.match(expr, pos)
            assert m, ('physical expression unsupported', expr[pos:])
            pos = m.end(); width, base, digits, name, hi, lo, punct = m.groups()
            if punct:
                continue
            if width:
                assert not re.search('[xXzZ]', digits)
                num = int(digits, {'b': 2, 'h': 16, 'd': 10}[base])
                result += ['VCC' if num>>i & 1 else 'GND' for i in range(int(width)-1, -1, -1)]
            else:
                name = name.lstrip('\\')
                if hi is not None:
                    a, b = int(hi), int(lo or hi)
                elif name in decl:
                    a, b = decl[name]
                else:
                    result.append(name); continue
                result += [f'{name}[{i}]' for i in range(a, b+(1 if b>=a else -1), 1 if b>=a else -1)]
        return result
    result = {}
    for kind, raw, body in cell.findall(text):
        name = raw.lstrip('\\').strip()
        assert not re.search(r'^defparam\s', body, re.M), ('absorbed next instance', name)
        assert name not in result
        result[name] = dict(name=name, kind=kind, parameters={},
                            ports={p: expand(e) for p,e in PORT.findall(body)})
    headers = re.findall(r'^('+kinds+r')\s+(\\\S+|[^\s(]+)\s*\(', text, re.M)
    assert len(headers) == len(result), ('physical BSRAM parser missed header', len(headers), len(result))
    for raw, key, raw_value in PARAM.findall(text):
        name = raw.strip().lstrip('\\')
        if name in result:
            assert key not in result[name]['parameters']
            result[name]['parameters'][key] = raw_value.strip()
    return result


def extract(source, physical):
    synvg = source/'impl/gwsynthesis/g2_top.vg'
    vo = physical/'impl/pnr/g2_top.vo'
    inv = inventory(source)
    targets = {x['path'].split('/', 1)[1].replace('\\', '') for x in inv['leaves']}
    allowed = set()
    for n in targets:
        parts = n.split('/')
        allowed.update('/'.join(parts[:i]) for i in range(1, len(parts)+1))
    cells = read(synvg, keep=lambda n: n in allowed)
    assert set(cells) == targets, dict(missing=sorted(targets-set(cells)), extra=sorted(set(cells)-targets))
    params = parameter_reader(definitions(synvg))
    source_rows = {n: dict(**c, parameters=params(c)) for n,c in cells.items()}
    physical_rows = physical_memories(vo)
    lib = dict(MODULE.findall(LIB.read_text(encoding='utf-8')))
    defaults = {}
    for kind in sorted({c['kind'] for c in source_rows.values()}):
        rows = re.findall(r'^parameter\s+(\w+)\s*=\s*(.*?);', lib[kind], re.M)
        defaults[kind] = dict(rows)
        assert all(f'INIT_RAM_{i:02X}' in defaults[kind] for i in range(64))
    return source_rows, physical_rows, defaults


def effective(row, defaults):
    raw = dict(defaults[row['kind']]); raw.update(row['parameters'])
    return {k: value(v) for k,v in raw.items()}


def init_vector(row, defaults):
    p = effective(row, defaults)
    page_bits = 288 if 'X9' in row['kind'] else 256
    return sum(p[f'INIT_RAM_{i:02X}'] << (page_bits*i) for i in range(64)), page_bits*64


def check(source, physical, syn, phys, defaults):
    assert len(syn) == 314 and len(phys) == 332
    assert set(syn) <= set(phys)
    splits = []; ordinary = []; used = set()
    for name,s in syn.items():
        p = phys[name]; used.add(name)
        assert p['kind'] == s['kind'], ('memory primitive changed', name)
        sp, pp = effective(s, defaults), effective(p, defaults)
        width_keys = [k for k in ('BIT_WIDTH','BIT_WIDTH_0','BIT_WIDTH_1') if k in s['parameters']]
        width = max(sp[k] for k in width_keys)
        source_init, bits = init_vector(s, defaults)
        explicit_init = {k:v for k,v in s['parameters'].items() if k.startswith('INIT_RAM_')}
        if width not in (32, 36):
            checks = dict(kind_equal=p['kind']==s['kind'], all_effective_parameters_equal=sp==pp,
                          all_explicit_parameters_equal=s['parameters']==p['parameters'],
                          explicit_INIT_equal=explicit_init=={k:v for k,v in p['parameters'].items() if k.startswith('INIT_RAM_')},
                          complete_effective_INIT_equal=init_vector(p, defaults)[0]==source_init)
            ordinary.append(dict(name=name, kind=s['kind'], geometry={k:sp[k] for k in width_keys},
                                 initialization_capacity_bits=bits, explicit_INIT_pages=len(explicit_init), checks=checks))
            continue
        parent, leaf = name.rsplit('/', 1)
        candidates = [parent+'/gowin_add_'+leaf, parent+'/gowin_add_'+s['kind']+'_'+leaf]
        found = [n for n in candidates if n in phys]
        assert len(found) == 1, ('split counterpart not unique', name, found)
        added = found[0]; q = phys[added]; used.add(added)
        qp = effective(q, defaults); half = width//2
        checks = dict(primitive_pair_equal=s['kind']==p['kind']==q['kind'],
                      half_geometry=all(pp[k]==qp[k]==sp[k]//2 for k in width_keys),
                      other_effective_parameters_equal=all(sp[k]==pp[k]==qp[k] for k in sp if k not in width_keys),
                      explicit_INIT_copied_to_both=all(explicit_init=={k:v for k,v in r['parameters'].items() if k.startswith('INIT_RAM_')} for r in (p,q)))
        # In 16/18 mode AD[4] selects the even/odd half-word of each original
        # 32/36 word. AD[13:5] must stay exactly bound to source address bits.
        address_pins = ('AD',) if s['kind'].startswith('pROM') else ('ADA','ADB')
        checks['address_high_bits_equal'] = all(s['ports'][k][:-5]==p['ports'][k][:-5]==q['ports'][k][:-5] for k in address_pins)
        checks['even_odd_address_select'] = all(p['ports'][k][-5]=='GND' and q['ports'][k][-5]=='VCC' for k in address_pins)
        checks['DO_lower_upper_reassembled'] = p['ports']['DO'][-half:]==s['ports']['DO'][-half:] and q['ports']['DO'][-half:]==s['ports']['DO'][:half]
        if 'DI' in s['ports']:
            checks['DI_lower_upper_reassembled'] = p['ports']['DI'][-half:]==s['ports']['DI'][-half:] and q['ports']['DI'][-half:]==s['ports']['DI'][:half]
            checks['write_byte_enables_preserved'] = p['ports']['ADA'][-2:]==s['ports']['ADA'][-2:] and q['ports']['ADA'][-2:]==s['ports']['ADA'][-4:-2]
        control_pins = [k for k in s['ports'] if k not in ('DI','DO','AD','ADA','ADB')]
        checks['actual_clock_enable_reset_block_select_equal'] = all(s['ports'][k]==p['ports'][k]==q['ports'][k] for k in control_pins)
        iv0 = init_vector(p, defaults)[0]; iv1 = init_vector(q, defaults)[0]
        depth = bits//width; mismatches=[]; hm=(1<<half)-1; wm=(1<<width)-1
        for address in range(depth):
            expected=(source_init>>(address*width))&wm
            actual=((iv0>>((2*address)*half))&hm) | (((iv1>>((2*address+1)*half))&hm)<<half)
            if expected != actual:
                mismatches.append(dict(address=address, source=expected, reassembled=actual))
        checks['every_initialized_word_reassembled_equal'] = not mismatches
        splits.append(dict(source=name, added=added, kind=s['kind'], source_width=width,
                           physical_width_each=half, original_logical_depth=depth,
                           physical_depth_each=bits//half, source_INIT_bits=bits,
                           source_explicit_INIT_pages=len(explicit_init), source_nonzero_INIT_pages=sum(value(v)!=0 for v in explicit_init.values()),
                           all_original_words_checked=depth, source_low_half_to_original=True, source_high_half_to_added=True,
                           source_geometry={k:sp[k] for k in width_keys}, original_geometry={k:pp[k] for k in width_keys}, added_geometry={k:qp[k] for k in width_keys},
                           checks=checks, INIT_word_mismatches=mismatches))
    assert used == set(phys), ('unmapped physical BSRAM', sorted(set(phys)-used))
    assert len(ordinary)==296 and len(splits)==18
    z = {k:v for k,v in defaults['SP'].items() if k.startswith('INIT_RAM_')}
    synimage, phyimage = cpu_image(syn,z), cpu_image(phys,z)
    binary = (source/'firmware/nav_fw.bin').read_bytes()
    words = b''.join(int(x,16).to_bytes(4,'little') for x in (source/'ram_w.hex').read_text().split())
    lanes = [bytes(int(x,16) for x in (source/f'ram_b{i}.hex').read_text().split()) for i in range(4)]
    laneimage = bytes(lanes[i%4][i//4] for i in range(131072))
    cpu = dict(actual_physical_SP_bitplanes=sum(n.startswith('u_soc/g_lane[') for n in phys),
               complete_image_bytes=len(phyimage), frozen_firmware_bytes=len(binary),
               checks=dict(source_and_physical_complete_128KiB_equal=synimage==phyimage,
                           physical_equals_original_firmware_and_zero_tail=phyimage==binary+bytes(131072-len(binary)),
                           physical_equals_word_hex=phyimage==words,
                           physical_equals_all_four_lane_hex=phyimage==laneimage))
    report=(physical/'impl/pnr/g2_top.rpt.txt').read_text()
    report_blocks=int(re.search(r'^\s*BSRAM\s*\|\s*(\d+)/',report,re.M)[1])
    bind = (physical/'g2_top.vg').read_bytes()==(source/'impl/gwsynthesis/g2_top.vg').read_bytes()
    split_enabled='-convert_sdp32_36_to_sdp16_18' in (physical/'impl/pnr/cmd.do').read_text()
    passed = bind and split_enabled and report_blocks==len(phys) and all(all(r['checks'].values()) for r in ordinary+splits) and all(cpu['checks'].values())
    return dict(passed=passed, source=str(source), physical=str(physical),
                scope='Actual BSRAM geometry/INIT, split port/address/data/byte-enable reconstruction and CPU image; no whole-design logic or routing/timing acceptance',
                frozen_physical_input_VG_equals_SYN37=bind,
                actual_wide_split_option_enabled=split_enabled,
                source_BSRAM=len(syn), unsplit_BSRAM=len(ordinary), split_pairs=len(splits), physical_BSRAM=len(phys), report_BSRAM=report_blocks,
                source_by_primitive=dict(Counter(r['kind'] for r in syn.values())), physical_by_primitive=dict(Counter(r['kind'] for r in phys.values())),
                split_by_primitive=dict(Counter(r['kind'] for r in splits)),
                source_capacity_bits=sum(init_vector(r,defaults)[1] for r in syn.values()), physical_capacity_bits=sum(init_vector(r,defaults)[1] for r in phys.values()),
                initialized_split_words_checked=sum(r['all_original_words_checked'] for r in splits),
                unsplit=ordinary, splits=splits, CPU64=cpu,
                existing_gate_reused=str(physical/'g2_gate.json'),
                routing_failure_recorded_in_report='1659 unrouted nets' in report,
                note='Split halves retain the complete original INIT vector. With mode32/36 to16/18 and AD[4]=0/1, each half selects even/odd words; copying INIT is required, not an unexplained duplicated image.')


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--source', type=Path, default=Path('F:/nav_integration_g2_20261023/baseline_fusion_model25_bounded_MAX_syn37'))
    ap.add_argument('--physical', type=Path, default=Path('F:/nav_integration_g2_20261023/baseline_fusion_model25_bounded_MAX_pnr50'))
    ap.add_argument('--out', type=Path, required=True)
    a = ap.parse_args(); a.out.mkdir(parents=True, exist_ok=True)
    inputs = [a.source/'impl/gwsynthesis/g2_top.vg', a.physical/'g2_top.vg', a.physical/'impl/pnr/g2_top.vo',
              a.physical/'impl/pnr/g2_top.rpt.txt', a.physical/'impl/pnr/cmd.do', a.physical/'g2_gate.json', LIB,
              a.source/'firmware/nav_fw.bin', a.source/'ram_w.hex']+[a.source/f'ram_b{i}.hex' for i in range(4)]
    before = [metadata(p) for p in inputs]
    syn, phys, defaults = extract(a.source, a.physical)
    save(a.out/'actual_bsram_parameters_ports.json', dict(source=syn, physical=phys, library_defaults=defaults))
    result = check(a.source, a.physical, syn, phys, defaults)
    result['inputs_before'] = before
    result['inputs_unchanged_after_read_only_review'] = before==[metadata(p) for p in inputs]
    result['passed'] &= result['inputs_unchanged_after_read_only_review']
    save(a.out/'result.json',result)
    print(json.dumps({k:v for k,v in result.items() if k not in ('unsplit','splits','inputs_before')},ensure_ascii=False))
    assert result['passed'], 'See saved result.json for exact failed claims'


if __name__ == '__main__':
    main()
