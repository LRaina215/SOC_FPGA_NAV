"""Read-only, fail-closed stack review of an existing linked RV32 image.

This does not compile or execute firmware.  A computed jump is accepted only
as an actual, bounded local switch: reaching definitions identify the constant
table base, a unique straight-line unsigned guard bounds its index, scale/load
are exact, and every ELF table entry is a local instruction boundary.  The
assembly table and all LTO stack-usage files are checked as well.  All other
indirect control flow remains an error.
"""
from __future__ import annotations

import argparse
import json
from pathlib import Path
import re
import struct
import subprocess


COND = {"beq", "bne", "blt", "bge", "bltu", "bgeu", "beqz", "bnez",
        "blez", "bgez", "bltz", "bgtz", "bgt", "ble", "bgtu", "bleu"}
NO_DEST = COND | {"sw", "sh", "sb", "sd", "ret", "j", "jr", "nop",
                  "fence", "fence.i", "ecall", "ebreak", "csrw", "csrs", "csrc"}
CALLER_SAVED = {"ra", "t0", "t1", "t2", "t3", "t4", "t5", "t6",
                "a0", "a1", "a2", "a3", "a4", "a5", "a6", "a7"}


def number(s):
    return int(s, 16) if s.startswith(("0x", "-0x")) else int(s)


def operands(s):
    return [p.strip() for p in s.split("#", 1)[0].split(",")]


def destination(op, arg):
    if op in NO_DEST:
        return None
    if op == "jal":
        return "ra"
    return operands(arg)[0]


def direct_target(arg):
    m = re.search(r"\b([0-9a-f]+)\s+<[^>]+>", arg)
    return int(m[1], 16) if m else None


def elf_sections(data):
    h = struct.unpack_from("<16sHHIIIIIHHHHHH", data)
    assert h[0][:7] == b"\x7fELF\x01\x01\x01" and h[2] == 243, "Expected little-endian ELF32 RISC-V"
    off, stride, count, names_index = h[6], h[11], h[12], h[13]
    sh = [struct.unpack_from("<IIIIIIIIII", data, off + i * stride) for i in range(count)]
    ns = sh[names_index]
    names = data[ns[4]:ns[4] + ns[5]]
    return [(names[s[0]:].split(b"\0")[0].decode(), s) for s in sh]


def read_table(data, sections, address, count):
    for name, s in sections:
        if s[3] <= address and address + count * 4 <= s[3] + s[5]:
            assert name.startswith(".rodata") and s[1] == 1 and not (s[2] & 1), "Table is not read-only data"
            pos = s[4] + address - s[3]
            return list(struct.unpack_from("<" + "I" * count, data, pos)), name
    raise AssertionError("Table outside ELF sections")


def assembly_tables(paths):
    jumps, valid = {}, {}
    for path in paths:
        owners, tables = {}, {}
        owner = label = None
        for line in path.read_text().splitlines():
            m = re.match(r"\s*\.type\s+([^,]+),\s*@function", line)
            if m:
                owner = m[1]
            m = re.match(r"([.$\w]+):$", line)
            if m:
                label = m[1]
                if owner:
                    owners[label] = owner
            m = re.match(r"\s*\.word\s+([.$\w]+)$", line)
            if m and label:
                tables.setdefault((owner, label), []).append(m[1])
            m = re.match(r"\s*(jr|jalr)\s+(.+)$", line)
            if owner and m and m[2].strip() not in ("ra", "x1"):
                jumps.setdefault(owner, []).append((m[1], m[2].strip()))
            if re.match(r"\s*\.size\s+([^,]+),", line):
                owner = label = None
        for (owner, label), targets in tables.items():
            if owner and targets and all(owners.get(t) == owner for t in targets):
                valid.setdefault(owner, []).append((label, targets))
    return valid, jumps


def cfg(rows, switch_targets):
    addresses = [r[0] for r in rows]
    by = {a: (op, arg) for a, op, arg in rows}
    next_addr = dict(zip(addresses, addresses[1:]))
    edges = {a: set() for a in addresses}
    for a, op, arg in rows:
        target = direct_target(arg)
        if op == "ret":
            continue
        if op in ("jr", "jalr"):
            edges[a].update(switch_targets.get(a, []))
            continue
        if op == "j":
            if target in by:
                edges[a].add(target)
            continue  # external tail jump does not return locally
        if op in COND and target in by:
            edges[a].add(target)
        if a in next_addr:
            edges[a].add(next_addr[a])
    reachable, work = {addresses[0]}, [addresses[0]]
    while work:
        for c in edges[work.pop()]:
            assert c in by, "Local CFG target outside function"
            if c not in reachable:
                reachable.add(c)
                work.append(c)
    pred = {a: set() for a in reachable}
    for a in reachable:
        for c in edges[a]:
            pred[c].add(a)
    return by, next_addr, reachable, pred


def reaching(rows, pred, reachable, register):
    by = {a: (op, arg) for a, op, arg in rows}
    incoming = {a: set() for a in reachable}
    outgoing = {a: set() for a in reachable}
    changed = True
    while changed:
        changed = False
        for a in sorted(reachable):
            value = set().union(*(outgoing[p] for p in pred[a])) if pred[a] else set()
            op, arg = by[a]
            # Calls preserve s0..s11 under the linked GCC ABI.  Caller-saved
            # bases cannot be carried through a call by this recognizer.
            defines = destination(op, arg) == register or (op == "jal" and register in CALLER_SAVED)
            out = {a} if defines else value
            if value != incoming[a] or out != outgoing[a]:
                incoming[a], outgoing[a] = value, out
                changed = True
    return incoming


def resolve_switch(rows, jump, sections, data, known_targets, asm_tables):
    by, next_addr, reachable, pred = cfg(rows, known_targets)
    assert jump in reachable, "Unreachable computed jump is not silently ignored"
    cache = {}

    def rd(reg, at):
        if reg not in cache:
            cache[reg] = reaching(rows, pred, reachable, reg)
        defs = cache[reg][at]
        assert len(defs) == 1, f"Non-unique {reg} reaching definitions at {at:x}: {defs}"
        return next(iter(defs))

    def constant(reg, at, visited=None):
        visited = set() if visited is None else visited
        site = rd(reg, at)
        assert (reg, site) not in visited, "Cyclic constant definition"
        visited = visited | {(reg, site)}
        op, arg = by[site]
        p = operands(arg)
        if op == "lui" and p[0] == reg:
            return number(p[1]) << 12
        if op == "li" and p[0] == reg:
            return number(p[1])
        if op == "addi" and p[0] == reg:
            return (constant(p[1], site, visited) + number(p[2])) & 0xffffffff
        raise AssertionError(f"Unsupported constant base {reg}@{site:x}: {op} {arg}")

    op, arg = by[jump]
    assert op == "jr" and len(operands(arg)) == 1, "Only bounded jr switch is supported"
    jump_reg = operands(arg)[0]
    load = rd(jump_reg, jump)
    load_op, load_arg = by[load]
    lp = operands(load_arg)
    assert load_op == "lw" and lp[0] == jump_reg, "Jump is not a 32-bit table load"
    m = re.fullmatch(r"0\((\w+)\)", lp[1])
    assert m, "Nonzero or unrecognized table load offset"
    address_reg = m[1]
    add = rd(address_reg, load)
    add_op, add_arg = by[add]
    ap = operands(add_arg)
    assert add_op == "add" and ap[0] == address_reg, "Table address is not base+scaled index"
    alternatives = []
    for index_reg, base_reg in ((ap[1], ap[2]), (ap[2], ap[1])):
        try:
            shift = rd(index_reg, add)
            sop, sarg = by[shift]
            sp = operands(sarg)
            assert sop == "slli" and sp[0] == index_reg and number(sp[2]) == 2
            base = constant(base_reg, add)
            assert base % 4 == 0
            alternatives.append((index_reg, base_reg, shift, sp[1], base))
        except (AssertionError, ValueError):
            pass
    assert len(alternatives) == 1, "Table base/index not uniquely identified"
    index_reg, base_reg, shift, raw_index, base = alternatives[0]
    # Walk the unique predecessor chain.  This proves the fallthrough bound,
    # not just textual proximity or domination by either branch outcome.
    chain, at, guard = [jump], jump, None
    while pred[at] and len(pred[at]) == 1:
        previous = next(iter(pred[at]))
        pop, parg = by[previous]
        pp = operands(parg)
        if pop in COND:
            assert pop == "bltu" and len(pp) == 3 and pp[1] == raw_index, "Unsupported switch bounds guard"
            assert next_addr.get(previous) == at and direct_target(parg) != at, "Bound must be unsigned <= fallthrough"
            upper = constant(pp[0], previous)
            assert 0 <= upper <= 255, "Switch table bound exceeds recognized limit"
            guard = previous
            break
        chain.append(previous)
        at = previous
        assert len(chain) <= 12, "Switch address chain is not the short supported straight-line pattern"
    assert guard is not None and shift in chain and add in chain and load in chain
    # The raw index must not change between guard and shift.  A self slli at
    # shift is intentional; all earlier writes would invalidate the bound.
    between = [a for a in chain if guard < a < shift]
    assert not any(destination(by[a][0], by[a][1]) == raw_index for a in between), "Index changed after bounds guard"
    targets, section = read_table(data, sections, base, upper + 1)
    assert all(t in by for t in targets), "Switch target not a local instruction boundary"
    matches = [(label, labels) for label, labels in asm_tables if len(labels) == len(targets)]
    assert len(matches) == 1, "Assembly local table ownership/count not unique"
    return dict(jump=hex(jump), table=hex(base), count=len(targets), section=section,
                targets=[hex(t) for t in targets], base_register=base_reg,
                base_reaching_definition=hex(rd(base_reg, add)), raw_index=raw_index,
                guard=hex(guard), shift=hex(shift), address_add=hex(add), load=hex(load),
                unique_fallthrough_chain=[hex(a) for a in reversed(chain)],
                assembly_table=matches[0][0], assembly_targets=matches[0][1],
                local_only=True), targets


def review(directory, soc, output, objdump, max_static):
    elf = directory / "nav_fw.elf"
    su_paths = sorted(directory.glob("*.ltrans.su"))
    asm_paths = sorted(directory.glob("*.ltrans.s"))
    inputs = [elf, soc] + su_paths + asm_paths
    snapshots = {p: p.read_bytes() for p in inputs}
    elf_before = elf.read_bytes()
    sections = elf_sections(elf_before)
    symbols = subprocess.check_output([str(objdump), "-t", str(elf)], text=True)
    dis = subprocess.check_output([str(objdump), "-d", str(elf)], text=True)
    ro = subprocess.check_output([str(objdump), "-s", "-j", ".rodata", str(elf)], text=True)
    (output / "linked_symbols.txt").write_text(symbols)
    (output / "linked_disassembly.txt").write_text(dis)
    (output / "linked_rodata.txt").write_text(ro)
    ranges = {}
    for line in symbols.splitlines():
        c = line.split()
        if len(c) >= 6 and c[2] == "F":
            ranges[c[-1]] = (int(c[0], 16), int(c[4], 16))
    reserved = next(int(l.split()[0], 16) for l in symbols.splitlines() if l.split() and l.split()[-1] == "STACK_SIZE")
    instructions = []
    for line in dis.splitlines():
        m = re.match(r"^\s*([0-9a-f]+):\s+[0-9a-f]+\s+(\S+)\s*(.*)", line)
        if m:
            instructions.append((int(m[1], 16), m[2], m[3]))
    functions = {n: [i for i in instructions if s <= i[0] < s + z] for n, (s, z) in ranges.items()}
    assert su_paths and asm_paths and len(su_paths) == len(asm_paths), "Missing LTO assembly/stack records"
    assert {p.name.removesuffix(".su") for p in su_paths} == {p.name.removesuffix(".s") for p in asm_paths}
    atables, ajumps = assembly_tables(asm_paths)
    frames, nonstatic = {}, []
    for path in su_paths:
        for line in path.read_text().splitlines():
            loc, size, kind = line.split("\t")
            name = loc.rsplit(":", 1)[1]
            frames[name] = max(frames.get(name, 0), int(size))
            if kind != "static":
                nonstatic.append((name, kind))
    assert not nonstatic, f"Nonstatic frames: {nonstatic}"
    own, unknown_sp = {}, []
    graph = {n: set() for n in functions}
    resolved = []
    for name, rows in functions.items():
        assert rows, f"Function without decoded instructions: {name}"
        local_jumps = [a for a, op, arg in rows if op in ("jr", "jalr")]
        if local_jumps:
            assert len(ajumps.get(name, [])) == len(local_jumps), "Assembly/ELF indirect control flow count mismatch"
            known = {}
            for jump in local_jumps:
                result, targets = resolve_switch(rows, jump, sections, elf_before, known, atables.get(name, []))
                known[jump] = targets
            # Recheck after closing every switch back edge; provisional CFG
            # cannot be the final basis for accepting a hoisted definition.
            for jump in local_jumps:
                result, targets = resolve_switch(rows, jump, sections, elf_before, known, atables.get(name, []))
                assert known[jump] == targets
                result["function"] = name
                result["function_range"] = [hex(ranges[name][0]), hex(sum(ranges[name]))]
                resolved.append(result)
        negatives = []
        for address, op, arg in rows:
            if destination(op, arg) == "sp":
                m = re.match(r"sp,sp,(-?\d+)$", arg.split("#", 1)[0].strip())
                if op in ("addi", "add") and m:
                    if int(m[1]) < 0:
                        negatives.append(-int(m[1]))
                elif name != "_start":
                    unknown_sp.append((name, hex(address), op, arg))
            if op in COND | {"jal", "j"}:
                target = direct_target(arg)
                assert target is not None, f"Unresolved direct branch {name}@{address:x}"
                matches = [n for n, (s, z) in ranges.items() if s <= target < s + z]
                assert matches, f"Direct branch target not in linked functions: {address:x} -> {target:x}"
                if not (ranges[name][0] <= target < sum(ranges[name])):
                    graph[name].update(matches)
        own[name] = max(frames.get(re.sub(r"\.\d+$", "", name), 0), sum(negatives))
    assert not unknown_sp, f"Unresolved SP writes: {unknown_sp}"
    # Every decoded computed transfer must have exactly one checked owner.
    actual_indirect = {a for a, op, arg in instructions if op in ("jr", "jalr")}
    assert actual_indirect == {int(r["jump"], 16) for r in resolved}, "Unreviewed computed control flow"
    memo, visiting = {}, set()

    def bound(name):
        assert name not in visiting, f"Recursive call cycle: {name}"
        if name not in memo:
            visiting.add(name)
            best = max((bound(c) for c in graph[name]), default=(0, []), key=lambda x: x[0])
            visiting.remove(name)
            memo[name] = own[name] + best[0], [name] + best[1]
        return memo[name]

    value, path = bound("main")
    assert re.findall(r"\.ENABLE_IRQ\s*\(\s*(\d+)\s*\)", soc.read_text()) == ["0"], "Selected frozen SoC must disable IRQ"
    assert value <= max_static and value <= reserved, "Static stack bound exceeds accepted limit"
    assert all(p.read_bytes() == snapshots[p] for p in inputs), "A frozen input changed during read-only review"
    return dict(passed=True, complete=True, scope="Read-only linked image; GCC ABI callee-saved registers; no IRQ/recursion/heap. Tail calls conservatively nested. Board watermark remains separate.",
                actual_static_bound_bytes=value, maximum_static_gate_bytes=max_static,
                stack_reserved_bytes=reserved, path=path, path_frame_bytes=[own[n] for n in path],
                resolved_switches=resolved, unresolved_indirect=[], nonstatic_frames=[], unresolved_sp_writes=[],
                actual_jr_count=sum(op == "jr" for _, op, _ in instructions),
                actual_jalr_count=sum(op == "jalr" for _, op, _ in instructions),
                ELF_sections=[dict(name=n, address=hex(s[3]), bytes=s[5], type=s[1], flags=s[2])
                              for n, s in sections if n in (".text", ".rodata", ".data", ".bss")],
                irq_disabled=True, all_SU_and_ltrans_included=True,
                inputs=[dict(path=str(p), bytes=p.stat().st_size, mtime_ns=p.stat().st_mtime_ns) for p in inputs],
                compile_performed=False, firmware_executed=False, input_files_changed=False)


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("directory", type=Path, help="Existing firmware directory containing ELF and LTO outputs")
    p.add_argument("--soc", type=Path, required=True, help="Selected frozen nav_soc.v")
    p.add_argument("--output", type=Path, required=True, help="New review directory; never overwrites failed compilation")
    p.add_argument("--objdump", type=Path, default=Path("D:/tools/xpack-riscv-none-elf-gcc-15.2.0-1/bin/riscv-none-elf-objdump.exe"))
    p.add_argument("--max-static", type=int, default=3072)
    a = p.parse_args()
    a.output.mkdir(parents=True, exist_ok=False)
    try:
        result = review(a.directory, a.soc, a.output, a.objdump, a.max_static)
    except Exception as e:
        result = dict(passed=False, complete=False, error=f"{type(e).__name__}: {e}",
                      compile_performed=False, firmware_executed=False, input_files_changed=False)
    (a.output / "result.json").write_text(json.dumps(result, indent=2) + "\n")
    print(json.dumps(result))
    return 0 if result["passed"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
