"""Conservative stack bound for a non-recursive RV image with IRQ disabled.

Uses GCC .su frames and linked function ranges; library prologues come from
objdump. Tail calls are conservatively counted as nested calls. Compiler switch
jumps are accepted only if their assembly table targets stay in that function.
Unresolved indirect control flow prevents reporting a complete bound.
"""
import argparse
import json
from pathlib import Path
import re
import subprocess


def analyze(elf, objdump, su, assembly, irq_disabled):
    symbols=subprocess.check_output([str(objdump),'-t',str(elf)],text=True)
    reserved=next(int(line.split()[0],16) for line in symbols.splitlines() if line.split() and line.split()[-1]=='STACK_SIZE')
    ranges={}
    for line in symbols.splitlines():
        cols=line.split()
        if len(cols)>=6 and cols[2]=='F':
            ranges[cols[-1]]=(int(cols[0],16),int(cols[4],16))
    dis=subprocess.check_output([str(objdump),'-d',str(elf)],text=True)
    functions={name:[] for name in ranges};cur=None
    for line in dis.splitlines():
        m=re.match(r'^([0-9a-f]+) <([^>]+)>:',line)
        if m:cur=m.group(2)
        elif cur in functions:
            m=re.match(r'^\s*([0-9a-f]+):\s+[0-9a-f]+\s+(\S+)\s*(.*)',line)
            if m:
                addr=int(m.group(1),16);start,size=ranges[cur]
                if start<=addr<start+size:functions[cur].append((m.group(2),m.group(3)))
    frames={};nonstatic=[]
    for line in su.read_text().splitlines():
        loc,size,kind=line.split('\t');name=loc.rsplit(':',1)[1]
        frames[name]=int(size)
        if kind!='static':nonstatic.append((name,kind))
    own={};stack_writes=[]
    for name,ins in functions.items():
        prologue=[]
        for op,args in ins:
            if args.startswith('sp,'):
                m=re.match(r'sp,sp,(-?\d+)',args)
                if op in ('addi','add') and m:
                    if int(m.group(1))<0:prologue.append(-int(m.group(1)))
                elif op not in ('sw','sh','sb','sd'):stack_writes.append((name,op,args))
        own[name]=frames.get(re.sub(r'\.\d+$','',name),max(prologue,default=0))
    # Assembly tables and ownership of their local targets.
    owners={};tables={};switches={};owner=None;label=None;recent=[]
    for line in assembly.read_text().splitlines():
        m=re.match(r'\s*\.type\s+([^,]+),\s*@function',line)
        if m:owner=m.group(1);recent=[]
        m=re.match(r'([.$\w]+):$',line)
        if m:
            label=m.group(1)
            if owner:owners[label]=owner
        m=re.match(r'\s*\.word\s+([.$\w]+)$',line)
        if m and label:tables.setdefault(label,[]).append(m.group(1))
        jump=re.match(r'\s*jr\s+(\w+)',line)
        if owner and jump and jump.group(1) not in ('ra','x1'):
            candidates=re.findall(r'%hi\((\.L\w+)\)','\n'.join(recent[-12:]))
            switches.setdefault(owner,[]).append(candidates[-1] if candidates else None)
        recent.append(line)
        if re.match(r'\s*\.size\s+([^,]+),',line):owner=None
    graph={n:set() for n in functions};unknown=[];resolved=[]
    ops={'jal','j','beq','bne','blt','bge','bltu','bgeu','beqz','bnez','blez','bgez','bltz','bgtz','bgt','ble','bgtu','bleu'}
    switch_index={}
    for name,ins in functions.items():
        for op,args in ins:
            if op in ('jalr','jr'):
                i=switch_index.get(name,0);switch_index[name]=i+1
                options=switches.get(name,[]);table=options[i] if i<len(options) else None
                targets=tables.get(table,[])
                if op=='jr' and targets and all(owners.get(t)==name for t in targets):
                    resolved.append(dict(function=name,table=table,targets=targets))
                else:unknown.append((name,op,args))
            if op in ops:
                m=re.search(r'<([^>+]+)',args)
                if m and m.group(1) in graph and m.group(1)!=name:graph[name].add(m.group(1))
    memo={};visiting=set()
    def bound(name):
        if name in visiting:raise ValueError('Recursive call cycle: '+name)
        if name in memo:return memo[name]
        visiting.add(name)
        best=max((bound(c) for c in graph[name]),default=(0,[]),key=lambda x:x[0])
        visiting.remove(name);memo[name]=(own[name]+best[0],[name]+best[1]);return memo[name]
    value,path=bound('main')
    # Startup intentionally sets sp to RAM end; it isn't called by main.
    stack_writes=[r for r in stack_writes if r[0]!='_start']
    complete=irq_disabled and not (unknown or nonstatic or stack_writes)
    return dict(complete=complete,static_bound_bytes=value if complete else None,
                direct_call_bound_bytes=value,path=path,resolved_switches=resolved,
                unresolved_indirect=unknown,nonstatic_frames=nonstatic,unresolved_sp_writes=stack_writes,
                irq_disabled=irq_disabled,stack_reserved_bytes=reserved,
                largest_frames=sorted(own.items(),key=lambda x:x[1],reverse=True)[:8],
                scope='linked image, no IRQ/recursion/heap; tail calls conservatively nested; board watermark still pending')


def main():
    ap=argparse.ArgumentParser();ap.add_argument('directory',type=Path)
    ap.add_argument('--objdump',type=Path,default=Path(r'D:/tools/xpack-riscv-none-elf-gcc-15.2.0-1/bin/riscv-none-elf-objdump.exe'))
    a=ap.parse_args();d=a.directory
    root=Path(__file__).resolve().parents[2]
    irq_disabled='.ENABLE_IRQ(0)' in (root/'rtl/soc/nav_soc.v').read_text()
    r=analyze(d/'nav_fw.elf',a.objdump,next(d.glob('*.su')),next(d.glob('*.ltrans.s')),irq_disabled)
    (d/'stack_analysis.json').write_text(json.dumps(r,indent=2));print(json.dumps(r))
    return 0 if r['complete'] and r['static_bound_bytes']<=r['stack_reserved_bytes'] else 1


if __name__=='__main__':raise SystemExit(main())
