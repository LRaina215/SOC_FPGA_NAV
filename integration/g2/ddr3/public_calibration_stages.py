"""Build read-only monitors from published C .vo wire declarations."""
import re


def observer(netlist):
    declarations=set(re.findall(r'(?m)^wire(?:\s+\[[^\]]+\])?\s+\\(\S+)\s*;',netlist))
    root='gw3_top/u_ddr_phy_top/u_ddr_init/'
    groups={name:sorted(n for n in declarations if n.startswith(root+name+'.'))
            for name in ('init_state','read_cal_state','wrlvl_state')}
    assert [len(groups[n]) for n in ('init_state','read_cal_state','wrlvl_state')]==[20,17,12]
    lines=['// Public .vo nets only; protected DQS objects are never inspected.']
    for label,names in groups.items():
        lines.append('wire['+str(len(names)-1)+':0] stage_'+label+'={'+', '.join('dut.\\'+n+' ' for n in names)+'};')
    extras={
        'read_sel':(root+'read_rclksel_conf',4),'burst_seen':(root+'rburst_reg',4),
        'burst_check':(root+'rburst_cnt_check',4),'burst_counts':(root+'rburst_cnt',40),
        'dll_codes':('gw3_top/u_ddr_phy_top/realtime_dllcode_Z',32),
        'eye_errors':('gw3_top/u_ddr_phy_top/eye_calib_error_Z',4),
        'eye_done':('gw3_top/u_ddr_phy_top/eye_calib_done_Z',1),
        'eye_valid':('gw3_top/u_ddr_phy_top/eye_calib_valid',1)}
    for label,(name,width) in extras.items():
        assert name in declarations,name
        lines.append(f'wire[{width-1}:0] stage_{label}=dut.\\{name} ;')
    lines += ['integer stage_cycles[0:16];integer stage_changes=0;reg[16:0]last_read_stage=0;',
              'reg[19:0]last_init_stage=0;',
              'initial for(integer k=0;k<17;k++)stage_cycles[k]=0;',
              'always@(posedge ui)begin',
              '  for(integer k=0;k<17;k++)if(stage_read_cal_state[k]===1\'b1)stage_cycles[k]++;',
              '  if(stage_read_cal_state!==last_read_stage)stage_changes++;',
              '  last_read_stage=stage_read_cal_state;',
              '  if(stage_init_state!==last_init_stage)begin',
              '    $display("PUBLIC_INIT_STAGE time_us=%0.6f onehot=%h",$realtime/1000.0,stage_init_state);',
              '    last_init_stage=stage_init_state;',
              '  end',
              'end',
              'initial forever begin #100000;',
              '  $display("PUBLIC_STAGE time_us=%0.6f init=%h read=%h write=%h read_sel=%h burst_seen=%h burst_check=%h burst_counts=%h dll=%h eye_err=%h eye_done=%b eye_valid=%b changes=%0d",',
              '    $realtime/1000.0,stage_init_state,stage_read_cal_state,stage_wrlvl_state,stage_read_sel,stage_burst_seen,stage_burst_check,stage_burst_counts,stage_dll_codes,stage_eye_errors,stage_eye_done,stage_eye_valid,stage_changes);']
    # The first item in a Verilog concatenation occupies the highest bit.
    for bit,name in enumerate(reversed(groups['read_cal_state'])):
        lines.append(f'  $display("PUBLIC_READ_DWELL state={name.rsplit(".",1)[1]} ui_cycles=%0d",stage_cycles[{bit}]);')
    lines.append('end')
    schema=dict(public_wire_groups=groups,
                state_bits_lsb_first={k:list(reversed(v)) for k,v in groups.items()},
                dwell_labels_match_bits=True,
                extras={k:dict(net=n,width=w) for k,(n,w) in extras.items()},
                protected_DQS_access=False,changes_drive_no_signals=True)
    return '\n'.join(lines)+'\n',schema


def decode_samples(text, schema):
    """Decode existing snapshots, including the first monitor's reversed labels.

    The captured vectors are unchanged. Older dwell labels used the ascending
    declaration list for descending concatenation bits; correct labels here
    without rerunning or changing the active simulation.
    """
    groups=schema['public_wire_groups']
    bits=schema.get('state_bits_lsb_first',
                    {k:list(reversed(v)) for k,v in groups.items()})
    samples=[]
    for line in text.splitlines():
        if 'PUBLIC_STAGE time_us=' in line:
            fields=dict(re.findall(r'(\w+)=([^\s]+)',line))
            row=dict(time_us=float(fields['time_us']),raw=line)
            for field,group in [('init','init_state'),('read','read_cal_state'),
                                ('write','wrlvl_state')]:
                value=fields[field]
                row[group]=(None if re.search('[xz]',value,re.I) else
                    [name.rsplit('.',1)[1] for bit,name in enumerate(bits[group])
                     if int(value,16)&(1<<bit)])
            row['fields']=fields
            row['read_dwell_ui_cycles']={}
            samples.append(row)
        elif samples and 'PUBLIC_READ_DWELL state=' in line:
            m=re.search(r'state=(\w+) ui_cycles=(\d+)',line)
            if not m:continue
            name=m[1]
            if not schema.get('dwell_labels_match_bits',False):
                index=[n.rsplit('.',1)[1] for n in groups['read_cal_state']].index(name)
                name=bits['read_cal_state'][index].rsplit('.',1)[1]
            samples[-1]['read_dwell_ui_cycles'][name]=int(m[2])
    return dict(samples=samples,
        old_dwell_labels_corrected=not schema.get('dwell_labels_match_bits',False),
        scope='published state observations only; no calibration success, failure root cause or service bound inferred')
