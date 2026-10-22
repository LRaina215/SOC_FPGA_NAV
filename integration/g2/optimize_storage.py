"""Lossless storage layout, leaving original4500 numerical parameters intact.

Keep the original MAC schedule and one-cycle enabled ROM reads. No quantization,
zero-column specialization, control law or training change is performed here.
"""
import json
from pathlib import Path

ROOT=Path(__file__).resolve().parents[2]


def values(path,bits):
    return [int(x,16)-(1<<bits) if int(x,16)&(1<<(bits-1)) else int(x,16) for x in path.read_text().split()]


def prepare(out,directed=False,packed9=False):
    out=Path(out);mem=out/'src/mem';original=ROOT/'control/fpga/src'
    weights=values(original/'mem/weights.mem',16);elu=values(original/'mem/elu.mem',24);bias=values(original/'mem/bias.mem',32)
    assert len(weights)==40704 and len(elu)==3073 and len(bias)==425
    banks=[]
    for i in range(40):
        w=weights[i*1024:(i+1)*1024];w+= [0]*(1024-len(w))
        (mem/f'weights_bank{i:02d}.mem').write_text('\n'.join(f'{v&65535:04x}' for v in w)+'\n')
        assert values(mem/f'weights_bank{i:02d}.mem',16)[:min(1024,40704-i*1024)]==weights[i*1024:(i+1)*1024]
        attr='(* syn_ramstyle="block_ram",syn_romstyle="block_rom" *)' if directed else ''
        banks.append(f'''{attr}reg [15:0] bank{i} [0:1023];
initial $readmemh("src/mem/weights_bank{i:02d}.mem",bank{i});
always @(posedge clk) if(en && addr[15:10]==6'd{i})q[{i}]<=bank{i}[addr[9:0]];''')
    (out/'fixed4500_weight_rom.v').write_text('''module fixed4500_weight_rom(input wire clk,en,input wire [15:0] addr,output wire signed [15:0] value);
reg [5:0] selected;
reg [15:0] q [0:39];
always @(posedge clk) if(en)selected<=addr[15:10];
assign value=q[selected];
'''+ '\n'.join(banks)+'\nendmodule\n')
    packed_report=None
    if packed9:
        import importlib.util
        spec=importlib.util.spec_from_file_location('weight9',ROOT/'control/tools/g2_weight_rom9.py')
        generator=importlib.util.module_from_spec(spec);spec.loader.exec_module(generator)
        packed_report=generator.generate(out,weights)
    tail=next(i for i in range(len(elu)) if all(v==elu[-1] for v in elu[i:]))
    assert elu[-1]==-4096 and min(elu)>=-4096 and max(elu)<=0
    (mem/'elu13.mem').write_text('\n'.join(f'{v&8191:04x}' for v in elu[:tail])+'\n')
    assert values(mem/'elu13.mem',13)+[-4096]*(3073-tail)==elu
    bias_bits=next(b for b in range(1,33) if min(bias)>=-(1<<(b-1)) and max(bias)<(1<<(b-1)))
    (mem/'bias_narrow.mem').write_text('\n'.join(f'{v&((1<<bias_bits)-1):x}' for v in bias)+'\n')
    assert values(mem/'bias_narrow.mem',bias_bits)==bias
    core=(original/'fc_v4_top.v').read_text()
    core=core.replace('parameter ELU_FILE = "src/mem/elu.mem"','parameter ELU_FILE = "src/mem/elu13.mem"').replace('parameter BIASES_FILE = "src/mem/bias.mem"','parameter BIASES_FILE = "src/mem/bias_narrow.mem"')
    core=core.replace('reg signed [WEIGHT_BITS-1:0] weight_mem [0:40703];','')
    core=core.replace('reg signed [31:0] bias_mem [0:424];',f'reg signed [{bias_bits-1}:0] bias_mem [0:424];')
    core=core.replace('reg signed [23:0] elu_mem [0:3072];',f'reg signed [12:0] elu_mem [0:{tail-1}];')
    core=core.replace('reg signed [WEIGHT_BITS-1:0] weight_q;','wire signed [WEIGHT_BITS-1:0] weight_q;\n    fixed4500_weight_rom weights(.clk(clk),.en(state==MAC_REQ),.addr(weight_address),.value(weight_q));')
    core=core.replace('        $readmemh(WEIGHTS_FILE, weight_mem);','').replace('            weight_q <= weight_mem[weight_address];','')
    core=core.replace('if (state == ACT_WAIT) elu_q <= elu_mem[elu_address];',f"if (state == ACT_WAIT) elu_q <= elu_address>={tail} ? -24'sd4096 : $signed(elu_mem[elu_address]);")
    if directed:
        for token in (f'reg signed [{bias_bits-1}:0] bias_mem','reg signed [23:0] input_mem','reg signed [23:0] bank_a','reg signed [23:0] bank_b'):
            assert token in core;core=core.replace(token,'(* syn_ramstyle="distributed_ram",syn_romstyle="distributed_rom" *)'+token)
        core=core.replace('reg signed [12:0] elu_mem','(* syn_ramstyle="block_ram",syn_romstyle="block_rom" *)reg signed [12:0] elu_mem')
    (out/'fc_v4_top.v').write_text(core)
    report=dict(weight_count=len(weights),weight_values_equal=True,weight_banks40x1024x16=not packed9,packed9=packed_report,
        elu_values_equal=True,elu_bits=13,elu_constant_tail_start=tail,bias_values_equal=True,bias_bits=bias_bits,
        original_control_changed=False,training_changed=False,MAC_schedule_changed=False,directed_storage=directed,
        boundary='exact numeric/layout checks; RTL cycle equivalence and joint resource/timing must also pass')
    (out/'storage_layout.json').write_text(json.dumps(report,indent=2));return report
