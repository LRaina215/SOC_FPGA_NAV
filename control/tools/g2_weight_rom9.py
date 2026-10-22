"""Lossless 16-bit weights in 9-bit physical ROM modes; no latency change.

Low byte+sign use20x2048x9. Bits13:8 use three byte lanes per four weights,
15x2048x8. Bit14 normally equals sign; record every exception explicitly.
For the frozen40704 words there are3 exceptions, but generation handles all.
"""
from pathlib import Path


def generate(out,weights):
    out=Path(out);mem=out/'src/mem';words=[w&65535 for w in weights]+[0]*(40960-len(weights))
    assert len(words)==40960
    flips=[i for i,w in enumerate(words) if ((w>>14)^(w>>15))&1]
    low=[(w&255)|((w>>15)<<8) for w in words]
    high=[[],[],[]]
    for i in range(0,len(words),4):
        packed=sum(((words[i+j]>>8)&63)<<(j*6) for j in range(4))
        for lane in range(3):high[lane].append((packed>>(lane*8))&255)
    restored=[]
    for i in range(len(words)):
        packed=sum(high[lane][i//4]<<(lane*8) for lane in range(3))
        middle=(packed>>((i%4)*6))&63;sign=low[i]>>8
        restored.append((sign<<15)|((sign^int(i in flips))<<14)|(middle<<8)|(low[i]&255))
    assert restored==words
    lines=['module fixed4500_weight_rom(input wire clk,en,input wire[15:0] addr,output wire signed[15:0] value);',
        'reg[4:0] selected_low;reg[2:0] selected_high;reg[1:0] selected_lane;reg bit14_flip;',
        'reg[8:0] low_q[0:19];reg[7:0] high_q0[0:4],high_q1[0:4],high_q2[0:4];',
        'always@(posedge clk)if(en)begin selected_low<=addr[15:11];selected_high<=addr[15:13];selected_lane<=addr[1:0];',
        'case(addr)']
    lines += [f"16'd{i}:bit14_flip<=1'b1;" for i in flips]
    lines += ["default:bit14_flip<=1'b0;endcase end",
        'wire[8:0] low_word=low_q[selected_low];',
        'wire[23:0] high_word={high_q2[selected_high],high_q1[selected_high],high_q0[selected_high]};',
        "wire[4:0] high_shift={selected_lane,2'b00}+{1'b0,selected_lane,1'b0};",
        'wire[5:0] middle=high_word>>high_shift;',
        'assign value={low_word[8],low_word[8]^bit14_flip,middle,low_word[7:0]};']
    for bank in range(20):
        name=f'weight9_low{bank:02d}.mem';data=low[bank*2048:(bank+1)*2048]
        (mem/name).write_text('\n'.join(f'{w:03x}' for w in data)+'\n')
        lines += [f'(* syn_ramstyle="block_ram",syn_romstyle="block_rom" *) reg[8:0] low_bank{bank}[0:2047];',
            f'initial $readmemh("src/mem/{name}",low_bank{bank});',
            f"always@(posedge clk)if(en && addr[15:11]==5'd{bank})low_q[{bank}]<=low_bank{bank}[addr[10:0]];"]
    for lane in range(3):
        for bank in range(5):
            name=f'weight9_high{lane}_{bank}.mem';data=high[lane][bank*2048:(bank+1)*2048]
            (mem/name).write_text('\n'.join(f'{w:02x}' for w in data)+'\n')
            lines += [f'(* syn_ramstyle="block_ram",syn_romstyle="block_rom" *) reg[7:0] high_bank{lane}_{bank}[0:2047];',
                f'initial $readmemh("src/mem/{name}",high_bank{lane}_{bank});',
                f"always@(posedge clk)if(en && addr[15:13]==3'd{bank})high_q{lane}[{bank}]<=high_bank{lane}_{bank}[addr[12:2]];"]
    lines.append('endmodule');(out/'fixed4500_weight_rom.v').write_text('\n'.join(lines)+'\n',encoding='utf-8')
    return dict(layout='low9_and_three_high_byte_lanes_per_four_words',original_words=len(weights),padded_words=len(words),
        decoded_all_words_equal=True,bit14_exception_addresses=flips,expected_native_bsram=35,
        quantization=False,weights_changed=False,additional_read_cycles=0,
        scope='numeric reconstruction and physical-mode budget only; RTL and synthesis/PNR still required')
