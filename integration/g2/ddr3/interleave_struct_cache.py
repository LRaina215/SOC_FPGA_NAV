"""Candidate: stripe each existing 14-word cache line over four narrow banks.

The logical mapping, native format and four 1024x18 memories are retained.
Only fill/eviction use all banks in parallel. This is opt-in until all numeric,
ownership, mapped-memory and full-subsystem proofs have been repeated.
"""
from derive_ddr_matcher import derive_fast_cache


def derive(source):
    source = derive_fast_cache(source)

    def one(old, new):
        nonlocal source
        assert source.count(old) == 1, old
        source = source.replace(old, new, 1)

    one('    wire ram_we=hit_write||state==FILL;\n', '')
    one('    wire[17:0]ram_wd=hit_write?saved_data:transfer[slot*18+:18];\n',
        '''    // A physical bank stores one word lane in every four-cell group.
    wire[9:0]bank_addr=(state==FILL||state==COLLECT)?
        {active_index,slot[1:0]}:ram_addr[11:2];
''')
    one('always@(posedge clk)if(ram_re)read_bank<=ram_addr[11:10];',
        'always@(posedge clk)if(ram_re)read_bank<=ram_addr[1:0];')
    one('''        always@(posedge clk)begin
            if(ram_we&&ram_addr[11:10]==b)data[ram_addr[9:0]]<=ram_wd;
            if(ram_re&&ram_addr[11:10]==b)q<=data[ram_addr[9:0]];
        end''', '''        wire[17:0]fill_word;
        if(b<2)begin
            assign fill_word=transfer[(slot[1:0]*4+b)*18+:18];
        end else begin
            assign fill_word=slot==0?transfer[b*18+:18]:
                slot==1?transfer[(4+b)*18+:18]:transfer[(8+b)*18+:18];
        end
        always@(posedge clk)begin
            if(hit_write&&ram_addr[1:0]==b)data[bank_addr]<=saved_data;
            else if(state==FILL&&(slot<3||b<2))data[bank_addr]<=fill_word;
            if((ram_re&&ram_addr[1:0]==b)||state==COLLECT)q<=data[bank_addr];
        end''')
    one('(state==FILL&&slot==13)', '(state==FILL&&slot==3)')
    one('''            CAPTURE:begin transfer[slot*18+:18]<=bank_q[read_bank];
                if(slot==13)state<=WRITE_LINE;else begin slot<=slot+1'b1;state<=COLLECT;end
            end''', '''            CAPTURE:begin
                case(slot)
                    0:transfer[71:0]<={bank_q[3],bank_q[2],bank_q[1],bank_q[0]};
                    1:transfer[143:72]<={bank_q[3],bank_q[2],bank_q[1],bank_q[0]};
                    2:transfer[215:144]<={bank_q[3],bank_q[2],bank_q[1],bank_q[0]};
                    3:transfer[251:216]<={bank_q[1],bank_q[0]};
                endcase
                if(slot==3)state<=WRITE_LINE;else begin slot<=slot+1'b1;state<=COLLECT;end
            end''')
    one('FILL:if(slot==13)', 'FILL:if(slot==3)')
    return source
