`timescale 1ns/1ps
module tb_interleaved_cache;
    reg clk=0;always #10 clk=~clk;
    reg rst=1,req=0,we=0,barrier=0,peek_req=0,peek_go=0;
    reg[15:0]a=0,peek_addr=0;reg[17:0]d=0;
    wire ack,bd,lr,lw,peek_hit;wire[17:0]q,peek_q;
    wire[24:0]la;wire[255:0]ld;
    reg line_ack=0;reg[255:0]line_q=0;
    reg[255:0]backing[0:8191];reg[17:0]gold[0:65535];
    reg[24:0]held_addr;reg held_we;reg[255:0]held_data;
    integer service=0,delay_left=0,checks=0,peeks=0,backing_checks=0,cycles=0;
`ifdef MAPPED_CACHE
    GSR GSR(.GSRI(1'b1));
    g2_top dut(
`else
    nav_ddr_struct_cache_fast dut(
`endif
        .clk(clk),.rst(rst),.req(req),.we(we),.word_addr(a),.wdata(d),
        .ack(ack),.rdata(q),.barrier_req(barrier),.barrier_done(bd),
        .line_req(lr),.line_we(lw),.line_addr(la),.line_wdata(ld),
        .line_ack(line_ack),.line_rdata(line_q),.peek_req(peek_req),
        .peek_go(peek_go),.peek_addr(peek_addr),.peek_hit(peek_hit),.peek_q(peek_q));
    always@(posedge clk)begin
        cycles<=cycles+1;
        if(rst)begin service<=0;line_ack<=0;end
        else case(service)
            0:if(lr)begin
                if(la>=4864)$fatal(1,"cache line outside full structure map");
                held_addr<=la;held_we<=lw;held_data<=ld;
                delay_left<=1+la%5;service<=1;
            end
            1:if(delay_left!=0)delay_left<=delay_left-1;
                else begin
                    if(held_we)backing[held_addr]<=held_data;
                    else line_q<=backing[held_addr];
                    line_ack<=1;service<=2;
                end
            2:if(!lr)begin line_ack<=0;service<=0;end
        endcase
    end
    task automatic access(input bit wr,input[15:0]address,input[17:0]value);
        integer waited;
        begin
            @(negedge clk);req=1;we=wr;a=address;d=value;waited=0;
            do begin @(posedge clk);#1;waited++;
                if(waited>1000)$fatal(1,"cache request timeout");end while(!ack);
            if(wr)gold[address]=value;
            else if(q!==gold[address])$fatal(1,"word mismatch at%h got%h expected%h",address,q,gold[address]);
            checks++;
            @(negedge clk);req=0;do begin @(posedge clk);#1;end while(ack);
            repeat(2)@(negedge clk);
        end
    endtask
    task automatic read_peek(input[15:0]address);
        begin
            access(0,address,0);
            @(negedge clk);peek_req=1;peek_go=1;peek_addr=address;#1;
            if(!peek_hit)$fatal(1,"resident peek not a hit at%h",address);
            @(posedge clk);#1;
            if(peek_q!==gold[address])$fatal(1,"single-cycle peek mismatch at%h",address);
            peeks++;@(negedge clk);peek_req=0;peek_go=0;
        end
    endtask
    integer i;reg[17:0]pattern;
    initial begin
        for(i=0;i<8192;i++)backing[i]='1;
        for(i=0;i<65536;i++)begin
            gold[i]=18'h15555^i;
            backing[(i/256)*19+(i%256)/14][((i%256)%14)*18+:18]=gold[i];
        end
        repeat(5)@(negedge clk);rst=0;repeat(300)@(negedge clk);
        for(i=0;i<65536;i++)access(0,i,0);
        for(i=0;i<65536;i++)begin pattern=18'h2a5a5^(i*321)^(i<<1);access(1,i,pattern);end
        for(i=65535;i>=0;i--)access(0,i,0);
        for(i=0;i<2048;i++)read_peek((i*31)&65535);
`ifdef MAPPED_CACHE
        // Extra mapped-memory coverage: make all256 resident lines dirty
        // with changed data before exercising the four-bank collect path.
        for(i=0;i<256;i++)begin
            pattern=18'h31579^(i*97);
            access(1,(i/19)*256+(i%19)*14,pattern);
        end
`endif
        @(negedge clk);barrier=1;wait(bd);@(negedge clk);
        for(i=0;i<65536;i++)begin
            if(backing[(i/256)*19+(i%256)/14][((i%256)%14)*18+:18]!==gold[i])
                $fatal(1,"dirty barrier backing mismatch at%h",i);
            backing_checks++;
        end
        barrier=0;wait(!bd);repeat(3)@(negedge clk);
        // Cold metadata invalidation must not expose old cache words as hits.
        rst=1;repeat(4)@(negedge clk);rst=0;repeat(300)@(negedge clk);
        for(i=0;i<256;i++)access(0,i*257,0);
        $display("TEST PASSED checks=%0d peek_checks=%0d backing_checks=%0d cycles=%0d",checks,peeks,backing_checks,cycles);
        $finish;
    end
    initial begin #2000000000;$fatal(1,"isolated cache fixture watchdog");end
endmodule
