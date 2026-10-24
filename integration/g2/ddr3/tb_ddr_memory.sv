`timescale 1ns/1ps
module tb_ddr_memory;
    reg clk=0,ui_clk=0,cold_rst=1,calib=0;
    always #10 clk=~clk;
    always #3.7 ui_clk=~ui_clk;
    reg req=0;reg[14:0]wa=0;reg[31:0]wd=0;reg[3:0]be=0;
    wire ack,initialized,line_req,line_we,line_ack,fault;
    wire[31:0]q,line_mask;wire[24:0]la;wire[255:0]lw,lq;
    wire[2:0]cmd;wire en,cr,we,last,wr,rv,re;
    wire[27:0]a;wire[255:0]w,r;wire[31:0]mask;
    nav_ddr_cache mem(.clk(clk),.cold_rst(cold_rst),.req(req),.word_addr(wa),.wdata(wd),.wstrb(be),
        .ack(ack),.rdata(q),.initialized(initialized),.line_req(line_req),.line_we(line_we),
        .line_addr(la),.line_wdata(lw),.line_mask(line_mask),.line_ack(line_ack),.line_rdata(lq));
    nav_ddr_native bridge(clk,ui_clk,cold_rst,line_req,line_we,la,lw,line_mask,line_ack,lq,
        calib,cmd,en,a,cr,w,mask,we,last,wr,r,rv,re,fault);
    ddr_native_model model(ui_clk,cold_rst,calib,cmd,en,a,cr,w,mask,we,last,wr,r,rv,re);
    reg[31:0]gold[0:32767];reg[31:0]rng=32'h4500138c;
    integer checks=0,writes=0,reads=0,holds=0,core_cycles=0;
    integer before_cmds,misses,hits;
    always@(posedge clk)core_cycles++;
    task access(input[14:0]x,input[31:0]d,input[3:0]s);
        integer timeout;
        begin
            @(negedge clk);wa=x;wd=d;be=s;req=1;timeout=0;
            while(!ack)begin @(negedge clk);timeout++;if(timeout>1000)$fatal(1,"memory timeout %d",x);end
            if(s==0)begin if(q!==gold[x])$fatal(1,"read %d: %h != %h",x,q,gold[x]);reads++;end
            else begin for(integer b=0;b<4;b++)if(s[b])gold[x][8*b+:8]=d[8*b+:8];writes++;end
            checks++;
            // Requests are allowed to remain asserted after ACK: exactly once.
            repeat(rng[2:0])begin @(negedge clk);if(!ack)$fatal(1,"ACK dropped before req");holds++;end
            req=0;while(ack)@(negedge clk);
        end
    endtask
    initial begin
        $readmemh("ram_w.hex",gold);
        repeat(5)@(negedge clk);cold_rst=0;
        repeat(70)@(negedge clk);
        if(initialized || model.writes!=0 || en || we)$fatal(1,"access before calibration");
        calib=1;
        wait(initialized);repeat(8)@(negedge clk);
        if(model.writes!=4096)$fatal(1,"boot burst count %d",model.writes);
        for(integer i=0;i<32768;i++)begin
            if(model.mem[i/8][(i%8)*32+:32]!==gold[i])$fatal(1,"boot mismatch %d",i);
            access(i,0,0);
        end
        before_cmds=model.reads;
        repeat(12)access(32767,0,0);
        if(model.reads!=before_cmds)$fatal(1,"read hits used DDR");
        // All byte masks, first/last addresses, cache tag conflicts and eviction.
        for(integer m=1;m<16;m++)begin
            access(0,32'h89abcdef^m,m);access(0,0,0);
            access(32767,32'h12345678^m,m);access(32767,0,0);
            access(1024,32'hf00d0080^m,m);access(1024,0,0);access(0,0,0);
        end
        for(integer k=0;k<2048;k++)begin
            rng=rng^(rng<<13);rng=rng^(rng>>17);rng=rng^(rng<<5);
            access(rng[14:0],rng^32'h87654321,rng[19:16]);
            access(rng[14:0],0,0);
        end
        // CPU warm reset is outside the backend: neither memory nor cache is
        // reset. A later SoC test drives that actual signal while traffic runs.
        for(integer i=0;i<32768;i++)
            if(model.mem[i/8][(i%8)*32+:32]!==gold[i])$fatal(1,"write-through mismatch %d",i);
        repeat(10)@(negedge clk);
        calib=0;repeat(12)@(negedge clk);
        if(!fault)$fatal(1,"calibration loss not latched");
        calib=1;repeat(12)@(negedge clk);
        if(!fault || en || we)$fatal(1,"automatic unsafe recovery");
        if(model.command_stalls==0||model.data_stalls==0||model.data_first==0||model.command_first==0)
            $fatal(1,"backpressure coverage missing");
        $display("TEST PASSED memory checks=%0d reads=%0d writes=%0d holds=%0d core_cycles=%0d native_reads=%0d native_writes=%0d command_stalls=%0d data_stalls=%0d data_first=%0d command_first=%0d",checks,reads,writes,holds,core_cycles,model.reads,model.writes,model.command_stalls,model.data_stalls,model.data_first,model.command_first);
        $finish;
    end
    initial begin #100000000;$fatal(1,"global test timeout");end
endmodule
