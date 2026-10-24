`timescale 1ns/1ps
module tb_structure_store;
    reg clk=0,ui_clk=0;always #10 clk=~clk;always #5 ui_clk=~ui_clk;
    reg rst=1,calib=1,req=0,we=0,start=0,valid=0,finish=0;
    reg[15:0]word_addr=0,record_addr=0;
    reg[17:0]wdata=0;reg[7:0]meta=0;reg signed[31:0]cx=0,cy=0;
    wire ack,record_ready,ready,busy,done,fault;wire[17:0]q;
    wire[2:0]cmd;wire cmd_en,cmd_ready,wen,wend,wready,rv,re;
    wire[27:0]addr;wire[255:0]wd,rd;wire[31:0]wm;
    integer refresh_phase=0,ready_period=8,read_delay=12;
    nav_ddr_structure_store dut(clk,ui_clk,rst,calib,req,we,word_addr,wdata,ack,q,
        start,cx,cy,valid,record_addr,meta,finish,record_ready,ready,busy,done,fault,
        cmd,cmd_en,addr,cmd_ready,wd,wm,wen,wend,wready,rd,rv,re);
    row_service_model dram(ui_clk,rst,calib,cmd,cmd_en,addr,cmd_ready,wd,wm,wen,wend,wready,
        rd,rv,re,refresh_phase,ready_period,read_delay);
    reg[17:0]initial_words[0:65535],expected[0:65535];reg[23:0]records[0:1023];
    integer cycles=0,checks=0,words=0,positive_max=0,negative_max=0;
    integer fd,ncase,nrec,cxi,cyi,offset,period,latency,expect_5ms,started;
    integer barrier_started,barrier_max=0,barrier_cycles;
    string first_file,gold_file,record_file;
    always@(posedge clk)begin
        cycles++;
        if(!rst&&dut.u_update.dma_req&&!dut.own_ack2)$fatal(1,"row transaction before ownership acknowledgement");
        if(!rst&&dut.own2&&(dut.ce||dut.cwe))$fatal(1,"cache native traffic during row ownership");
    end
    task automatic access(input bit write_,input integer a,input[17:0]d,input[17:0]gold);
        integer waited;
        begin
            @(negedge clk);req=1;we=write_;word_addr=a;wdata=d;waited=0;
            while(!ack)begin @(posedge clk);#1;waited++;if(waited>1000)$fatal(1,"cache timeout");end
            if(!write_&&q!==gold)$fatal(1,"cache mismatch a=%h got=%h expected=%h",a,q,gold);
            repeat(a%3)begin @(posedge clk);#1;if(!ack||(!write_&&q!==gold))$fatal(1,"held ack changed");end
            @(negedge clk);req=0;
            while(ack)begin @(posedge clk);#1;end
            checks++;
        end
    endtask
    initial begin
        fd=$fopen("cases.txt","r");if($fscanf(fd,"%d",ncase)!=1)$fatal(1,"case count");
        repeat(16)@(negedge clk);rst=0;wait(ready);
        for(integer c=0;c<ncase;c++)begin
            if($fscanf(fd,"%s %s %s %d %d %d %d %d %d %d",first_file,gold_file,record_file,nrec,cxi,cyi,offset,period,latency,expect_5ms)!=10)
                $fatal(1,"case parse");
            $readmemh(first_file,initial_words);$readmemh(gold_file,expected);
            if(nrec>0)$readmemh(record_file,records,0,nrec-1);
            for(integer a=0;a<8192;a++)dram.mem[a]=0;
            for(integer a=0;a<65536;a++)dram.mem[(a/256)*19+(a%256)/14][((a%256)%14)*18+:18]=initial_words[a];
            refresh_phase=offset;ready_period=period;read_delay=latency;
            if(c==0)begin
                // Real value changes, not writes of the old backing value.
                for(integer a=0;a<65536;a++)
                    dram.mem[(a/256)*19+(a%256)/14][((a%256)%14)*18+:18]=initial_words[a]^18'h15555;
                for(integer a=0;a<65536;a++)access(0,a,0,initial_words[a]^18'h15555);
                for(integer a=0;a<65536;a++)access(1,a,initial_words[a],0);
                for(integer a=65535;a>=0;a--)access(0,a,0,initial_words[a]);
                for(integer a=0;a<65536;a++)
                    if(dram.mem[(a/256)*19+(a%256)/14][((a%256)%14)*18+:18]!==initial_words[a])
                        $fatal(1,"full-capacity write-back failed a=%h",a);
                for(integer l=0;l<256;l++)if(dut.u_cache.meta[l][5])
                    $fatal(1,"test reset would discard a dirty cache line");
                @(negedge clk);rst=1;repeat(12)@(negedge clk);rst=0;wait(ready);
            end
            // Each resident index is now dirty. Include complete write-back
            // and invalidation before the timed streaming update begins.
            for(integer l=0;l<256;l++)begin
                automatic integer a=(l/19)*256+(l%19)*14;
                // Seed a different backing value with the cache cold/empty;
                // only draining the dirty line can restore the gold input.
                dram.mem[l][0+:18]=initial_words[a]^18'h22222;
                access(1,a,initial_words[a],0);
            end
            @(negedge clk);cx=cxi;cy=cyi;started=cycles;start=1;
            if(c==2)begin req=1;we=0;word_addr=65535;end
            @(negedge clk);start=0;
            if(c==2)begin
                // Same-cycle acceptance: an existing client miss must finish
                // with OLD data before ownership can move to the updater.
                wait(ack);#1;
                if(q!==initial_words[65535]||dut.own_ack2)
                    $fatal(1,"in-flight client was lost or crossed UPDATE ownership");
                @(negedge clk);req=0;wait(!ack);checks++;
            end
            wait(record_ready);barrier_cycles=cycles-started;
            if(barrier_cycles>barrier_max)barrier_max=barrier_cycles;
            for(integer k=0;k<nrec;k++)begin
                @(negedge clk);valid=1;record_addr=records[k][23:8];meta=records[k][7:0];
                do begin @(posedge clk);end while(!record_ready);
                @(negedge clk);valid=0;repeat(12)@(negedge clk);
            end
            @(negedge clk);finish=1;
            @(negedge clk);finish=0;
            // A client holds a request through UPDATE. It must receive the
            // NEW map word, never a stale pre-update hit or another owner ACK.
            @(negedge clk);req=1;we=0;word_addr=0;
            while(!done)begin
                @(posedge clk);#1;
                if(ack&&!done)$fatal(1,"client acknowledged before exclusive UPDATE completed");
            end
            $display("COHERENCE_TIMING case=%0d cycles=%0d us_at_50MHz=%0f drain_and_start_cycles=%0d expected_meets_5ms=%0d",c,cycles-started,(cycles-started)/50.0,barrier_cycles,expect_5ms);
            if(expect_5ms&&cycles-started>positive_max)positive_max=cycles-started;
            if(!expect_5ms&&cycles-started>negative_max)negative_max=cycles-started;
            if(((cycles-started<=250000)?1:0)!=expect_5ms)$fatal(1,"5ms service expectation");
            wait(ack);#1;if(q!==expected[0])$fatal(1,"stale cache survived UPDATE");
            @(negedge clk);req=0;wait(!ack);
            for(integer a=0;a<65536;a++)begin
                if(dram.mem[(a/256)*19+(a%256)/14][((a%256)%14)*18+:18]!==expected[a])
                    $fatal(1,"backing mismatch after dirty barrier case=%0d a=%h",c,a);
                words++;
            end
            if(c==0)for(integer a=0;a<65536;a++)access(0,a,0,expected[a]);
            else for(integer l=0;l<4864;l++)begin
                automatic integer a=(l/19)*256+(l%19)*14;
                access(0,a,0,expected[a]);
            end
            access(0,65535,0,expected[65535]);
            // Cold-reset only after the barrier/update drained all dirty
            // entries. This is not a warm CPU reset and does not clear DDR.
            @(negedge clk);rst=1;repeat(12)@(negedge clk);rst=0;wait(ready);
        end
        $fclose(fd);
        // Loss of calibration after admission must latch and not create an
        // ACK or clear the fault merely because calibration returns.
        @(negedge clk);req=1;word_addr=513;we=0;
        wait(cmd_en&&cmd_ready);@(negedge ui_clk);calib=0;
        repeat(30)@(negedge clk);
        if(!fault||ack||done||ready)$fatal(1,"calibration loss not fail-closed");
        calib=1;repeat(30)@(negedge clk);
        if(!fault||ack||done||ready)$fatal(1,"calibration fault auto recovered");
        $display("TEST PASSED cases=%0d client_checks=%0d backing_words=%0d positive_max_cycles=%0d negative_cycles=%0d maximum_drain_start_cycles=%0d",ncase,checks,words,positive_max,negative_max,barrier_max);
        $finish;
    end
    initial begin #1000000000;$fatal(1,"isolated simulation time budget");end
endmodule
