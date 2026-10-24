`timescale 1ns/1ps
// Native-interface service envelope, NOT vendor PHY/WCET proof.
module row_service_model(
    input wire clk,rst,calib,input wire[2:0]cmd,input wire cmd_en,
    input wire[27:0]addr,output wire cmd_ready,
    input wire[255:0]wd,input wire[31:0]wm,input wire wen,wend,
    output wire wready,output reg[255:0]rd,output reg rv,re,
    input integer refresh_phase,input integer ready_period,input integer read_delay
);
    reg[255:0]mem[0:8191];
    integer t=0,delay_left=0;
    bit pending=0,data_pending=0;
    reg[2:0]held_cmd;reg[12:0]held_addr;
    reg[255:0]held_wd;reg[31:0]held_wm;
    integer reads=0,writes=0,refresh_stalls=0,command_stalls=0;
    wire refresh=((t+refresh_phase)%390)<26;
    assign cmd_ready=calib&&!refresh&&!pending&&(t%ready_period==0);
    assign wready=calib&&!refresh&&!data_pending&&(t%7==3);
    always@(posedge clk)begin
        t<=t+1;rv<=0;re<=0;
        if(rst)begin pending<=0;data_pending<=0;delay_left<=0;end
        else begin
            if(cmd_en&&!cmd_ready)command_stalls++;
            if(refresh)refresh_stalls++;
            if(cmd_en&&cmd_ready)begin
                if(addr[27:16]!=0||addr[2:0]!=0)$fatal(1,"bad native address");
                if(cmd!=0&&cmd!=1)$fatal(1,"bad command");
                held_cmd<=cmd;held_addr<=addr[15:3];pending<=1;delay_left<=read_delay;
            end
            if(wen&&wready)begin
                if(!wend)$fatal(1,"missing write end");
                held_wd<=wd;held_wm<=wm;data_pending<=1;
            end
            if(pending&&!refresh)begin
                if(delay_left>0)delay_left<=delay_left-1;
                else if(held_cmd==1)begin
                    rd<=mem[held_addr];rv<=1;re<=1;pending<=0;reads++;
                end else if(data_pending)begin
                    for(integer b=0;b<32;b++)if(!held_wm[b])mem[held_addr][b*8+:8]<=held_wd[b*8+:8];
                    pending<=0;data_pending<=0;writes++;
                end
            end
        end
    end
endmodule

module tb_slf_row_update;
    reg clk=0,ui_clk=0;always #10 clk=~clk;always #5 ui_clk=~ui_clk;
    reg rst=1,calib=1,start=0,valid=0,finish=0;
    reg signed[31:0]cx=0,cy=0;
    reg[15:0]record_addr=0;reg[7:0]meta=0;
    wire record_ready,ready,busy,done,fault;
    wire[2:0]cmd;wire cmd_en,cmd_ready,wen,wend,wready,rv,re;
    wire[27:0]addr;wire[255:0]wd,rd;wire[31:0]wm;
    integer refresh_phase=0,ready_period=8,read_delay=12;
    nav_slf_row_update dut(clk,ui_clk,rst,calib,start,cx,cy,valid,record_addr,meta,finish,
        record_ready,ready,busy,done,fault,cmd,cmd_en,addr,cmd_ready,wd,wm,wen,wend,wready,rd,rv,re);
    row_service_model dram(ui_clk,rst,calib,cmd,cmd_en,addr,cmd_ready,wd,wm,wen,wend,wready,
        rd,rv,re,refresh_phase,ready_period,read_delay);
    reg[17:0]initial_words[0:65535],expected[0:65535];
    reg[23:0]records[0:1023];
    integer cycles=0,checks=0,max_cycles=0,started=0,ce_stalls=0;
    integer fd,ncase,nrec,cxi,cyi,offset,period,latency,expect_5ms,case_stalls;
    string first_file,gold_file,record_file;
    always@(posedge clk)begin
        cycles++;
        if(fault)$fatal(1,"unexpected DDR fault");
`ifndef MAPPED_ROW_UPDATE
        if(dut.phase==4&&dut.lf_busy&&!dut.ce)ce_stalls++;
        if(!rst&&dut.ce&&dut.u_lf.v1&&dut.u_lf.inr1&&$isunknown(dut.lf_q))
            $fatal(1,"unknown LF read row=%0d col=%0d pending=%b",dut.u_lf.r1,dut.u_lf.c1,dut.read_pending);
        if(!rst&&dut.ce&&dut.lw&&$isunknown(dut.lwd))
            $fatal(1,"unknown LF write address=%h data=%h",dut.lwa,dut.lwd);
        if(!rst&&wen&&wready&&$isunknown(wd))
            $fatal(1,"unknown packed row=%0d beat=%0d data=%h",dut.dma_row,dut.u_dma.data_sent,wd);
        if(!rst&&dut.dma_req&&!dut.dma_we&&!dut.dma_ack&&dut.ce&&dut.lr&&
            dut.dma_bank==dut.la[8])$fatal(1,"read DMA overwrites live LF row");
        if(!rst&&dut.up_state!=0&&dut.ce&&dut.lr&&dut.up_bank==dut.la[8])
            $fatal(1,"update writes live LF row");
`endif
    end
    initial begin
        fd=$fopen("cases.txt","r");
        if(!fd)$fatal(1,"missing cases");
        if($fscanf(fd,"%d",ncase)!=1)$fatal(1,"case count");
        repeat(16)@(negedge clk);rst=0;
        wait(ready);
        for(integer c=0;c<ncase;c++)begin
            if($fscanf(fd,"%s %s %s %d %d %d %d %d %d %d",first_file,gold_file,record_file,nrec,cxi,cyi,offset,period,latency,expect_5ms)!=10)
                $fatal(1,"case parse");
            $readmemh(first_file,initial_words);$readmemh(gold_file,expected);
            if(nrec>0)$readmemh(record_file,records,0,nrec-1);
            for(integer a=0;a<8192;a++)dram.mem[a]=0;
            for(integer a=0;a<65536;a++)
                dram.mem[(a/256)*19+(a%256)/14][((a%256)%14)*18+:18]=initial_words[a];
            @(negedge clk);cx=cxi;cy=cyi;refresh_phase=offset;
            ready_period=period;read_delay=latency;started=cycles;start=1;case_stalls=ce_stalls;
            @(negedge clk);start=0;
            wait(record_ready);
            for(integer k=0;k<nrec;k++)begin
                @(negedge clk);valid=1;record_addr=records[k][23:8];meta=records[k][7:0];
                do begin @(posedge clk);end while(!record_ready);
                @(negedge clk);valid=0;
                // Retain a conservative original point-transform allowance:
                // records arrive at least fourteen core cycles apart.
                repeat(12)@(negedge clk);
            end
            @(negedge clk);finish=1;
            @(negedge clk);finish=0;
            wait(done);#1;
            $display("ROW_TIMING case=%0d records=%0d cycles=%0d us_at_50MHz=%0f refresh_phase=%0d ready_period=%0d read_delay=%0d stalls=%0d expected_meets_5ms=%0d",c,nrec,cycles-started,(cycles-started)/50.0,offset,period,latency,ce_stalls-case_stalls,expect_5ms);
            if(cycles-started>max_cycles)max_cycles=cycles-started;
            for(integer a=0;a<65536;a++)begin
                if(dram.mem[(a/256)*19+(a%256)/14][((a%256)%14)*18+:18]!==expected[a])
                    $fatal(1,"full map mismatch case=%0d address=%h got=%h gold=%h",c,a,dram.mem[(a/256)*19+(a%256)/14][((a%256)%14)*18+:18],expected[a]);
                checks++;
            end
            if(((cycles-started<=250000)?1:0)!=expect_5ms)
                $fatal(1,"5ms service-envelope expectation changed");
            repeat(10)@(negedge clk);
        end
        $fclose(fd);
        $display("TEST PASSED cases=%0d words=%0d maximum_update_cycles=%0d reads=%0d writes=%0d refresh_cycles=%0d command_stalls=%0d",ncase,checks,max_cycles,dram.reads,dram.writes,dram.refresh_stalls,dram.command_stalls);
        $finish;
    end
    initial begin #100000000;$fatal(1,"bounded isolated simulation budget");end
endmodule
