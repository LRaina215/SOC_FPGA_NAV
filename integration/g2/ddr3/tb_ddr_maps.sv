`timescale 1ns/1ps
module tb_ddr_maps;
    reg clk=0,ui_clk=0;
    always #10 clk=~clk;
    always #5 ui_clk=~ui_clk;
    reg rst=1,calib=1;
    reg sr=0,sw=0,er=0,ew=0,flush_req=0;
    reg[15:0]sa=0;reg[17:0]sd=0;wire[17:0]sq;
    reg[13:0]ea=0;reg[52:0]ed=0;wire[52:0]eq;
    wire sack,eack,flush_done,fault;
    wire[2:0]cmd;wire cmd_en,cmd_ready,wr_en,wr_end,wr_ready,rd_valid,rd_end;
    wire[27:0]addr;wire[255:0]wd,rd;wire[31:0]wm;
    nav_ddr_maps_probe dut(clk,ui_clk,rst,sr,sw,sa,sd,sack,sq,
        er,ew,ea,ed,eack,eq,flush_req,flush_done,calib,fault,
        cmd,cmd_en,addr,cmd_ready,wd,wm,wr_en,wr_end,wr_ready,rd,rd_valid,rd_end);
    ddr_map_native_model dram(ui_clk,rst,calib,cmd,cmd_en,addr,cmd_ready,
        wd,wm,wr_en,wr_end,wr_ready,rd,rd_valid,rd_end);
    reg[17:0]gold_sm[0:65535];
    reg[52:0]gold_em[0:16383];
    integer sm_checks=0,em_checks=0,sm_max=0,em_max=0;
    integer cycle=0,line_start=0,line_min=999999,line_max=0,line_total=0,line_checks=0;
    reg previous_req=0,previous_ack=0;
    always@(posedge clk)begin
        cycle++;
`ifndef MAPPED_MAPS
        if(!rst)begin
            if(dut.native_req && !previous_req)line_start=cycle;
            if(dut.native_ack && !previous_ack)begin
                line_checks++;line_total+=cycle-line_start;
                if(cycle-line_start>line_max)line_max=cycle-line_start;
                if(cycle-line_start<line_min)line_min=cycle-line_start;
            end
        end
        previous_req<=dut.native_req;previous_ack<=dut.native_ack;
`endif
        if(fault)$fatal(1,"native fault");
    end
    task automatic sm(input bit write_,input integer address_,input[17:0]data_);
        integer wait_,hold_;reg[17:0]expected;
        begin
            expected=gold_sm[address_];
            @(negedge clk);#1;sr=1;sw=write_;sa=address_;sd=data_;
            wait_=0;
            while(!sack)begin @(posedge clk);#1;wait_++;
                if(wait_>1000)$fatal(1,"SM timeout");end
            if(!write_ && sq!==expected)$fatal(1,"SM mismatch a=%h got=%h expected=%h",address_,sq,expected);
            if(write_)gold_sm[address_]=data_;
            if(wait_>sm_max)sm_max=wait_;
            sm_checks++;
            repeat(address_%3)begin @(posedge clk);#1;
                if(!sack || (!write_ && sq!==expected))$fatal(1,"SM held request changed");end
            @(negedge clk);#1;sr=0;
            while(sack)begin @(posedge clk);#1;end
        end
    endtask
    task automatic em(input bit write_,input integer address_,input[52:0]data_);
        integer wait_;reg[52:0]expected;
        begin
            expected=gold_em[address_];
            @(negedge clk);#1;er=1;ew=write_;ea=address_;ed=data_;
            wait_=0;
            while(!eack)begin @(posedge clk);#1;wait_++;
                if(wait_>1000)$fatal(1,"EM timeout");end
            if(!write_ && eq!==expected)$fatal(1,"EM mismatch a=%h got=%h expected=%h",address_,eq,expected);
            if(write_)gold_em[address_]=data_;
            if(wait_>em_max)em_max=wait_;
            em_checks++;
            repeat(address_%3)begin @(posedge clk);#1;
                if(!eack || (!write_ && eq!==expected))$fatal(1,"EM held request changed");end
            @(negedge clk);#1;er=0;
            while(eack)begin @(posedge clk);#1;end
        end
    endtask
    initial begin
        for(integer i=0;i<65536;i++)gold_sm[i]=0;
        for(integer i=0;i<16384;i++)gold_em[i]=0;
        repeat(12)@(negedge clk);rst=0;
`ifdef MAPPED_MAPS
        repeat(520)@(negedge clk);
`else
        wait(dut.u_sm.state==0 && dut.u_em.state==0);
`endif
        // Full original capacity, all bits, no reduced map in this test.
        for(integer i=0;i<65536;i++)sm(0,i,0);
        for(integer i=0;i<16384;i++)em(0,i,0);
        for(integer i=0;i<65536;i++)sm(1,i,18'h2aaaa^i);
        for(integer i=0;i<16384;i++)em(1,i,53'h15000000000001^(53'h12345679*i));
        for(integer i=65535;i>=0;i--)sm(0,i,0);
        for(integer i=16383;i>=0;i--)em(0,i,0);
        // Conflict evictions, simultaneous clients, and maximum signed bits.
        fork
            for(integer i=0;i<2048;i++)begin
                sm(1,(i*1024+i%8)&65535,18'h3ffff^i);
                sm(0,(i*1024+i%8)&65535,0);
            end
            for(integer i=0;i<2048;i++)begin
                em(1,(i*1024+i%4)&16383,53'h1fffffffffffff^i);
                em(0,(i*1024+i%4)&16383,0);
            end
        join
        @(negedge clk);#1;flush_req=1;
        wait(flush_done);
        for(integer i=0;i<65536;i++)
            if(dram.mem[i>>3][(i&7)*32+:18]!==gold_sm[i])$fatal(1,"SM flush mismatch %d",i);
        for(integer i=0;i<16384;i++)
            if(dram.mem[8192+(i>>2)][(i&3)*64+:53]!==gold_em[i])$fatal(1,"EM flush mismatch %d",i);
        @(negedge clk);#1;flush_req=0;
        repeat(8)@(negedge clk);rst=1;
        repeat(8)@(negedge clk);rst=0;
`ifdef MAPPED_MAPS
        repeat(520)@(negedge clk);
`else
        wait(dut.u_sm.state==0 && dut.u_em.state==0);
`endif
        sm(0,0,0);sm(0,65535,0);em(0,0,0);em(0,16383,0);
        $display("TEST PASSED sm_checks=%0d em_checks=%0d sm_max_cycles=%0d em_max_cycles=%0d native_lines=%0d native_min_cycles=%0d native_max_cycles=%0d native_total_cycles=%0d reads=%0d writes=%0d",sm_checks,em_checks,sm_max,em_max,line_checks,line_min,line_max,line_total,dram.reads,dram.writes);
        $finish;
    end
    initial begin #100000000;$fatal(1,"simulation budget");end
endmodule
