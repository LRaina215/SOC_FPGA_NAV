`timescale 1ns/1ps
module tb_ddr_soc;
    reg clk=0,ui_clk=0,rst=1,cold=1,calib=0,pll_locked=1,stop_ui=0;
    always #10 clk=~clk;
    always #5.3 if(!stop_ui)ui_clk=~ui_clk;
    reg[1:0]dr=0,dw=0;reg[15:0]da=0;reg[31:0]dd=0;
    wire[1:0]dk,mreq,mwe,trap;
    wire[31:0]dq[0:1],mdata[0:1];wire[15:0]ma[0:1];wire[3:0]led[0:1];
    wire[2:0]cmd;wire ce,cr,we,wend,wr,rv,rend,init,fault;
    wire[27:0]a;wire[255:0]w,r;wire[31:0]mask;
    nav_soc #(.RAM_KB(128),.INIT_B0("ram_b0.hex"),.INIT_B1("ram_b1.hex"),.INIT_B2("ram_b2.hex"),.INIT_B3("ram_b3.hex"))
        baseline(clk,rst,mreq[0],mwe[0],ma[0],mdata[0],mreq[0],32'h87654321,
        dr[0],dw[0],da,dd,dk[0],dq[0],led[0],trap[0]);
    nav_soc_ddr #(.INIT_B0("ram_b0.hex"),.INIT_B1("ram_b1.hex"),.INIT_B2("ram_b2.hex"),.INIT_B3("ram_b3.hex")) candidate (
        .clk(clk),.rst(rst),.m_req(mreq[1]),.m_we(mwe[1]),.m_addr(ma[1]),.m_wdata(mdata[1]),.m_ack(mreq[1]),.m_rdata(32'h87654321),
        .d_req(dr[1]),.d_we(dw[1]),.d_addr(da),.d_wdata(dd),.d_ack(dk[1]),.d_rdata(dq[1]),.led(led[1]),.trap(trap[1]),
        .ddr_clk(ui_clk),.ddr_cold_rst(cold),.ddr_calib_done(calib),.ddr_pll_locked(pll_locked),.ddr_cmd(cmd),.ddr_cmd_en(ce),.ddr_addr(a),.ddr_cmd_ready(cr),
        .ddr_wr_data(w),.ddr_wr_mask(mask),.ddr_wr_en(we),.ddr_wr_end(wend),.ddr_wr_ready(wr),
        .ddr_rd_data(r),.ddr_rd_valid(rv),.ddr_rd_end(rend),.memory_initialized(init),.memory_fault(fault));
    ddr_native_model model(ui_clk,cold,calib,cmd,ce,a,cr,w,mask,we,wend,wr,r,rv,rend);
    reg[31:0]gold[0:32767],program_image[0:255];
    integer program_words,debug_checks=0,nav_count[0:1],run_cycles[0:1],cycles=0;
    reg[31:0]nav_writes[0:1];reg[1:0]nav_active=0;reg counting=0;
    reg[31:0]expected=0;
    integer before_writes;
    always@(posedge clk)begin
        cycles++;
        for(integer n=0;n<2;n++)begin
            if(counting&&!trap[n])run_cycles[n]++;
            if(mreq[n]&&!nav_active[n])begin
                nav_count[n]++;if(mwe[n])nav_writes[n]=mdata[n];
                if(ma[n]!==16'h0124)$fatal(1,"NAV address changed");
                nav_active[n]=1;
            end
            if(!mreq[n])nav_active[n]=0;
        end
    end
    task debug_access(input bit wrt,input[15:0]address,input[31:0]data,input bit compare);
        bit[1:0]done;integer timeout;
        begin
            @(negedge clk);dr=3;dw=wrt?3:0;da=address;dd=data;done=0;timeout=0;
            while(done!=3)begin
                @(negedge clk);timeout++;
                for(integer n=0;n<2;n++)if(dk[n]&&!done[n])begin
                    done[n]=1;
                    if(!wrt&&compare&&dq[n]!==data)$fatal(1,"debug %h n%0d: %h != %h",address,n,dq[n],data);
                    dr[n]=0;
                end
                if(timeout>2000)$fatal(1,"debug timeout addr=%h done=%b",address,done);
            end
            repeat(5)@(negedge clk);debug_checks++;
        end
    endtask
    initial begin
        nav_count[0]=0;nav_count[1]=0;run_cycles[0]=0;run_cycles[1]=0;
        $readmemh("ram_w.hex",gold);$readmemh("cpu_program.hex",program_image);
        program_words=program_image[255];
        repeat(8)@(negedge clk);cold=0;rst=0;
        debug_access(1,16'hf000,1,0);calib=1;wait(init);
        debug_access(0,16'hf014,32'h00800001,1);
        foreach(gold[i])if(i==0||i==12927||i==12928||i==32767)begin
            debug_access(1,16'hf004,i,0);debug_access(0,16'hf008,gold[i],1);
        end
        // The same debug protocol loads the exact same RISC-V program.
        debug_access(1,16'hf004,0,0);
        for(integer k=0;k<program_words;k++)debug_access(1,16'hf008,program_image[k],0);
        debug_access(1,16'hf004,0,0);
        for(integer k=0;k<program_words;k++)begin
            debug_access(1,16'hf004,k,0);debug_access(0,16'hf008,program_image[k],1);
        end
        for(integer k=0;k<1024;k++)
            expected=expected^(((1024-k)&65535)<<16)^(((1024-k)&255)<<8)^((32'h4500138c+k)&255);
        counting=1;debug_access(1,16'hf000,0,0);
        wait(trap==3);counting=0;
        if(nav_count[0]!=2||nav_count[1]!=2||nav_writes[0]!==expected||nav_writes[1]!==expected)
            $fatal(1,"CPU/NAV result mismatch %0d/%0d %h/%h expected=%h",nav_count[0],nav_count[1],nav_writes[0],nav_writes[1],expected);
        if(led[0]!=15||led[1]!=15)$fatal(1,"CPU completion LED");
        debug_access(0,16'hf010,expected^32'h87654321,1);
        debug_access(1,16'hf000,1,0);
        debug_access(1,16'hf004,32767,0);debug_access(1,16'hf008,32'h81726354,0);
        before_writes=model.writes;
        // A real SoC warm reset must preserve the DDR contents and boot state.
        @(negedge clk);rst=1;repeat(8)@(negedge clk);rst=0;
        debug_access(1,16'hf000,1,0);
        debug_access(1,16'hf004,32767,0);debug_access(0,16'hf008,32'h81726354,1);
        if(!init||model.writes!=before_writes)$fatal(1,"warm reset reinitialized RAM");
        if(fault)$fatal(1,"unexpected native fault");
        // Clock loss must be caught even though the UI domain cannot run.
        stop_ui=1;pll_locked=0;repeat(12)@(negedge clk);
        if(!fault||!candidate.cpu_rst)$fatal(1,"PLL/clock loss failed open");
        pll_locked=1;repeat(12)@(negedge clk);
        if(!fault)$fatal(1,"unsafe automatic PLL recovery");
        $display("TEST PASSED soc debug_checks=%0d program_words=%0d nav_transactions=%0d checksum=%h baseline_run_cycles=%0d ddr_run_cycles=%0d warm_reset_retained=1 clock_loss_latched=1",debug_checks,program_words,nav_count[1],expected,run_cycles[0],run_cycles[1]);
        $finish;
    end
    initial begin #100000000;$fatal(1,"SoC timeout");end
endmodule
