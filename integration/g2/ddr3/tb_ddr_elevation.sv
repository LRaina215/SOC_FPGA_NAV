`timescale 1ns/1ps
module tb_ddr_elevation;
    reg clk=0,ui_clk=0;always #10 clk=~clk;always #5 ui_clk=~ui_clk;
    reg rst=1,clear=0,valid=0,upd=0,eof=0,swp_sel=0,swp_re=0,swp_we=0,cpu_req=0,flush=0;
    reg[13:0]a=0,swp_ra=0,swp_wa=0,cpu_a=0;reg[7:0]tag=0;reg signed[15:0]z=0;
    reg[52:0]swp_wd=0;
    wire ready,gold_ready,done,gold_done,busy,gold_busy,ack,gold_ack,ce,mem_req,mem_we,mem_ack,ff,fault;
    wire[13:0]mem_a;wire[52:0]q,gold_q,mem_wd,mem_q;wire[7:0]fc,gold_fc;
    elev_map reference_core(clk,rst,clear,valid&&ready, gold_ready,upd,a,tag,z,eof,
        gold_fc,gold_done,gold_busy,swp_sel,swp_re,swp_ra,gold_q,swp_we,swp_wa,swp_wd,cpu_req,cpu_a,gold_ack);
    elev_map_ddr dut(clk,rst,clear,valid,ready,upd,a,tag,z,eof,fc,done,busy,
        swp_sel,swp_re,swp_ra,q,swp_we,swp_wa,swp_wd,cpu_req,cpu_a,ack,
        ce,mem_req,mem_we,mem_a,mem_wd,mem_ack,mem_q,fault);
    wire[2:0]cmd;wire cmd_en,cmd_ready,wen,wend,wready,rv,re;
    wire[27:0]addr;wire[255:0]wd,rd;wire[31:0]wm;
    integer refresh_phase=389,ready_period=16,read_delay=20;
    nav_ddr_elevation_store store(clk,ui_clk,rst,1'b1,mem_req,mem_we,mem_a,mem_wd,mem_ack,mem_q,
        flush,ff,fault,cmd,cmd_en,addr,cmd_ready,wd,wm,wen,wend,wready,rd,rv,re);
    row_service_model dram(ui_clk,rst,1'b1,cmd,cmd_en,addr,cmd_ready,wd,wm,wen,wend,wready,rd,rv,re,
        refresh_phase,ready_period,read_delay);
    integer checks=0,points=0,cycles=0,frames=0,start_cycle,cold_cycles;
    reg[52:0]expected[0:16383];reg[52:0]gold_hold,dut_hold;
    always@(posedge clk)begin cycles++;if(!rst&&fault)$fatal(1,"unexpected memory fault");end
    task automatic point(input bit update_,input bit end_,input integer addr_,tag_,height);
        begin
            @(negedge clk);valid=1;upd=update_;eof=end_;a=addr_;tag=tag_;z=height;
            do begin @(posedge clk);end while(!ready);
            if(!gold_ready)$fatal(1,"reference did not accept identical point");
            @(negedge clk);valid=0;upd=0;eof=0;points++;
        end
    endtask
    task automatic compare_word(input integer addr_);
        reg seen_gold,seen_dut;integer timeout_;
        begin
            @(negedge clk);cpu_req=1;cpu_a=addr_;seen_gold=0;seen_dut=0;timeout_=0;
            while(!seen_gold||!seen_dut)begin
                @(posedge clk);#1;timeout_++;
                if(gold_ack)begin gold_hold=gold_q;seen_gold=1;end
                if(ack)begin dut_hold=q;seen_dut=1;end
                if(timeout_>2000)$fatal(1,"elevation CPU request timeout");
            end
            if(dut_hold!==gold_hold)$fatal(1,"53-bit elevation mismatch a=%h got=%h expected=%h",addr_,dut_hold,gold_hold);
            expected[addr_]=gold_hold;checks++;
            @(negedge clk);cpu_req=0;repeat(4)@(negedge clk);
        end
    endtask
    initial begin
        for(integer l=0;l<16384;l++)dram.mem[l]='1;
        repeat(16)@(negedge clk);rst=0;wait(!busy&&!gold_busy);cold_cycles=cycles;
        $display("ELEV_CLEAR cycles=%0d us_at_50MHz=%0f",cold_cycles,cold_cycles/50.0);
        for(integer f=0;f<4;f++)begin
            start_cycle=cycles;
            for(integer i=0;i<512;i++)begin
                // Same cell, bank conflict, tag alias, signed extrema and
                // saturating count all remain in the original arithmetic.
                point(1,0,i%4==0?15:(i*129)%16384,(i/17+f)%256,
                    i%3==0?-32768:i%3==1?32767:(i*19-f*31)%65536-32768);
            end
            point(0,1,0,0,0);wait(!busy&&!gold_busy);
            if(fc!==gold_fc||fc!==f+1)$fatal(1,"frame count changed");
            for(integer addr_=0;addr_<16384;addr_++)compare_word(addr_);
            frames++;
            $display("ELEV_FRAME frame=%0d cycles=%0d",f,cycles-start_cycle);
        end
        // Verify actual backing after all dirty values have drained.
        @(negedge clk);flush=1;wait(ff);
        for(integer addr_=0;addr_<16384;addr_++)
            if(dram.mem[8192+addr_/4][(addr_%4)*64+:53]!==expected[addr_])$fatal(1,"elevation backing write lost");
        $display("TEST PASSED frames=%0d points=%0d complete53bit_words=%0d backing_words=16384",frames,points,checks);$finish;
    end
    initial begin #(64'd10000000000);$fatal(1,"bounded elevation simulation budget");end
endmodule
