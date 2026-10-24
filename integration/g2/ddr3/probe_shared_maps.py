"""Shared native-port additions to the full original matcher proof.

An independent elevation client continuously changes full53-bit words while
the original matcher clears, scores, loads prior maps and updates. All native
commands use the same controller. Each UPDATE deliberately overlaps a dirty
elevation eviction so a writeback-to-read-fill gap cannot be mistaken for an
idle client. This is a transport/ownership proof, not a physical DDR bound.
"""
from derive_ddr_shared import derive_cache, derive_store, derive_matcher


def prepare(out, here, names, tb):
    (out/'nav_ddr_structure_store_full.v').write_text(
        derive_store((out/'nav_ddr_structure_store_full.v').read_text()))
    (out/'slf_match_ddr.v').write_text(
        derive_matcher((out/'slf_match_ddr.v').read_text()))
    (out/'nav_ddr_em_cache.v').write_text(
        derive_cache((here/'nav_ddr_map_cache.v').read_text()))
    names.append('nav_ddr_em_cache.v')
    model=(out/'row_service_model.sv').read_text()
    for a,b in [('[0:8191]','[0:16383]'),('[12:0]held_addr','[13:0]held_addr'),
                ('addr[27:16]','addr[27:17]'),('addr[15:3]','addr[16:3]')]:
        assert model.count(a)==1,a
        model=model.replace(a,b)
    (out/'row_service_model.sv').write_text(model)
    def one(a,b):
        nonlocal tb
        assert tb.count(a)==1,a[:100]
        tb=tb.replace(a,b)
    one('for(integer a=0;a<8192;a++)dram.mem[a]',
        'for(integer a=0;a<16384;a++)dram.mem[a]')
    one('    wire fault,pw_ready;', '''    reg em_req=0,em_we=0,em_flush=0;
    reg[13:0]em_addr=0;reg[52:0]em_wdata=0;
    wire em_ack,em_flushed,sm_initialized;wire[52:0]em_reply;
    reg[52:0]em_expected[0:16383];
    reg stop_em=0,em_stopped=0;
    integer em_ops=0,em_reads=0,em_writes=0,em_overlaps=0,em_backing=0;
    reg last_drain=0;
    wire fault,pw_ready;''')
    one('.ddr_rd_valid(rv),.ddr_rd_end(re));', '''.ddr_rd_valid(rv),.ddr_rd_end(re),
        .em_req(em_req),.em_we(em_we),.em_addr(em_addr),.em_wdata(em_wdata),
        .em_ack(em_ack),.em_reply(em_reply),.em_flush(em_flush),.em_flushed(em_flushed),
        .sm_initialized(sm_initialized));''')
    one('    integer started_update,', '''    task automatic em_access(input bit w,input[13:0]a,input[52:0]d);
        integer waited;
        begin
            @(negedge clk);em_req=1;em_we=w;em_addr=a;em_wdata=d;waited=0;
            do begin @(posedge clk);#1;waited++;
                if(waited>2000000)$fatal(1,"elevation client timeout");end while(!em_ack);
            if(w)begin em_expected[a]=d;em_writes++;end
            else begin
                if(em_reply!==em_expected[a])$fatal(1,"shared EM word mismatch addr=%0d got=%h expected=%h",a,em_reply,em_expected[a]);
                em_reads++;
            end
            em_ops++;@(negedge clk);em_req=0;
            do begin @(posedge clk);#1;end while(em_ack);
        end
    endtask
    initial begin:em_worker
        integer i;reg[52:0]pattern;
        for(i=0;i<16384;i++)em_expected[i]='1;
        wait(!rst);repeat(600)@(negedge clk);i=0;
        while(!stop_em)begin
            pattern=53'h1a123456789abc^{39'd0,i[13:0]}^{i[13:0],39'd0};
            em_access(1,i[13:0],pattern);
            em_access(0,i[13:0]^14'h0400,0);
            i=(i+1)%16384;
        end
        em_stopped=1;
    end
    always@(posedge clk)if(!rst)begin
        if(dut.u_store.state==1&&!last_drain&&!dut.u_store.held_clear&&!dut.u_store.em_quiet)em_overlaps++;
        last_drain<=dut.u_store.state==1;
        if(dut.u_store.row_owner&&(!dut.u_store.em_quiet||dut.u_store.arb_state!=0||dut.u_store.native_ack))
            $fatal(1,"row ownership granted before full EM operation drained");
    end
    always@(posedge ui_clk)if(!rst)begin
        if(de&&cr)begin
            if(da[2:0]!=0||!((da[27:3]<4864)||(da[27:3]>=8192&&da[27:3]<12288)))
                $fatal(1,"native map address overlap/outside capacity: %h",da);
        end
        if(dut.u_store.row_owner&&(dut.u_store.ce||dut.u_store.cwe))
            $fatal(1,"native cache command active during row ownership");
    end
    initial begin:cold_backing_check
        wait(!rst);wait(sm_initialized);
        for(integer a=0;a<65536;a++)
            if(dram.mem[(a/256)*19+(a%256)/14][((a%256)%14)*18+:18]!==18'd0)
                $fatal(1,"structure map cold initialization mismatch addr=%0d",a);
        $display("COLD_INIT_PROOF structure_words=65536");
    end
    integer started_update,''')
    one('            @(posedge clk);started_update=timing_cycle;cmd_update<=1;', '''            // Deliberately enter UPDATE with an active dirty EM eviction.
            // The read fill following this writeback must finish before DMA.
            wait(em_req&&dut.u_store.u_em.state==5);
            @(posedge clk);started_update=timing_cycle;cmd_update<=1;''')
    one('        if (errs == 0) $display("TEST PASSED frames=', '''        stop_em=1;wait(em_stopped);
        @(negedge clk);em_flush=1;wait(em_flushed);
        @(negedge clk);em_flush=0;wait(!em_flushed);
        // The model commits admitted writes in order. This comparison is a
        // functional model check, not a PHY write-completion timing proof.
        repeat(10)@(posedge clk);
        for(integer a=0;a<16384;a++)begin
            if(dram.mem[8192+a/4][(a%4)*64+:53]!==em_expected[a])
                $fatal(1,"shared EM backing mismatch addr=%0d",a);
            em_backing++;
        end
        if(em_overlaps!=n_frm)$fatal(1,"missing full dirty EM eviction overlap got=%0d expected=%0d",em_overlaps,n_frm);
        $display("SHARED_PROOF em_ops=%0d em_reads=%0d em_writes=%0d update_overlaps=%0d backing_words=%0d",em_ops,em_reads,em_writes,em_overlaps,em_backing);
        if (errs == 0) $display("TEST PASSED frames=''')
    return tb
