"""Combine the original elevation/scan pipeline with exclusive SM UPDATE."""
from pathlib import Path
import json
import shutil
from derive_ddr_matcher import derive_store,derive_row_engine,derive_fast_cache
from derive_ddr_shared import derive_store as share_store,derive_cache


def prepare(out,here,names,tb):
    row=Path('F:/nav_integration_g2_20261023/ddr_map_row_update_probe7')
    assert json.loads((row/'functional_result.json').read_text())['passed']
    for name in ['nav_ddr_row_stream.v','nav_ddr_struct_cache.v']:
        shutil.copyfile(here/name,out/name);names.append(name)
    for name,source in [
        ('nav_ddr_structure_store_full.v',share_store(derive_store((here/'nav_ddr_structure_store.v').read_text()))),
        ('nav_slf_row_update_full.v',derive_row_engine((here/'nav_slf_row_update.v').read_text())),
        ('nav_ddr_struct_cache_fast.v',derive_fast_cache((here/'nav_ddr_struct_cache.v').read_text())),
        ('nav_ddr_em_cache.v',derive_cache((here/'nav_ddr_map_cache.v').read_text()))]:
        (out/name).write_text(source);names.append(name)
    shutil.copyfile(row/'slf_lf_ce.v',out/'slf_lf_ce.v');names.append('slf_lf_ce.v')
    a="""    nav_ddr_elevation_store store(clk,ui_clk,rst,1'b1,mem_req,mem_we,mem_addr,mem_wdata,mem_ack,mem_reply,
        1'b0,,fault,cmd,cmd_en,addr,cmd_ready,wd,wm,wen,wend,wready,rd,rv,re);"""
    b="""    reg sm_req=0,sm_we=0,sm_update=0,records_end=0;
    reg[15:0]sm_addr=0;reg[17:0]sm_wdata=0;
    wire sm_ack,store_ready,store_busy,store_done,record_ready;wire[17:0]sm_q;
    integer sm_updates=0,sm_writes=0,em_overlaps=0;
    nav_ddr_structure_store_full store(
        .clk(clk),.ui_clk(ui_clk),.rst(rst),.calib_done(1'b1),
        .req(sm_req),.we(sm_we),.word_addr(sm_addr),.wdata(sm_wdata),.ack(sm_ack),.rdata(sm_q),
        .update_start(sm_update),.cx(32'd0),.cy(32'd0),.record_valid(1'b0),.record_addr(16'd0),
        .record_meta(8'd0),.records_end(records_end),.record_ready(record_ready),
        .ready(store_ready),.busy(store_busy),.done(store_done),.fault(fault),
        .cmd(cmd),.cmd_en(cmd_en),.addr(addr),.cmd_ready(cmd_ready),.wr_data(wd),.wr_mask(wm),
        .wr_en(wen),.wr_end(wend),.wr_ready(wready),.rd_data(rd),.rd_valid(rv),.rd_end(re),
        .clear_start(1'b0),.peek_req(1'b0),.peek_go(1'b0),.peek_addr(16'd0),.peek_hit(),.peek_q(),
        .em_req(mem_req),.em_we(mem_we),.em_addr(mem_addr),.em_wdata(mem_wdata),
        .em_ack(mem_ack),.em_reply(mem_reply),.em_flush(1'b0),.em_flushed());
    task automatic sm_write(input[15:0]a,input[17:0]d);
        begin
            @(negedge clk);sm_req=1;sm_we=1;sm_addr=a;sm_wdata=d;
            do begin @(posedge clk);#1;end while(!sm_ack);
            @(negedge clk);sm_req=0;
            do begin @(posedge clk);#1;end while(sm_ack);
            sm_writes++;
        end
    endtask
    initial begin:structure_worker
        integer previous_start;
        previous_start=-1;wait(!rst);wait(store_ready);
        for(integer f=0;f<n_frm;f++)begin
            wait(frame_start>=0&&frame_start!=previous_start);previous_start=frame_start;
            for(integer l=0;l<256;l++)
                sm_write((l/19)*256+(l%19)*14,18'h10000|l);
            // The live elevation pipeline must be in an operation when
            // the structure map requests its exclusive DMA interval.
            wait(!store.em_quiet);
            @(negedge clk);sm_update=1;
            @(negedge clk);sm_update=0;
            wait(record_ready);
            @(negedge clk);records_end=1;
            @(negedge clk);records_end=0;
            wait(store_done);sm_updates++;
        end
    end
    reg last_drain=0;
    always@(posedge clk)if(!rst)begin
        if(store.state==1&&!last_drain&&!store.em_quiet)em_overlaps++;
        last_drain<=store.state==1;
        if(store.row_owner&&(!store.em_quiet||store.arb_state!=0||store.native_ack))
            $fatal(1,"pipeline UPDATE began before EM drain");
    end"""
    assert tb.count(a)==1
    tb=tb.replace(a,b)
    a=tb.index('    initial begin:structure_worker')
    b=tb.index('    reg last_drain=0;',a)
    worker=tb[a:b]
    tb=tb[:a]+tb[b:]
    anchor='    int n_pix, n_flt, n_frm;'
    assert tb.count(anchor)==1
    tb=tb.replace(anchor,anchor+'\n'+worker)
    a='        if (errs == 0 && (n_flt<0||fi == n_flt)) $display("TEST PASSED frames='
    b='''        wait(sm_updates==n_frm);
        if(em_overlaps!=n_frm)$fatal(1,"pipeline overlap count");
        $display("SHARED_PIPELINE updates=%0d dirty_writes=%0d live_EM_overlaps=%0d",sm_updates,sm_writes,em_overlaps);
        if (errs == 0 && (n_flt<0||fi == n_flt)) $display("TEST PASSED frames='''
    assert tb.count(a)==1
    return tb.replace(a,b)
