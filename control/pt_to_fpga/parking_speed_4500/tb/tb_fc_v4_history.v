`timescale 1ns/1ps
module tb_fc_v4_history;
    reg clk=0; always #5 clk=~clk;
    reg rst_n=0,obs_wr_en=0,commit=0;
    reg [4:0] obs_wr_addr=0;
    reg signed [23:0] obs_wr_data=0;
    reg [7:0] rd_addr=0;
    wire signed [23:0] rd_data;
    wire rd_valid,busy,done,initialized;
    wire [3:0] error_flags;
    fc_v4_history_buffer dut(.clk(clk),.rst_n(rst_n),.obs_wr_en(obs_wr_en),.obs_wr_addr(obs_wr_addr),
        .obs_wr_data(obs_wr_data),.commit(commit),.rd_addr(rd_addr),.rd_data(rd_data),.rd_valid(rd_valid),
        .busy(busy),.done(done),.initialized(initialized),.error_flags(error_flags));
    integer frame,j,elapsed,expected_frame;
    task tick;
        begin @(posedge clk); #1; end
    endtask
    task write_obs;
        input integer index;
        input signed [23:0] value;
        begin
            @(negedge clk); obs_wr_en=1; obs_wr_addr=index; obs_wr_data=value;
            tick;
            @(negedge clk); obs_wr_en=0;
        end
    endtask
    task pulse_commit;
        begin @(negedge clk); commit=1; tick; @(negedge clk); commit=0; end
    endtask
    initial begin
        tick;
        if (busy || initialized || rd_valid) $fatal(1,"history reset failed");
        @(negedge clk); rst_n=1;
        pulse_commit;
        if (busy || !error_flags[3]) $fatal(1,"empty history observation accepted");
        write_obs(0,0);
        pulse_commit;
        if (busy) $fatal(1,"partial observation accepted");
        write_obs(28,0);
        if (!error_flags[2]) $fatal(1,"bad observation address not flagged");
        for (frame=0;frame<8;frame=frame+1) begin
            for (j=0;j<28;j=j+1) write_obs(j,frame*100+j-400);
            // Same-edge commit/write must reject the commit while retaining write.
            @(negedge clk); commit=1; obs_wr_en=1; obs_wr_addr=0; obs_wr_data=frame*100-400;
            tick;
            if (busy || !error_flags[1]) $fatal(1,"same-edge commit accepted");
            @(negedge clk); commit=0; obs_wr_en=0;
            pulse_commit;
            if (!busy || rd_valid) $fatal(1,"history commit failed");
            elapsed=0;
            while (!done) begin
                if (elapsed==7) begin
                    @(negedge clk); obs_wr_en=1; obs_wr_addr=0; obs_wr_data=24'h7fffff; commit=1;
                end
                tick; elapsed=elapsed+1;
                if (rd_valid) $fatal(1,"history exposed in-progress data");
                if (elapsed==8) begin
                    if (!error_flags[0] || !error_flags[1]) $fatal(1,"history busy rejection missing");
                    @(negedge clk); obs_wr_en=0; commit=0;
                end
                if (elapsed>300) $fatal(1,"history timeout");
            end
            if (elapsed!=280 || busy || !initialized || rd_valid) $fatal(1,"history completion boundary failed");
            for (j=0;j<140;j=j+1) begin
                @(negedge clk); rd_addr=j;
                tick;
                expected_frame=frame-4+j/28;
                if (expected_frame<0) expected_frame=0;
                if (!rd_valid || rd_data !== (expected_frame*100+j%28-400))
                    $fatal(1,"history mismatch frame=%0d address=%0d got=%0d expected=%0d",frame,j,rd_data,expected_frame*100+j%28-400);
            end
            // Valid is delayed with the returned word, not the new address.
            @(negedge clk); rd_addr=255;
            if (!rd_valid) $fatal(1,"read validity changed before read clock");
            tick;
            if (rd_valid || rd_data!==24'd0) $fatal(1,"invalid history read");
            pulse_commit;
            if (busy || !error_flags[3]) $fatal(1,"stale observation accepted");
            write_obs(0,frame*100-400);
            pulse_commit;
            if (busy) $fatal(1,"partial fresh observation accepted");
        end
        for (j=0;j<28;j=j+1) write_obs(j,j);
        pulse_commit;
        repeat(30) tick;
        @(negedge clk); rst_n=0;
        tick;
        if (busy || done || initialized || rd_valid || error_flags) $fatal(1,"mid-shift reset failed");
        @(negedge clk); rst_n=1;
        pulse_commit;
        if (busy || !error_flags[3]) $fatal(1,"reset did not clear history readiness");
        for(j=0;j<28;j=j+1) write_obs(j,700-j);
        pulse_commit;
        elapsed=0;
        while(!done) begin tick; elapsed=elapsed+1; if(elapsed>280) $fatal(1,"history recovery timeout"); end
        if(elapsed!=280 || !initialized || busy) $fatal(1,"history recovery failed");
        for(j=0;j<140;j=j+1) begin
            @(negedge clk); rd_addr=j; tick;
            if(!rd_valid || rd_data !== (700-j%28)) $fatal(1,"history reset recovery content %0d",j);
        end
        $display("PASS HISTORY init-repeat5 shift-newest8 sync-read busy/reset/recovery/freshness");
        $finish;
    end
    initial begin #100000; $fatal(1,"history timeout"); end
endmodule
