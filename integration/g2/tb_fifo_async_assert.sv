`timescale 1ns/1ps
module tb_fifo_async_assert;
    reg rclk=0,wclk=0,rrun=1,wrun=1;
    integer rhalf=10,whalf=20;
    always begin #(rhalf);if(rrun)rclk=~rclk;end
    always begin #(whalf);if(wrun)wclk=~wclk;end
    reg arst=0;
    wire rst,wrst;
    nav_rst_sync core_reset(.clk(rclk),.arst(arst),.rst(rst));
    nav_rst_sync rx_reset(.clk(wclk),.arst(rst),.rst(wrst));
    reg s_valid=0,m_ready=0;
    reg [9:0] s_data=0;
    wire old_ready,new_ready,old_valid,new_valid;
    wire [9:0] old_data,new_data;
    wire [11:0] old_free,new_free;
    nav_async_fifo_original #(.DW(10),.AW(11)) original(
        .wclk(wclk),.wrst(wrst),.s_valid(s_valid),.s_data(s_data),.s_ready(old_ready),.wfree(old_free),
        .rclk(rclk),.rrst(rst),.m_valid(old_valid),.m_data(old_data),.m_ready(m_ready));
    nav_async_fifo #(.DW(10),.AW(11)) candidate(
        .wclk(wclk),.wrst(wrst),.s_valid(s_valid),.s_data(s_data),.s_ready(new_ready),.wfree(new_free),
        .rclk(rclk),.rrst(rst),.m_valid(new_valid),.m_data(new_data),.m_ready(m_ready));
    // phase1: cycle-exact normal original/candidate comparison and queue.
    // phase2: original's stale replay must remain visible; candidate must not.
    // phase3: candidate-only recovery, without a second reset of the stopped RX.
    integer phase=0,ready_mode=0,rcycle=0;
    integer wi=0,ri=0,normal_writes=0,normal_reads=0,port_checks=0;
    integer replay_old=0,replay_new=0,recovered=0,reset_checks=0;
    integer full_capacity=0,wrap_writes=0,readstop_cases=0,reset_phases=0;
    integer streak=0,max_streak=0,reset_write_attempts=0;
    reg [9:0] expected[0:32767];
    reg [9:0] replay_words[0:3];
    reg [9:0] memory_before;
    function [9:0] word(input integer n);
        word=(n*17)^10'h2a5;
    endfunction
    always @(negedge rclk) begin
        case(ready_mode)
          0:m_ready=0;
          1:m_ready=1;
          2:m_ready=(rcycle%11>=3)&&(rcycle%97<74);
          default:m_ready=0;
        endcase
    end
    always @(posedge wclk) begin
        if(wrst)begin
            if(new_ready!==0 || candidate.wr!==0)$fatal(1,"candidate writer active during reset");
            reset_checks=reset_checks+1;
            if(s_valid)reset_write_attempts=reset_write_attempts+1;
        end
        if(!wrst && (phase==1 || phase==3))begin
            if(phase==1)begin
                if(new_ready!==old_ready || new_free!==old_free)$fatal(1,"normal writer cycle mismatch");
                port_checks=port_checks+1;
            end
            if(s_valid && new_ready)begin
                if(wi>=32768)$fatal(1,"test queue capacity");
                expected[wi]=s_data;wi=wi+1;
                if(phase==1)normal_writes=normal_writes+1;
            end
        end
    end
    always @(posedge rclk) begin
        rcycle=rcycle+1;
        if(rst)begin
            if(new_valid!==0 || candidate.rd!==0 || candidate.u_ob.pend!==0 || candidate.u_ob.cnt!==0)
                $fatal(1,"candidate read/valid state active during reset");
            reset_checks=reset_checks+1;
        end
        if(!rst && phase==1)begin
            if(new_valid!==old_valid)$fatal(1,"normal valid cycle mismatch");
            if(new_valid && new_data!==old_data)$fatal(1,"normal registered read/data mismatch");
            port_checks=port_checks+1;
            if(new_valid && m_ready)begin
                if(ri>=wi || new_data!==expected[ri])$fatal(1,"normal ordering/read-first capture mismatch ri=%0d wi=%0d got=%h",ri,wi,new_data);
                ri=ri+1;normal_reads=normal_reads+1;streak=streak+1;
                if(streak>max_streak)max_streak=streak;
            end else streak=0;
        end
        if(!rst && phase==2)begin
            if(old_valid && m_ready)begin
                if(replay_old>=4 || old_data!==replay_words[replay_old])$fatal(1,"old counterexample changed");
                replay_old=replay_old+1;
            end
            if(new_valid && m_ready)replay_new=replay_new+1;
        end
        if(!rst && phase==3 && new_valid && m_ready)begin
            if(ri>=wi || new_data!==expected[ri])$fatal(1,"candidate fresh recovery mismatch");
            ri=ri+1;recovered=recovered+1;
        end
    end
    task empty_wait;
        begin
            wait(ri==wi);
            repeat(20)@(negedge rclk);
            repeat(10)@(negedge wclk);
            if(new_valid || candidate.u_ob.held!==0)$fatal(1,"candidate did not drain");
            if(phase==1 && (old_valid || original.u_ob.held!==0))$fatal(1,"original did not drain");
        end
    endtask
    task reset_common;
        begin
            phase=0;ready_mode=0;s_valid=0;
            #3;arst=1;#1;
            if(candidate.wp_b!==0 || candidate.rp_b!==0 || candidate.rp_b_w!==0 || candidate.wp_b_r!==0)
                $fatal(1,"async reset did not clear all pointer state");
            repeat(6)@(negedge rclk);repeat(6)@(negedge wclk);
            #3;arst=0;
            wait(!rst && !wrst);
            repeat(6)@(negedge rclk);repeat(6)@(negedge wclk);
            wi=0;ri=0;streak=0;
        end
    endtask
    task push_word(input [9:0] data);
        begin
            @(negedge wclk);s_valid=1;s_data=data;
            @(posedge wclk);
            while(!new_ready)@(posedge wclk);
        end
    endtask
    task stop_input;
        begin @(negedge wclk);s_valid=0;end
    endtask
    task seed_four;
        begin
            reset_common();phase=1;ready_mode=1;
            for(integer i=0;i<4;i=i+1)begin replay_words[i]=word(i+20);push_word(replay_words[i]);end
            stop_input();empty_wait();
            if(candidate.wp_b!=4 || candidate.rp_b!=4 || original.wp_b!=4 || original.rp_b!=4)
                $fatal(1,"nonzero empty pointer precondition");
        end
    endtask
    initial begin
        // Full/empty and FWFT two-word capacity.  This also demonstrates
        // sustained one-per-read-clock drain after initial prefetch latency.
        reset_common();phase=1;ready_mode=0;
        for(integer i=0;i<2300;i=i+1)begin
            @(negedge wclk);s_valid=1;s_data=word(wi);
        end
        stop_input();
        if(new_ready!==0 || old_ready!==0 || new_free!==0 || wi!=2050)$fatal(1,"full capacity mismatch words=%0d free=%0d",wi,new_free);
        full_capacity=wi;ready_mode=1;empty_wait();
        if(max_streak<2048)$fatal(1,"FWFT throughput regressed max=%0d",max_streak);
        $display("CASE full/read-first registered capture PASS words=%0d continuous_reads=%0d",wi,max_streak);

        // Faster producer, backpressure, source valid gaps and two complete
        // pointer wraps.  Valid/data remain stable while ready is deasserted.
        reset_common();whalf=5;rhalf=10;phase=1;ready_mode=2;
        for(integer i=0;i<10000;i=i+1)begin
            if(i%17==0)begin stop_input();repeat(2)@(negedge wclk);end
            push_word(word(i+500));
        end
        stop_input();ready_mode=1;empty_wait();
        if(wi!=10000 || ri!=10000 || candidate.wp_b!=(10000%4096) || original.wp_b!=candidate.wp_b)
            $fatal(1,"wrap/backpressure stream mismatch");
        wrap_writes=wi;$display("CASE wrap/backpressure PASS writes=%0d reads=%0d",wi,ri);

        // Preserve the original empty/nonzero pointer failure next to the
        // candidate.  Neither clock nor any internal state is forced.
        whalf=20;rhalf=10;seed_four();
        @(negedge wclk);#1;wrun=0;
        phase=0;#3;arst=1;#1;
        if(candidate.wp_b!==0 || candidate.rp_b!==0 || original.wp_b!==4)$fatal(1,"stopped writer assertion mismatch");
        repeat(8)@(negedge rclk);
        phase=2;ready_mode=1;#3;arst=0;
        repeat(30)@(negedge rclk);
        if(replay_old!=4 || replay_new!=0 || wrst!==1 || candidate.wp_b!==0 || candidate.rp_b!==0)
            $fatal(1,"stopped writer repair failed old=%0d new=%0d",replay_old,replay_new);
        $display("CASE stopped-write common reset PASS old_replay=%0d candidate_replay=%0d",replay_old,replay_new);
        // Candidate resumes fresh traffic after RX clock returns, without a
        // second common reset.  Old implementation is already out of epoch.
        wi=0;ri=0;phase=3;wrun=1;wait(!wrst);repeat(5)@(negedge wclk);
        for(integer i=0;i<24;i=i+1)push_word(word(i+800));
        stop_input();empty_wait();
        if(recovered!=24)$fatal(1,"candidate did not recover after stopped writer");
        $display("CASE candidate no-second-reset recovery PASS words=%0d",recovered);

        // Read/core clock stopped: the actual top's root reset also stays
        // asserted until the core clock returns.  Both endpoints stay closed.
        seed_four();@(negedge rclk);#1;rrun=0;phase=0;#3;arst=1;#1;
        if(candidate.rp_b!==0 || original.rp_b!==4)$fatal(1,"stopped reader async state mismatch");
        repeat(8)@(negedge wclk);#3;arst=0;repeat(8)@(negedge wclk);
        if(rst!==1 || wrst!==1 || new_valid!==0 || new_ready!==0)$fatal(1,"root reset released without read clock");
        rrun=1;wait(!rst && !wrst);repeat(8)@(negedge rclk);repeat(8)@(negedge wclk);
        wi=0;ri=0;phase=1;ready_mode=1;
        for(integer i=0;i<32;i=i+1)push_word(word(i+900));
        stop_input();empty_wait();readstop_cases=readstop_cases+1;
        $display("CASE stopped-read common reset and recovery PASS");

        // Interrupt all four legal output-buffer validity/occupancy shapes.
        // Payload registers and RAM are intentionally retained, validity is not.
        for(integer p=0;p<4;p=p+1)begin
            reset_common();whalf=5;rhalf=10;phase=1;ready_mode=0;
            fork
                begin
                    for(integer j=0;j<(p==1?1:8);j=j+1)push_word(word(j+1100+p*10));
                    stop_input();
                    // Do not let the producer winning join_any cancel the
                    // reset watcher before the selected buffer shape occurs.
                    wait(arst);
                end
                begin
                    case(p)
                        0:wait(candidate.u_ob.cnt==0 && candidate.u_ob.pend==1);
                        1:wait(candidate.u_ob.cnt==1 && candidate.u_ob.pend==0);
                        2:wait(candidate.u_ob.cnt==1 && candidate.u_ob.pend==1);
                        3:wait(candidate.u_ob.cnt==2 && candidate.u_ob.pend==0);
                    endcase
                    #2;phase=0;memory_before=candidate.mem[0];arst=1;
                end
            join_any
            disable fork;
            // Keep attempting writes while reset is held, checking that even
            // the retained RAM does not change as the clocks continue.
            s_valid=1;s_data=10'h3e1;#1;
            if(candidate.u_ob.pend!==0 || candidate.u_ob.cnt!==0 || new_valid!==0 || candidate.rd!==0)
                $fatal(1,"FWFT async validity did not clear phase=%0d",p);
            repeat(8)@(negedge wclk);
            if(candidate.mem[0]!==memory_before)$fatal(1,"candidate wrote or cleared RAM during reset");
            s_valid=0;repeat(8)@(negedge rclk);#3;arst=0;
            wait(!rst && !wrst);repeat(8)@(negedge rclk);repeat(8)@(negedge wclk);
            wi=0;ri=0;phase=1;ready_mode=1;
            for(integer j=0;j<32;j=j+1)push_word(word(j+1400+p*40));
            stop_input();empty_wait();reset_phases=reset_phases+1;
        end
        if(reset_phases!=4 || reset_write_attempts<16)$fatal(1,"reset boundary coverage missing");
        $display("FIFO ASYNC ASSERT TEST PASSED normal_writes=%0d normal_reads=%0d port_cycle_checks=%0d full_capacity_words=%0d continuous_read_streak=%0d wrap_writes=%0d old_stopped_write_replays=%0d candidate_stopped_write_replays=%0d candidate_recovery_words=%0d stopped_read_cases=%0d middle_reset_phases=%0d reset_checks=%0d reset_write_attempts=%0d",
            normal_writes,normal_reads,port_checks,full_capacity,max_streak,wrap_writes,replay_old,replay_new,recovered,readstop_cases,reset_phases,reset_checks,reset_write_attempts);
        $finish;
    end
    initial begin #5000000;$fatal(1,"finite FIFO contract watchdog");end
endmodule
