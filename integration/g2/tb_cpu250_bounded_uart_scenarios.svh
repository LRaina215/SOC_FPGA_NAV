// Appended to the already proven real G2/Pico/UART fixture. External drive is
// UART/reset only. Every CPU/RAM/PC/bus observation below is read-only.
  reg [7:0] packet[0:1099];reg [7:0] prior_image[0:4135];
  integer phase=0,drain_returns=0,backlogged_returns=0,max_rx=0,frontier_entries=0;
  integer checkpoint_entries=0,tx_publications=0,map_ack_publications=0,interrupt_seen=0;
  integer step_entries=0,last_pc=0,pops=0,drain_limit=0;
  reg drain_active=0,frontier_active=0,loss_active=0,timeout_in_frontier=0;
  reg [31:0] drain_ret,frontier_ret,last_step_us=0,loss_rx=0,zero_commit_us=0;
  reg [31:0] tx_words_seen[0:511];integer tx_index=0;
  reg [31:0] last_tx_seq=0,previous_flags=0;
  wire [31:0] pc=dut.u_soc.u_cpu.reg_pc;
  function [7:0] ram_byte(input integer address);
    reg [31:0] word_value;
    begin word_value=ram_word(address);ram_byte=word_value>>((address&3)*8);end
  endfunction
  always @(posedge clk)if(monitoring)begin
    if(dut.u_nav.u_mbx.rx_pkts>max_rx)max_rx=dut.u_nav.u_mbx.rx_pkts;
    if(pc==DRAIN_ENTRY && last_pc!=DRAIN_ENTRY)begin
      if(drain_active)$fatal(1,"Unexpected nested receive drain");
      drain_active=1;drain_ret=dut.u_soc.u_cpu.cpuregs[1];
      drain_limit=dut.u_soc.u_cpu.cpuregs[10];pops=0;
      if(drain_limit!=1 && drain_limit!=4)$fatal(1,"Unexpected actual drain ABI budget %0d",drain_limit);
    end
    if(drain_active && dut.u_nav.p_MBX_RX_POP)pops=pops+1;
    if(drain_active && pc==drain_ret)begin
      if(pops>drain_limit)$fatal(1,"Actual drain exceeded budget limit=%0d pops=%0d",drain_limit,pops);
      drain_returns=drain_returns+1;
      if(phase==1 && drain_limit==4 && dut.u_nav.u_mbx.rx_pkts!=0)backlogged_returns=backlogged_returns+1;
      $display("REAL_DRAIN_RETURN us=%0d budget=%0d pops=%0d queued=%0d",dut.u_nav.time_us,drain_limit,pops,dut.u_nav.u_mbx.rx_pkts);
      drain_active=0;
    end
    if(pc==FRONTIER_ENTRY && last_pc!=FRONTIER_ENTRY)begin
      frontier_entries=frontier_entries+1;frontier_active=1;frontier_ret=dut.u_soc.u_cpu.cpuregs[1];
      $display("REAL_FRONTIER_ENTER us=%0d last_rx=%0d",dut.u_nav.time_us,last_rx);
    end
    if(frontier_active && pc==frontier_ret)begin
      $display("REAL_FRONTIER_RETURN us=%0d interrupted=%0d",dut.u_nav.time_us,ram_byte(INTERRUPTED_ADDRESS));frontier_active=0;
    end
    if(pc==CHECKPOINT_ENTRY && last_pc!=CHECKPOINT_ENTRY)checkpoint_entries=checkpoint_entries+1;
    if(pc==STEP_ENTRY && last_pc!=STEP_ENTRY)begin step_entries=step_entries+1;last_step_us=dut.u_nav.time_us;end
    if(loss_active && ram_byte(INTERRUPTED_ADDRESS)!=0)interrupt_seen=1;
    if(loss_active && timed_out && !previous_flags[16])begin
      timeout_in_frontier=frontier_active;
      $display("REAL_BUSY_TIMEOUT us=%0d age=%0d frontier_active=%0d pc=%h",dut.u_nav.time_us,dut.u_nav.time_us-last_rx,frontier_active,pc);
      if(dut.u_nav.time_us-last_rx<250000)$fatal(1,"Timeout before250ms");
      if(!frontier_active)$fatal(1,"Busy-loss prerequisite missing at actual timeout; do not report idle as busy pass");
    end
    if(loss_active && timed_out && locked && zero_commit_us==0 && dut.u_nav.rf_wr && dut.u_nav.a_q==16'h501c &&
       dut.u_nav.wd_q[23:16]==4 && dut.u_nav.wd_q[15:0]==0 && dut.u_nav.r_SAFE_CMD_V==0)begin
      zero_commit_us=dut.u_nav.time_us;loss_rx=last_rx;
      if(zero_commit_us-loss_rx<250000 || zero_commit_us-loss_rx>350000)$fatal(1,"Actual SAFE_CMD_W zero age outside250..350ms");
      $display("REAL_SAFE_ZERO_COMMIT us=%0d last_rx=%0d age=%0d",zero_commit_us,loss_rx,zero_commit_us-loss_rx);
    end
    if(dut.u_nav.p_MBX_TX_DATA)begin
      if(tx_index>=512)$fatal(1,"Observer TX buffer exceeded actual mailbox capacity");
      tx_words_seen[tx_index]=dut.u_nav.wd_q;tx_index=tx_index+1;
    end
    if(dut.u_nav.p_MBX_TX_GO)begin
      tx_publications=tx_publications+1;
      if(tx_words_seen[0][31:24]==8'h82)begin
        map_ack_publications=map_ack_publications+1;
        $display("REAL_CPU_MAP_ACK us=%0d next=%0d state=%0d",dut.u_nav.time_us,tx_words_seen[1],tx_words_seen[2][7:0]);
      end
      tx_index=0;
    end
    previous_flags=guard_flags;last_pc=pc;
  end

  task rd32(input [15:0] addr,output [31:0] value);
    reg [7:0] a,b,c,d,e;
    begin
      fork
        begin send("R");send(addr[15:8]);send(addr[7:0]);end
        begin recv(a);recv(b);recv(c);recv(d);recv(e);end
      join
      if(a!=="D")$fatal(1,"Raw UART read response framing %h",a);
      value={b,c,d,e};
    end
  endtask
  task header(input [7:0] kind);
    begin for(integer i=0;i<1100;i=i+1)packet[i]=0;packet[0]="N";packet[1]="V";packet[2]=1;packet[3]=kind;end
  endtask
  task put32(input integer offset,input [31:0] value);
    begin for(integer i=0;i<4;i=i+1)packet[offset+i]=value>>(8*i);end
  endtask
  task inject(input integer n);
    reg [31:0] value;
    begin
      for(integer j=0;j<(n+3)/4;j=j+1)begin
        value=0;for(integer i=0;i<4;i=i+1)if(j*4+i<n)value=value|({24'd0,packet[j*4+i]}<<(i*8));
        wr32(16'h1230,value);
      end
      wr32(16'h1234,n);wr32(16'h1238,1);
    end
  endtask
  task heartbeat;
    begin header(8'h20);inject(24);end
  endtask
  task map_ack(input integer expected_next,input integer expected_state);
    reg [31:0] seq0,seq1,len,w0,w1,w2,start;integer done;
    begin
      done=0;start=dut.u_nav.time_us;
      while(!done && dut.u_nav.time_us-start<150000)begin
        rd32(16'h1244,seq0);
        if(seq0!=last_tx_seq)begin
          rd32(16'h1224,len);wr32(16'h123c,0);rd32(16'h1240,w0);
          wr32(16'h123c,1);rd32(16'h1240,w1);wr32(16'h123c,2);rd32(16'h1240,w2);rd32(16'h1244,seq1);
          if(seq0==seq1 && len==12 && w0==32'h8201564e && w1==expected_next && w2[7:0]==expected_state)done=1;
        end
      end
      if(!done)$fatal(1,"UART map ACK missing/overwritten expected offset=%0d state=%0d",expected_next,expected_state);
      last_tx_seq=seq0;checks=checks+1;
      $display("REAL_UART_MAP_ACK seq=%0d next=%0d state=%0d",seq0,expected_next,expected_state);
    end
  endtask
  task wait_accepted(input integer target);
    reg [31:0] start;
    begin start=dut.u_nav.time_us;while(accepted<target && dut.u_nav.time_us-start<100000)repeat(500)@(negedge clk);
      if(accepted<target)$fatal(1,"CPU acceptance timeout");end
  endtask
  initial begin : bounded_stimulus
    reg [31:0] start,at,stat,before_accept;integer loops_before,step_target,sent_frontier;
    $readmemh("prior_image.hex",prior_image);
    #200;key_n=1;wait(!dut.rst);monitoring=1;
    start=dut.u_nav.time_us;while(dut.u_soc.status<3 && dut.u_nav.time_us-start<100000)repeat(500)@(negedge clk);
    if(dut.u_soc.status<3 || !locked || timed_out)$fatal(1,"Actual CPU initial lock/boot failed");
    before_accept=accepted;mode(0);wait_accepted(before_accept+1);if(locked)$fatal(1,"Fresh IDLE did not unlock");checks=checks+1;
    // Finite line-rate-feasible120-packet refill; no simulated mailbox writes.
    phase=1;loops_before=dut.u_soc.status;start=dut.u_nav.time_us;
    for(integer k=0;k<120;k=k+1)begin
      at=dut.u_nav.time_us;header(8'h43);inject(4);
      while(dut.u_nav.time_us-at<500)repeat(50)@(negedge clk);
    end
    if(backlogged_returns==0 || max_rx<4 || dut.u_soc.status==loops_before)$fatal(1,"Persistent refill/service prerequisite not observed");
    if(locked || timed_out)$fatal(1,"Healthy refill stream locked unexpectedly");checks=checks+1;
    $display("REAL_REFILL_SERVICE elapsed=%0d backlog_returns=%0d max_rx=%0d loops=%0d",dut.u_nav.time_us-start,backlogged_returns,max_rx,dut.u_soc.status-loops_before);
    phase=2;start=dut.u_nav.time_us;
    while(dut.u_nav.u_mbx.rx_pkts!=0 && dut.u_nav.time_us-start<150000)repeat(500)@(negedge clk);
    if(dut.u_nav.u_mbx.rx_pkts!=0)$fatal(1,"Finite stress tail failed to drain");
    before_accept=accepted;mode(0);wait_accepted(before_accept+1);
    // Legal host protocol upload,512B chunks, ACK after each, plus one duplicate
    // and one out-of-order offset. The host reads the real latest-TX shadow.
    header(8'h40);put32(4,4136);inject(8);map_ack(0,1);
    for(integer off=0;off<4136;off=off+512)begin
      integer n;n=4136-off;if(n>512)n=512;
      header(8'h41);put32(4,off);for(integer j=0;j<n;j=j+1)packet[8+j]=prior_image[off+j];inject(8+n);map_ack(off+n,1);
      if(off==0)begin
        // Duplicate the first complete chunk: offset0 cannot advance512.
        inject(8+n);map_ack(512,1);
        header(8'h41);put32(4,1024);packet[8]=8'h5a;inject(9);map_ack(512,1);
      end
    end
    header(8'h42);inject(4);map_ack(4136,2);checks=checks+1;
    // Set pose first; its real frame-jump handler clears the hardware map.
    before_accept=accepted;header(8'h12);put32(4,3000);put32(8,3000);inject(24);wait_accepted(before_accept+1);
    // Guard acceptance precedes the callback. Wait for the real drain return
    // before sending SWEEP, so a late frame-jump CLEAR cannot erase it.
    start=dut.u_nav.time_us;
    while(drain_active && dut.u_nav.time_us-start<100000)repeat(100)@(negedge clk);
    if(drain_active)$fatal(1,"SET_POSE handler failed to return before real sweep");
    wr32(16'h3004,1);start=dut.u_nav.time_us;stat=0;
    while(stat[31:16]==0 && dut.u_nav.time_us-start<100000)rd32(16'h3028,stat);
    if(stat[31:16]==0)$fatal(1,"Real hardware sweep did not produce a local map sequence");
    // Let the first real map copy finish, then observe a subsequent idle plan
    // entry. Send SEARCH20ms after that entry, so the next100ms tick starts
    // frontier with an already-aged real lease. No fake old arrival timestamp.
    step_target=step_entries+2;start=dut.u_nav.time_us;
    while(step_entries<step_target && dut.u_nav.time_us-start<300000)begin
      heartbeat();at=dut.u_nav.time_us;while(step_entries<step_target && dut.u_nav.time_us-at<50000)repeat(500)@(negedge clk);
    end
    if(step_entries<step_target)$fatal(1,"Actual idle planner tick prerequisite missing");
    at=last_step_us;while(dut.u_nav.time_us-at<20000)repeat(100)@(negedge clk);
    sent_frontier=frontier_entries;before_accept=accepted;mode(3);wait_accepted(before_accept+1);
    if(locked || timed_out)$fatal(1,"SEARCH rejected after legal input setup");
    loss_active=1;start=dut.u_nav.time_us;
    while(frontier_entries==sent_frontier && dut.u_nav.time_us-start<160000)repeat(100)@(negedge clk);
    if(frontier_entries==sent_frontier)$fatal(1,"Actual frontier PC never reached: busy prerequisite failed");
    // The wire now stays idle. Watch actual arrival-based guard state and the
    // real SAFE_CMD_W commit pulse; a preexisting zero register cannot pass.
    start=dut.u_nav.time_us;
    while(zero_commit_us==0 && dut.u_nav.time_us-start<350000)repeat(100)@(negedge clk);
    if(zero_commit_us==0 || !timeout_in_frontier)$fatal(1,"Missing real busy timeout/zero commit");
    start=dut.u_nav.time_us;
    while(interrupt_seen==0 && dut.u_nav.time_us-start<50000)repeat(100)@(negedge clk);
    if(interrupt_seen==0)$fatal(1,"Busy frontier cancellation flag not observed");checks=checks+1;
    // Recovery uses raw UART too: no-target heartbeat cannot heal a latch.
    loss_active=0;before_accept=accepted;heartbeat();wait_accepted(before_accept+1);
    if(!locked || !timed_out)$fatal(1,"Heartbeat incorrectly healed timeout");
    before_accept=accepted;mode(0);wait_accepted(before_accept+1);if(locked || timed_out)$fatal(1,"Fresh IDLE recovery failed");checks=checks+1;
    $display("BOUNDED_DRAIN_UART_PICO PREPARED_CASES_PASSED checks=%0d time_us=%0d writes=%0d backlog_returns=%0d frontier_entries=%0d checkpoint_entries=%0d zero_age_us=%0d map_ack_publications=%0d",checks,dut.u_nav.time_us,writes,backlogged_returns,frontier_entries,checkpoint_entries,zero_commit_us-loss_rx,map_ack_publications);
    $finish;
  end
  initial begin #1600000000;$fatal(1,"Finite1.6s real-time CPU/UART watchdog");end
  initial begin repeat(16)begin #100000000;$display("BOUNDED_UART_PROGRESS time_us=%0d loops=%0d phase=%0d PC=%h",dut.u_nav.time_us,dut.u_soc.status,phase,pc);end end
endmodule
