 // One synchronous ROM read per edge, with independently changing held address.
 reg ram_done=0;
 reg scan_en=0;reg[15:0]scan_addr=0,scan_expected;wire signed[15:0]scan_old,scan_new;
 reg[15:0]golden_weights[0:40959];reg scan_seen=0;integer scan_reads=0,scan_holds=0;
 fixed4500_weight_rom_ref scan_ref(clk,scan_en,scan_addr,scan_old);
 fixed4500_weight_rom scan_dut(clk,scan_en,scan_addr,scan_new);
 always@(posedge clk)if(scan_en)begin scan_expected<=golden_weights[scan_addr];scan_seen<=1;end
 always@(negedge clk)if(scan_seen)begin
  if($isunknown({scan_old,scan_new,scan_expected}))$fatal(1,"unknown legal ROM value");
  if(scan_old!==scan_expected || scan_new!==scan_expected)$fatal(1,"weight/hold mismatch addr=%0d",scan_addr);
 end
 initial begin
  $readmemh("weights_expected.mem",golden_weights);
  repeat(5)@(negedge clk);
  for(integer k=0;k<40960;k++)begin
   #1;scan_en=1;scan_addr=k;scan_reads++;
   @(negedge clk);
   if(k%13==0)begin #1;scan_en=0;scan_addr=~k;scan_holds++;@(negedge clk);end
  end
  // The last partial trained bank and all256 zero-padded words were scanned.
  #1;scan_en=0;repeat(3)@(negedge clk);ram_done=1;
  $display("WEIGHT9 SCAN reads=%0d holds=%0d",scan_reads,scan_holds);
 end
