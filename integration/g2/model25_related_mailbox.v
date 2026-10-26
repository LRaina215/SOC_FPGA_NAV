// Bundled-data transaction between explicitly related 50/25MHz clocks.
// Keep both clocks related and all paths timed. No async-clock exceptions.
// Source payload stays fixed from request until the returned acknowledgement.
module g2_model25_mailbox #(parameter WIDTH=1)(
 input wire src_clk,src_rst_n,src_valid,input wire[WIDTH-1:0]src_data,
 output wire src_ready,output reg src_overflow,
 input wire dst_clk,dst_rst_n,output reg dst_valid,output reg[WIDTH-1:0]dst_data
);
 reg[WIDTH-1:0]holding;
 reg request,acknowledge;
 (* syn_preserve=1 *) reg ack1,ack2,req1,req2;
 assign src_ready=request==ack2;
 always @(posedge src_clk or negedge src_rst_n)begin
  if(!src_rst_n)begin request<=0;ack1<=0;ack2<=0;holding<=0;src_overflow<=0;end
  else begin
   ack1<=acknowledge;ack2<=ack1;
   if(src_valid)begin
    if(src_ready)begin holding<=src_data;request<=~request;end
    else src_overflow<=1;
   end
  end
 end
 always @(posedge dst_clk or negedge dst_rst_n)begin
  if(!dst_rst_n)begin req1<=0;req2<=0;acknowledge<=0;dst_valid<=0;dst_data<=0;end
  else begin
   req1<=request;req2<=req1;dst_valid<=0;
   if(req2!=acknowledge)begin dst_data<=holding;dst_valid<=1;acknowledge<=req2;end
  end
 end
endmodule

// Latest-value source: intermediate continuously-held command states may coalesce.
// Every committed word is atomic; the source is retransmitted after any change.
module g2_model25_latest #(parameter WIDTH=1)(
 input wire src_clk,src_rst_n,input wire[WIDTH-1:0]src_data,
 input wire dst_clk,dst_rst_n,output wire dst_valid,output wire[WIDTH-1:0]dst_data
);
 reg sent;reg[WIDTH-1:0]last_sent;
 wire ready,unused_overflow;
 wire send=ready&&(!sent||src_data!=last_sent);
 always @(posedge src_clk or negedge src_rst_n)begin
  if(!src_rst_n)begin sent<=0;last_sent<=0;end
  else if(send)begin sent<=1;last_sent<=src_data;end
 end
 g2_model25_mailbox #(.WIDTH(WIDTH)) transfer(
  .src_clk(src_clk),.src_rst_n(src_rst_n),.src_valid(send),.src_data(src_data),
  .src_ready(ready),.src_overflow(unused_overflow),.dst_clk(dst_clk),.dst_rst_n(dst_rst_n),
  .dst_valid(dst_valid),.dst_data(dst_data));
endmodule

// Supervision runs from surviving50MHz. Failure is sticky until physical reset.
// Asynchronous assertion; two-edge release separately in each actual domain.
module g2_model25_guard #(parameter MAX_SILENT50=16)(
 input wire clk50,clk25,reset_n,pll_lock,transport_fault,
 output wire io_rst_n,model_rst_n,output wire fault
);
 reg lock1,lock2,seen_lock,hb25,hb1,hb2,last_hb;
 reg clocked_fault;
 // A lock-loss pulse between50MHz sampling edges must still latch. This is
 // an asynchronous preset, not a generated/gated clock. Clear only on a50MHz
 // edge while the original physical reset is asserted.
 reg lock_fault_async=1'b0;
 wire asynchronous_lock_loss=seen_lock&&!pll_lock;
 assign fault=clocked_fault|lock_fault_async;
 always @(posedge clk50 or posedge asynchronous_lock_loss)
  if(asynchronous_lock_loss)lock_fault_async<=1'b1;
  else if(!reset_n)lock_fault_async<=1'b0;
 reg[5:0]age;
 wire common_arst=!reset_n||!pll_lock||fault;
 reg[1:0]release50,release25;
 assign io_rst_n=release50[1];assign model_rst_n=release25[1];
 always @(posedge clk50 or posedge common_arst)
  if(common_arst)release50<=0;else release50<={release50[0],1'b1};
 always @(posedge clk25 or posedge common_arst)
  if(common_arst)release25<=0;else release25<={release25[0],1'b1};
 always @(posedge clk25 or negedge reset_n)
  if(!reset_n)hb25<=0;else hb25<=~hb25;
 always @(posedge clk50 or negedge reset_n)begin
  if(!reset_n)begin lock1<=0;lock2<=0;seen_lock<=0;hb1<=0;hb2<=0;last_hb<=0;age<=0;clocked_fault<=0;end
  else begin
   lock1<=pll_lock;lock2<=lock1;hb1<=hb25;hb2<=hb1;last_hb<=hb2;
   if(lock2)seen_lock<=1;
   if(hb2!=last_hb)age<=0;else if(age<MAX_SILENT50)age<=age+1'b1;
   if(transport_fault||(seen_lock&&(!lock2||age>=MAX_SILENT50-1)))clocked_fault<=1;
  end
 end
endmodule
