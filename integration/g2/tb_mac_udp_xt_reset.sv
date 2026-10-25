`timescale 1ns/1ps
// Finite actual-source network integration test. No CPU/control replacement is
// represented as a full G2 test: this bench ends at the real XT pixel output.
module tb_mac_udp_xt_reset;
  reg clk=0,rxc=0,rx_run=1,arst=0;
  always #10 clk=~clk;
  always #20 if(rx_run)rxc=~rxc;
  wire rst,rxrst;
  nav_rst_sync root_reset(.clk(clk),.arst(arst),.rst(rst));
  nav_rst_sync rx_reset(.clk(rxc),.arst(rst),.rst(rxrst));
  reg [3:0] rxd=0;reg ctl=0;
  wire rv,rl,re,txv,txl,txready;wire [7:0] rd,txd;
  rgmii_mac100 mac(.clk(clk),.rst(rst),.rx_rst(rxrst),
    .rgmii_rxc(rxc),.rgmii_rx_ctl(ctl),.rgmii_rxd(rxd),.rx_neg(1'b1),.mdio_i(1'b1),
    .rx_valid(rv),.rx_data(rd),.rx_last(rl),.rx_error(re),
    .tx_valid(txv),.tx_data(txd),.tx_last(txl),.tx_ready(txready));
  wire uv,ul,ue;wire [7:0] ud;wire [15:0] ulen;
  reg [31:0] ticks=0;always @(posedge clk)ticks<=ticks+1;
  wire [31:0] time_us=ticks/50;
  net_port #(.RX_AW(11),.TX_AW(11),.PL_AW(11),.RAW(1)) port0(
    .clk(clk),.rst(rst),.time_us(time_us),
    .rx_mac_clk(rxc),.rx_mac_rst(rxrst),.rx_mac_valid(rv),.rx_mac_data(rd),.rx_mac_last(rl),.rx_mac_error(re),
    .tx_mac_clk(clk),.tx_mac_rst(rst),.tx_mac_valid(txv),.tx_mac_data(txd),.tx_mac_last(txl),.tx_mac_ready(txready),
    .cfg_mac(48'h024e56000001),.cfg_ip(32'hc0a80064),.cfg_port(16'd7687),
    .cfg_enable(1'b1),.cfg_any_ip(1'b0),.cfg_arp_en(1'b1),.cfg_mcast_ip(32'd0),.cfg_learn(1'b1),
    .peer_we(1'b0),.peer_mac_w(48'd0),.peer_ip_w(32'd0),.peer_port_w(16'd0),
    .u_valid(uv),.u_data(ud),.u_last(ul),.u_err(ue),.u_len(ulen),
    .tx_go(1'b0),.tx_len(16'd0),.pl_q(8'd0),
    .raw_tx_we(1'b0),.raw_tx_waddr(5'd0),.raw_tx_wdata(32'd0),.raw_tx_go(1'b0),.raw_tx_len(8'd0),
    .raw_rx_en(1'b1),.raw_rx_raddr(5'd0),.raw_rx_pop(1'b0));
  wire pv,ps,pe;wire [7:0] pu,pvrow;wire [15:0] depth;
  wire ready=(ticks[2:0]!=0 && ticks[2:0]!=3);
  wire [15:0] sn,w,h;wire [31:0] good,bad;
  xt_frame_rx #(.CLK_HZ(50000000)) xt(
    .clk(clk),.rst(rst),.time_us(time_us),.cfg_enable(1'b1),.cfg_hdr_le(1'b1),.cfg_pix_le(1'b1),
    .cfg_subhdr(8'd20),.cfg_amp_min(16'd0),.cfg_raw_inv(16'd64000),
    .cfg_wait_pose(1'b0),.pose_commit(1'b0),.cfg_retain(1'b0),
    .s_valid(uv),.s_data(ud),.s_last(ul),.s_err(ue),
    .m_valid(pv),.m_ready(ready),.m_u(pu),.m_v(pvrow),.m_depth(depth),.m_sof(ps),.m_eof(pe),
    .frm_sn(sn),.frm_w(w),.frm_h(h),.stat_frames(good),.stat_bad(bad));
  reg [7:0] wire_bytes[0:100000];
  reg [63:0] packets[0:63]; // offset32 | frame length16 | UDP payload length16
  reg [31:0] groups[0:3]; // first packet16 | packet count16
  reg [33:0] pixels[0:38399];
  integer byte_checks=0,packet_checks=0,pixel_checks=0,frame_checks=0,reset_checks=0,stalls=0;
  integer udp_base=0,udp_len=0,udp_index=0,pixel_base=0,pixel_index=0,expected_sn=0;
  reg monitor_udp=0,expect_udp_error=0,monitor_pixel=0,reject=0;
  reg held=0;reg [33:0] held_pixel;
  always @(posedge clk)begin
    if(!rst)begin
      if(held && !pv)$fatal(1,"XT dropped valid while a pixel was stalled");
      if(reject && (uv||pv))$fatal(1,"post-reset tail escaped UDP or XT");
      if(uv && monitor_udp)begin
        if(udp_index>=udp_len || ud!==wire_bytes[udp_base+udp_index] || ul!==(udp_index==udp_len-1) ||
           ue!==((udp_index==udp_len-1)&&expect_udp_error) || ulen!==udp_len)
          $fatal(1,"UDP data/length/last/error mismatch index=%0d",udp_index);
        udp_index=udp_index+1;byte_checks=byte_checks+1;
      end
      if(pv)begin
        if(!monitor_pixel)$fatal(1,"unexpected XT publication");
        if(held && {ps,pe,pu,pvrow,depth}!==held_pixel)$fatal(1,"XT changed stalled pixel");
        held=!ready;held_pixel={ps,pe,pu,pvrow,depth};
        if(!ready)stalls=stalls+1;
        else begin
          if(pixel_index>=9600 || {ps,pe,pu,pvrow,depth}!==pixels[pixel_base+pixel_index] ||
             sn!==expected_sn || w!==160 || h!==60)$fatal(1,"XT pixel/metadata mismatch frame=%0d idx=%0d",expected_sn,pixel_index);
          pixel_index=pixel_index+1;pixel_checks=pixel_checks+1;
          if(pe)frame_checks=frame_checks+1;
        end
      end else held=0;
    end else held=0;
  end
  task nibble(input [3:0] d,input bit dv);
    begin @(posedge rxc);#2;rxd=d;ctl=dv;end
  endtask
  task idle(input integer n);
    begin repeat(n)nibble(0,0);end
  endtask
  task preamble;
    begin repeat(15)nibble(5,1);nibble(13,1);end
  endtask
  task packet(input integer index,input bit fcs_bad);
    integer off,n;reg [7:0] v;
    begin
      off=packets[index][63:32];n=packets[index][31:16];
      udp_base=off+42;udp_len=packets[index][15:0];udp_index=0;expect_udp_error=fcs_bad;monitor_udp=1;
      preamble();
      for(integer j=0;j<n;j=j+1)begin
        v=wire_bytes[off+j];if(fcs_bad && j==n-1)v=v^1;
        nibble(v[3:0],1);nibble(v[7:4],1);
      end
      idle(24);repeat(16)@(negedge clk);
      if(udp_index!=udp_len)$fatal(1,"UDP truncated got=%0d want=%0d",udp_index,udp_len);
      monitor_udp=0;packet_checks=packet_checks+1;
    end
  endtask
  task frame(input integer k,input bit corrupt);
    integer first,n;
    begin
      first=groups[k][31:16];n=groups[k][15:0];
      expected_sn=k+1;pixel_base=k*9600;pixel_index=0;monitor_pixel=!corrupt;
      for(integer j=0;j<n;j=j+1)packet(first+j,corrupt&&j==n-1);
      repeat(20000)@(negedge clk);
      if(!corrupt && pixel_index!=9600)$fatal(1,"missing XT pixels %0d",pixel_index);
      if(corrupt && (pixel_index!=0 || bad==0))$fatal(1,"bad FCS frame not rejected");
      monitor_pixel=0;
    end
  endtask
  task interrupted(input integer index,input bit stopped);
    integer off;reg [7:0] v;
    begin
      off=packets[index][63:32];monitor_udp=0;monitor_pixel=0;
      preamble();
      for(integer j=0;j<80;j=j+1)begin v=wire_bytes[off+j];nibble(v[3:0],1);nibble(v[7:4],1);end
      @(negedge rxc);#3;rx_run=!stopped;arst=1;
      repeat(8)@(negedge clk);#3;arst=0;
      repeat(24)@(negedge clk);
      if(stopped && rxrst!==1)$fatal(1,"RX reset released with clock stopped");
      rx_run=1;wait(!rst && !rxrst);#1;reject=1;
      // Still DV-high: header-shaped tail must never count as a new frame.
      preamble();
      for(integer j=0;j<120;j=j+1)begin v=wire_bytes[off+j];nibble(v[3:0],1);nibble(v[7:4],1);end
      idle(24);repeat(32)@(negedge clk);reject=0;reset_checks=reset_checks+1;
    end
  endtask
  initial begin
    $readmemh("wire.hex",wire_bytes);$readmemh("packets.hex",packets);
    $readmemh("groups.hex",groups);$readmemh("pixels.hex",pixels);
    #1;arst=1;repeat(8)@(negedge clk);#3;arst=0;wait(!rst&&!rxrst);idle(24);
    frame(0,0);interrupted(groups[1][31:16],0);frame(1,0);
    interrupted(groups[2][31:16],1);frame(2,0);
    frame(3,1);frame(3,0);
    if(frame_checks!=4 || pixel_checks!=38400 || reset_checks!=2 || stalls==0)$fatal(1,"coverage incomplete");
    $display("MAC UDP XT RESET PASSED frames=%0d pixels=%0d packets=%0d UDP_bytes=%0d resets=%0d stalls=%0d",frame_checks,pixel_checks,packet_checks,byte_checks,reset_checks,stalls);
    $finish;
  end
  initial begin #15000000;$fatal(1,"finite network stimulus watchdog");end
endmodule
