`timescale 1ns/1ps
module tb_actual_c;
 reg clk=0; always #10 clk=~clk;
 parameter integer RESET_HOLD_NS=200;
 reg rst_n=0;
 wire memory_clk,pll_lock,pll_stop,ui,ddr_rst,cal,cr,wr,rv,re;
 reg [2:0]cmd=0; reg ce=0,we=0; reg[28:0]addr=0;
 reg[255:0]wd=0; wire[255:0]rd;
 wire[14:0]da; wire[2:0]ba; wire cs,ras,cas,wen,ck,ckn,cke,odt,resetn;
 wire[3:0]dm; wire[31:0]dq; wire[3:0]dqs,dqsn;
 Nav_DDR3_PLL pll(.lock(pll_lock),.clkout0(),.clkout1(),.clkout2(memory_clk),.clkin(clk),.reset(~rst_n),.enclk0(1'b1),.enclk1(1'b1),.enclk2(pll_stop));
 Nav_DDR3_PHY dut(.clk(clk),.memory_clk(memory_clk),.pll_lock(pll_lock),.rst_n(rst_n),.pll_stop(pll_stop),.clk_out(ui),.ddr_rst(ddr_rst),.init_calib_complete(cal),
 .cmd_ready(cr),.cmd(cmd),.cmd_en(ce),.addr(addr),.wr_data_rdy(wr),.wr_data(wd),.wr_data_en(we),.wr_data_end(we),.wr_data_mask(32'b0),
 .rd_data(rd),.rd_data_valid(rv),.rd_data_end(re),.sr_req(1'b0),.ref_req(1'b0),.sr_ack(),.ref_ack(),.burst(1'b0),
 .O_ddr_addr(da),.O_ddr_ba(ba),.O_ddr_cs_n(cs),.O_ddr_ras_n(ras),.O_ddr_cas_n(cas),.O_ddr_we_n(wen),.O_ddr_clk(ck),.O_ddr_clk_n(ckn),.O_ddr_cke(cke),.O_ddr_odt(odt),.O_ddr_reset_n(resetn),.O_ddr_dqm(dm),.IO_ddr_dq(dq),.IO_ddr_dqs(dqs),.IO_ddr_dqs_n(dqsn));
 genvar g;generate for(g=0;g<2;g=g+1)begin:mem
 ddr3_model #(.DEBUG(0),.STOP_ON_ERROR(1)) ram(.rst_n(resetn),.ck(ck),.ck_n(ckn),.cke(cke),.cs_n(cs),.ras_n(ras),.cas_n(cas),.we_n(wen),.dm_tdqs(dm[g*2+:2]),.ba(ba),.addr(da),.dq(dq[g*16+:16]),.dqs(dqs[g*2+:2]),.dqs_n(dqsn[g*2+:2]),.tdqs_n(),.odt(odt));
 end endgenerate
 realtime mem_last=0,ui_last=0,mem_period=0,ui_period=0;
 always@(posedge memory_clk)begin mem_period=$realtime-mem_last;mem_last=$realtime;end
 always@(posedge ui)begin ui_period=$realtime-ui_last;ui_last=$realtime;end
 integer cyc=0,refreshes=0,activates=0,reads=0,writes=0;
 reg write_leveling=0;
 // Observe public DRAM pins and native phase; no forced training state.
 always@(resetn) $display("DDR_RESET_PIN time_us=%0.6f value=%b cke=%b",$realtime/1000.0,resetn,cke);
 always@(cal) $display("CALIBRATION_PIN time_us=%0.6f value=%b reads=%0d writes=%0d",$realtime/1000.0,cal,reads,writes);
`ifdef OBSERVE_VENDOR_PUBLIC_CALIBRATION
 // These are published .vo wires, not objects inside the protected DQS.
 wire[7:0]eye_cor0=dut.\gw3_top/u_ddr_phy_top/u_eye_calib_rmove_mod/eye_calib_gen[0].cor_cnt ;
 wire[7:0]eye_cor1=dut.\gw3_top/u_ddr_phy_top/u_eye_calib_rmove_mod/eye_calib_gen[1].cor_cnt ;
 wire[7:0]eye_cor2=dut.\gw3_top/u_ddr_phy_top/u_eye_calib_rmove_mod/eye_calib_gen[2].cor_cnt ;
 wire[7:0]eye_cor3=dut.\gw3_top/u_ddr_phy_top/u_eye_calib_rmove_mod/eye_calib_gen[3].cor_cnt ;
 wire[31:0]write_step=dut.\gw3_top/u_ddr_phy_top/wstep_Z ;
 initial forever begin #100000;
   $display("PUBLIC_CAL time_us=%0.6f eye_correct=%h/%h/%h/%h write_step=%h",$realtime/1000.0,eye_cor0,eye_cor1,eye_cor2,eye_cor3,write_step);
 end
`endif
 always@(posedge ui)cyc=cyc+1;
 always@(posedge ck)if(resetn && cke && !cs)case({ras,cas,wen})
 3'b000:begin
   if(ba==1)write_leveling=da[7];
   $display("DRAM_MRS time_us=%0.6f bank=%0d addr=%h write_leveling=%b cal=%b",$realtime/1000.0,ba,da,write_leveling,cal);
 end
 3'b001:refreshes=refreshes+1;3'b011:activates=activates+1;3'b100:writes=writes+1;3'b101:reads=reads+1;endcase
 initial begin #(RESET_HOLD_NS);rst_n=1;end
 initial begin repeat(40) begin #100000; $display("PROGRESS time=%0t lock=%b stop=%b reset=%b cal=%b ui=%0d ref=%0d act=%0d rd=%0d wr=%0d memory_period_ns=%0.3f ui_period_ns=%0.3f",$time,pll_lock,pll_stop,ddr_rst,cal,cyc,refreshes,activates,reads,writes,mem_period,ui_period);end $fatal(1,"calibration/probe watchdog");end
 task write_one(input[28:0]a,input[255:0]v);
 reg ca,wa; integer start;begin
 ca=0;wa=0;start=cyc;@(negedge ui);cmd=0;addr=a;wd=v;ce=1;we=1;
 while(!ca || !wa) begin @(posedge ui); if(ce&&cr)ca=1;if(we&&wr)wa=1;@(negedge ui);ce=!ca;we=!wa;if(cyc-start>10000)$fatal(1,"write timeout");end
 $display("WRITE ack %h latency_ui=%0d",a,cyc-start);end endtask
 task read_one(input[28:0]a,input[255:0]v);
 integer start;begin
 start=cyc;@(negedge ui);cmd=1;addr=a;ce=1;do @(posedge ui);while(!cr);@(negedge ui);ce=0;
 while(!rv)begin @(posedge ui);if(cyc-start>10000)$fatal(1,"read timeout");end
 if(rd !==v)$fatal(1,"read mismatch addr=%h expected=%h actual=%h",a,v,rd);
 $display("READ PASS %h latency_ui=%0d",a,cyc-start);@(negedge ui);end endtask
 reg[255:0]v; integer i;
 initial begin
 wait(cal===1'b1);$display("CALIBRATED at %0t ui=%0d",$time,cyc);
 if(mem_period<2.499 || mem_period>2.501 || ui_period<9.999 || ui_period>10.001)
   $fatal(1,"clock periods mismatch memory=%0.3f UI=%0.3f",mem_period,ui_period);
 $display("MODEL GEOMETRY row=%0d col=%0d dq=%0d RFC=%0d",mem[0].ram.ROW_BITS,mem[0].ram.COL_BITS,mem[0].ram.DQ_BITS,mem[0].ram.TRFC_MIN);
 if(mem[0].ram.ROW_BITS!=15 || mem[0].ram.COL_BITS!=10 || mem[0].ram.DQ_BITS!=16 || mem[0].ram.TRFC_MIN!=260000)
   $fatal(1,"DRAM geometry/refresh model mismatch");
 for(i=0;i<16;i=i+1) begin v={8{32'h12340000+i}};write_one(i*8,v);end
 for(i=15;i>=0;i=i-1) begin v={8{32'h12340000+i}};read_one(i*8,v);end
 #20000;
 $display("ACTUAL C PHY PROBE PASSED refs=%0d acts=%0d reads=%0d writes=%0d",refreshes,activates,reads,writes);$finish;
 end
endmodule
