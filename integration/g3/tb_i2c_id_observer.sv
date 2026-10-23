`timescale 1ns/1ps
module tb_i2c_id_observer;
 reg clk=0;always #5 clk=~clk;
 reg reset_n=0,scl=1,sda=1;
 wire[127:0]acks,nacks,reads;wire[31:0]ids;wire[3:0]seen;
 i2c_id_observer dut(clk,reset_n,scl,sda,acks,nacks,reads,ids,seen);
 task idle;begin scl=1;sda=1;#200;end endtask
 task start;begin scl=0;#100;sda=1;#100;scl=1;#100;sda=0;#100;scl=0;#100;end endtask
 task stop;begin scl=0;sda=0;#100;scl=1;#100;sda=1;#100;end endtask
 task byte_on_bus(input[7:0]value,input bit nack);
  begin
   for(integer k=7;k>=0;k--)begin scl=0;sda=value[k];#100;scl=1;#100;scl=0;#100;end
   sda=nack;#100;scl=1;#100;scl=0;#100;
  end
 endtask
 task read_register(input[6:0]addr,input[7:0]regaddr,value,input bit address_nack,input bit master_nack);
  begin
   start();byte_on_bus({addr,1'b0},address_nack);
   if(!address_nack)begin byte_on_bus(regaddr,0);start();byte_on_bus({addr,1'b1},0);byte_on_bus(value,master_nack);end
   stop();idle();
  end
 endtask
 task check(input[127:0]a,n,r,input[31:0]v,input[3:0]s);
  begin #100;if({acks,nacks,reads,ids,seen}!=={a,n,r,v,s})
   $fatal(1,"observer mismatch a=%h n=%h r=%h ids=%h seen=%h",acks,nacks,reads,ids,seen);
  end
 endtask
 initial begin
  #30;reset_n=1;idle();check(0,0,0,0,0);
  read_register(7'h18,0,8'h1e,0,1);
  check({96'd0,32'd2},0,{96'd0,32'd1},32'h0000001e,1);
  read_register(7'h68,0,0,1,1);
  check({96'd0,32'd2},{32'd0,32'd1,64'd0},{96'd0,32'd1},32'h0000001e,1);
  // Successful bus transfer with wrong ID must be observable independently.
  read_register(7'h69,0,8'ha2,0,1);
  check({32'd2,64'd0,32'd2},{32'd0,32'd1,64'd0},{32'd1,64'd0,32'd1},32'ha200001e,9);
  read_register(7'h19,0,8'h1e,0,0);
  check({32'd2,32'd0,32'd2,32'd2},{32'd0,32'd1,64'd0},{32'd1,32'd0,32'd1,32'd1},32'ha2001e1e,11);
  // Non-ID register reads and other devices cannot fabricate ID counts.
  read_register(7'h68,8'h02,8'h0f,0,1);
  read_register(7'h50,0,8'h55,0,1);
  check({32'd2,32'd2,32'd2,32'd2},{32'd0,32'd1,64'd0},{32'd1,32'd0,32'd1,32'd1},32'ha2001e1e,11);
  // A STOP discards the saved register pointer; bare reads are not ID reads.
  start();byte_on_bus(8'hd1,0);byte_on_bus(8'h0f,1);stop();idle();
  check({32'd2,32'd3,32'd2,32'd2},{32'd0,32'd1,64'd0},{32'd1,32'd0,32'd1,32'd1},32'ha2001e1e,11);
  start();byte_on_bus(8'hd0,0);byte_on_bus(0,0);reset_n=0;#40;reset_n=1;stop();idle();
  check(0,0,0,0,0);
  read_register(7'h68,0,8'h0f,0,1);
  check({32'd0,32'd2,64'd0},0,{32'd0,32'd1,64'd0},32'h000f0000,4);
  $display("I2C_OBSERVER_PASS address ACK/NACK,wrong ID,register filtering,master NACK,STOP,reset");$finish;
 end
 initial begin #250000;$fatal(1,"timeout");end
endmodule
