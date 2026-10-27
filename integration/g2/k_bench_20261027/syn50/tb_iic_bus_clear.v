`timescale 1ns/1ps
// A slave left mid-byte holds SDA low until it sees 5 more SCL rising edges.
// BUS_CLEAR=1: the first transaction reports error 5 after clocking the bus free, and
// the next one gets past START (error 1, address NACK, since no slave answers).
// BUS_CLEAR=0: SDA stays low and every transaction reports error 5.
module tb_iic_bus_clear;
 parameter CLEAR=1;
 reg clk=0;always #5 clk=~clk;
 reg reset_n=0,start=0;
 wire scl,sda;
 pullup(scl);pullup(sda);
 reg slave_low=1;integer edges=0;
 assign sda=slave_low?1'b0:1'bz;
 always @(posedge scl)if(slave_low)begin edges=edges+1;if(edges==5)slave_low=0;end
 wire busy,done,error;wire [7:0] rd;wire [63:0] rb;wire [3:0] code;
 iic #(.QUARTER_CYCLES(4),.TIMEOUT_CYCLES(200),.READ_BYTES(1),.BUS_CLEAR(CLEAR)) dut(
  .clk(clk),.reset_n(reset_n),.start(start),.device_addr(7'h19),.register_addr(8'h00),.write_data(8'h00),
  .IIC_SCL(scl),.IIC_SDA(sda),.busy(busy),.done(done),.error(error),.read_data(rd),.read_bytes(rb),.error_code(code));
 integer first_code,second_code;
 task run(output integer c);
  begin
   @(negedge clk);start=1;@(negedge clk);start=0;
   @(posedge done);@(negedge clk);c=code;
   repeat(50)@(negedge clk);
  end
 endtask
 initial begin
  repeat(5)@(negedge clk);reset_n=1;repeat(5)@(negedge clk);
  run(first_code);run(second_code);
  $display("BUS_CLEAR=%0d first error %0d, second error %0d, slave released %0d, scl edges seen %0d",
           CLEAR,first_code,second_code,!slave_low,edges);
  if(CLEAR && !(first_code==5 && second_code==1 && !slave_low))$fatal(1,"bus clear did not free the bus");
  if(!CLEAR && !(first_code==5 && second_code==5 && slave_low))$fatal(1,"unexpected recovery without bus clear");
  $display("TEST PASSED");$finish;
 end
 initial begin #2000000;$fatal(1,"timeout");end
endmodule
