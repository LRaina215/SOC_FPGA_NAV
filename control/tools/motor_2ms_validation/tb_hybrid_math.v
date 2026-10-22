`timescale 1ns/1ps
module tb_hybrid_math;
reg clk=0;always #5 clk=~clk;reg rst=0;reg[31:0]period_input=1,count_input=0;reg neg=0;
wire signed[31:0]speed;wire signed[15:0]cms;wire valid;
motor520_hybrid_speed #(.CLK_HZ(1000000),.SAMPLE_CYCLES(2000),.STOP_CYCLES(50000)) dut(.clk(clk),.reset_n(rst),.position(32'sd0),.speed_mmps(speed),.speed_cms(cms),.sample_valid(valid));
integer i,want,want_cms;reg[63:0]n,d,q;
task check;begin
 force dut.effective_period=period_input;force dut.direction=neg;force dut.count_abs=count_input;force dut.window_delta=neg ? -$signed(count_input) : $signed(count_input);
 wait(valid);@(negedge clk);
 if(count_input>=4)begin n=count_input*64'd204204*1000000;d=1320*64'd1000*2000;end
 else begin n=204204*64'd1000000;d=1320*64'd1000*period_input;end
 q=(n+d/2)/d;if(q>32767)q=32767;want=neg ? -q : q;
 if(speed-want>1||speed-want< -1)$fatal(1,"math period=%0d count=%0d speed=%0d want=%0d",period_input,count_input,speed,want);
 want_cms=speed<0 ? -((-speed+5)/10) : (speed+5)/10;
 if(cms!=want_cms)$fatal(1,"cms sign/round speed=%0d cms=%0d",speed,cms);
 wait(!valid);
end endtask
initial begin
 force dut.age=0;force dut.usable_period=1;force dut.effective_period=period_input;
 force dut.count_abs=count_input;force dut.direction=neg;force dut.window_delta=neg ? -$signed(count_input) : $signed(count_input);
 repeat(5)@(negedge clk);rst=1;
 for(i=0;i<30;i=i+1)begin period_input=1+i*i*57;neg=i%2;check;end
 for(i=0;i<30;i=i+1)begin count_input=4+i*40;neg=i%2;check;end
 $display("PASS narrow M/T arithmetic:60 signed period/count cases, saturation,cm/s rounding;<=1mm/s error");$finish;
end
endmodule
