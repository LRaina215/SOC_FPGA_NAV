`timescale 1ns/1ps
module gyro_sum_math(output reg done=0);
 reg signed[15:0] x=0,y=0,z=0;
 fusion_original reference(.clk(1'b0),.rst_n(1'b0),.sample_valid(1'b0),
  .accel_x(16'sd0),.accel_y(16'sd0),.accel_z(16'sd0),.gyro_x(x),.gyro_y(y),.gyro_z(z));
 imu_fusion candidate(.clk(1'b0),.rst_n(1'b0),.sample_valid(1'b0),
  .accel_x(16'sd0),.accel_y(16'sd0),.accel_z(16'sd0),.gyro_x(x),.gyro_y(y),.gyro_z(z));
 wire[31:0] old_sum=reference.abs16(x)+reference.abs16(y)+reference.abs16(z);
 function integer magnitude(input signed[15:0] v);
  integer n;begin n=v;magnitude=n<0 ? -n : n;end
 endfunction
 integer vectors=0,maximum=0,expected,u,v,w;
 initial begin
  for(integer axis=0;axis<3;axis++)for(integer context_id=0;context_id<4;context_id++)
   for(integer k=-32768;k<=32767;k++)begin
    case(context_id)
     0:begin v=0;w=0;end
     1:begin v=-32768;w=-32768;end
     2:begin v=32767;w=-32768;end
     3:begin v=(k*1103515245+12345)&65535;w=(k*1664525+1013904223)&65535;end
    endcase
    case(axis)
     0:begin x=k;y=v;z=w;end
     1:begin x=v;y=k;z=w;end
     2:begin x=v;y=w;z=k;end
    endcase
    #1;expected=magnitude(x)+magnitude(y)+magnitude(z);
    if(old_sum!==expected || candidate.gyro_abs_sum32!==expected)
     $fatal(1,"gyro absolute sum mismatch x=%0d y=%0d z=%0d",x,y,z);
    vectors++;if(expected>maximum)maximum=expected;
   end
  if(vectors!=786432 || maximum!=98304)$fatal(1,"gyro range coverage incomplete");
  done=1;$display("GYRO WIDTHS PASSED vectors=%0d max_sum=%0d",vectors,maximum);
 end
endmodule
