`timescale 1ns/1ps
module tb_observation;
reg clk=0;always #10 clk=~clk;
reg rst_n=0,capture=0;
reg [143:0] last_actions=0;
reg signed [15:0] dl=10,dr=-20;
wire valid,initialized;wire [671:0] obs;wire [3359:0] hist;
rl_observation_v5 #(.LEFT_ENCODER_SIGN(-1),.RIGHT_ENCODER_SIGN(-1)) dut(
 .clk(clk),.rst_n(rst_n),.capture(capture),.angular_velocity_q12({-24'sd300,24'sd200,24'sd100}),
 .gravity_q12({-24'sd4096,24'd0,24'd0}),.left_delta(dl),.right_delta(dr),
 .linear_command_q12(24'sd410),.yaw_command_q12(-24'sd819),.height_command_q12(24'sd535),
 .last_executed_actions(last_actions),.valid(valid),.initialized(initialized),.current_flat(obs),.history_flat(hist));
reg [671:0] first;
integer i;
initial begin
 repeat(2)@(negedge clk);rst_n=1;
 @(negedge clk);capture=1;@(negedge clk);capture=0;
 wait(valid);@(negedge clk);
 if($signed(obs[18*24+:24])!=-2925 || $signed(obs[21*24+:24])!=5849)
  $fatal(1,"wheel scale/sign left=%0d right=%0d",$signed(obs[18*24+:24]),$signed(obs[21*24+:24]));
 if($signed(obs[0+:24])!=130 || $signed(obs[2*24+:24])!=-390 ||
    $signed(obs[5*24+:24])!=-12288 || $signed(obs[6*24+:24])!=820 ||
    $signed(obs[7*24+:24])!=-1065 || $signed(obs[8*24+:24])!=2675 || obs[9*24+:24]!=0)
  $fatal(1,"observation physical scales");
 for(i=0;i<5;i=i+1)if(hist[i*672+:672]!==obs)$fatal(1,"first history not repeated");
 first=obs;dl=-10;dr=20;
 last_actions={24'sd4096,-24'sd4096,24'sd2048,-24'sd2048,24'sd4096,-24'sd4096};
 capture=1;@(negedge clk);capture=0;
 wait(valid);@(negedge clk);
 for(i=0;i<4;i=i+1)if(hist[i*672+:672]!==first)$fatal(1,"history ordering");
 if(hist[4*672+:672]!==obs)$fatal(1,"newest frame not current");
 if($signed(obs[10*24+:24])!=-123 || $signed(obs[11*24+:24])!=123 || obs[12*24+:24]!=0 ||
    $signed(obs[13*24+:24])!=61 || $signed(obs[14*24+:24])!=-123 || obs[15*24+:24]!=0)
  $fatal(1,"leg previous action observations");
 if(obs[22*24+:144]!==last_actions)$fatal(1,"previous normalized actions");
 $display("PASS observation scales, raw encoder signs, previous actions and five-frame history order");$finish;
end
initial begin #1000000;$fatal(1,"timeout");end
endmodule
