// Exact symmetric nearest conversion: signed mm/s or mrad/s -> signed Q12.
// Every input is accepted. Four stages, valid/sequence aligned; kill flushes.
// |x|*4096/1000 nearest == floor((|x|*512+62)/125).
// 67109*125=2^23+17; reciprocal error <4*17/125<1 for n<2^25.
// The quotient is exact or one too high; comparison with q*125 corrects it.
// The 25x17 multiply fits a 27x18 DSP. No combinational division is inferred.
module nav_command_units(input wire clk,reset_n,kill,valid,
 input wire [15:0] seq,input wire signed [15:0] vx,wz,
 output reg out_valid,output reg [15:0] out_seq,
 output reg signed [23:0] out_vx_q12,out_wz_q12);
 wire [15:0] abs_v=vx[15]?(~vx+16'd1):vx;
 wire [15:0] abs_w=wz[15]?(~wz+16'd1):wz;
 reg v0,v1,v2;reg [15:0] seq0,seq1,seq2;
 reg sv0,sw0,sv1,sw1,sv2,sw2;
 reg [24:0] nv0,nw0,nv1,nw1,nv2,nw2;
 reg [41:0] pv1,pw1;
 reg [18:0] qv2,qw2;
 reg [25:0] scaled_v2,scaled_w2;
 wire [18:0] approx_v=pv1[41:23],approx_w=pw1[41:23];
 wire [25:0] av={7'd0,approx_v},aw={7'd0,approx_w};
 wire [18:0] qv=qv2-(scaled_v2>{1'b0,nv2});
 wire [18:0] qw=qw2-(scaled_w2>{1'b0,nw2});
 always @(posedge clk or negedge reset_n)begin
  if(!reset_n)begin
   v0<=0;v1<=0;v2<=0;out_valid<=0;
   seq0<=0;seq1<=0;seq2<=0;out_seq<=0;
   sv0<=0;sw0<=0;sv1<=0;sw1<=0;sv2<=0;sw2<=0;
   nv0<=0;nw0<=0;nv1<=0;nw1<=0;nv2<=0;nw2<=0;
   pv1<=0;pw1<=0;qv2<=0;qw2<=0;scaled_v2<=0;scaled_w2<=0;
   out_vx_q12<=0;out_wz_q12<=0;
  end else if(kill)begin
   v0<=0;v1<=0;v2<=0;out_valid<=0;out_seq<=0;
   out_vx_q12<=0;out_wz_q12<=0;
  end else begin
   v0<=valid;v1<=v0;v2<=v1;out_valid<=v2;
   if(valid)begin
    nv0<={abs_v,9'b0}+25'd62;nw0<={abs_w,9'b0}+25'd62;
    sv0<=vx[15];sw0<=wz[15];seq0<=seq;
   end
   if(v0)begin
    pv1<=nv0*17'd67109;pw1<=nw0*17'd67109;
    nv1<=nv0;nw1<=nw0;sv1<=sv0;sw1<=sw0;seq1<=seq0;
   end
   if(v1)begin
    qv2<=approx_v;qw2<=approx_w;
    scaled_v2<=(av<<7)-(av<<1)-av;scaled_w2<=(aw<<7)-(aw<<1)-aw;
    nv2<=nv1;nw2<=nw1;sv2<=sv1;sw2<=sw1;seq2<=seq1;
   end
   if(v2)begin
    out_vx_q12<=sv2?-$signed({5'd0,qv}):$signed({5'd0,qv});
    out_wz_q12<=sw2?-$signed({5'd0,qw}):$signed({5'd0,qw});out_seq<=seq2;
   end
  end
 end
endmodule
