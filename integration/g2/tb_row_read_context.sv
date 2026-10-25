// Compare the unchanged complete extractor, including its real address/consumer pipeline.
`timescale 1ns/1ps
module tb;
reg clk=0; always #10 clk=~clk;
reg rst=1,en=1,fire=0,pv=0,sof=0,eof=0;
reg signed[31:0] x=0,y=0,z=0,zlo=-100000,zhi=100000,tx=0,ty=0;
wire ow,nw,od,nd,ob,nb; wire[11:0]oa,na; wire[35:0]oq,nq; wire[12:0]onp,nnp;
extract_old a(clk,rst,en,zlo,zhi,tx,ty,fire,x,y,z,pv,sof,eof,ow,oa,oq,od,onp,ob);
extract_new b(clk,rst,en,zlo,zhi,tx,ty,fire,x,y,z,pv,sof,eof,nw,na,nq,nd,nnp,nb);
integer cycles=0,writes=0,completions=0,frames=0,resets=0,gaps=0;
reg[31:0]rng=32'h18914500;
always @(negedge clk)begin
 cycles++;
 if({ow,od,ob,onp}!=={nw,nd,nb,nnp})$fatal(1,"control mismatch cycle=%0d",cycles);
 if(ow)begin
  if({oa,oq}!=={na,nq})$fatal(1,"point mismatch cycle=%0d",cycles);
  writes++;
 end
 if(od)completions++;
end
task idle(input integer n);
 begin repeat(n)begin @(negedge clk);#1;fire=0;sof=0;eof=0;gaps++;end end
endtask
task reset_now;
 begin @(negedge clk);#1;rst=1;fire=0;sof=0;eof=0;resets++;idle(4);rst=0;idle(4);end
endtask
task frame(input integer kind, input integer count);
 integer row,col,before_done;
 begin
  before_done=completions;
  for(integer i=0;i<count;i++)begin
   rng=rng^(rng<<13);rng=rng^(rng>>17);rng=rng^(rng<<5);
   if(kind==2 && rng[1:0]==0)idle(rng[3:2]+1);
   @(negedge clk);#1;row=i/160;col=i%160;
   fire=1;sof=(i==0);eof=(i==9599);pv=1;
   // Full 9600-point input saturates the original4096 point capacity.
   if(kind==0 || kind==1)begin x=0;y=0;z=(row&1)?100:0;end
   else if(kind==2)begin
    x=$signed(rng);y=$signed({rng[15:0],rng[31:16]});
    z=$signed(rng^32'h8090ffff);pv=(rng[7:4]!=0);
   end else begin
    x=-90000+col;y=70000-col;z=(row&1)?-999:999;pv=(col%11!=0);
   end
  end
  idle(220);
  if(count==9600 && en)begin
   if(completions!=before_done+1 || ob || nb)$fatal(1,"frame did not flush once");
   if(kind==0 && (onp!=4096 || nnp!=4096))$fatal(1,"capacity saturation not exercised");
  end
  if(!en && (completions!=before_done || ob || onp))$fatal(1,"disabled frame emitted");
  frames++;
 end
endtask
initial begin
 reset_now();frame(0,9600);
 en=0;frame(1,9600);en=1;
 zlo=-2147483647;zhi=2147483647;tx=-65537;ty=131071;frame(2,9600);
 // Reset interrupts a partially overwritten row store; the next frame must recover identically.
 frame(3,437);reset_now();zlo=-2000;zhi=2000;frame(3,9600);
 // A second no-reset frame checks the persisted row contents and frame boundary.
 frame(3,9600);
 if(writes<8192 || completions!=4 || resets!=2)$fatal(1,"insufficient context coverage");
 $display("TEST PASSED cycles=%0d valid_points=%0d completed_frames=%0d frame_attempts=%0d resets=%0d gaps=%0d saturation4096=1",cycles,writes,completions,frames,resets,gaps);
 $finish;
end
initial begin #3000000;$fatal(1,"finite context stimulus did not complete");end
endmodule
