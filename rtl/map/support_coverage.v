// 1.6m wheel-support observation layer:32x32 coarse cells,4x4 subcells each.
//9bits/subcell = {capture_epoch[6:0], distinct_epoch_count[1:0]}.
// Epoch200ms; fresh<=4ticks, retained<=99ticks (conservative upper ages1/20s).
// Exposed rolling strips and expired cells are invalidated; no hole filling.
module support_coverage (
 input wire clk,rst,clear,
 input wire [6:0] age_epoch,
 input wire frame_start,
 input wire signed [31:0] frame_cx,frame_cy,
 input wire [6:0] capture_epoch,
 input wire capture_valid,
 input wire s_valid,s_upd,
 output wire s_ready,
 input wire [13:0] s_addr, // {coarse_y[4:0],coarse_x[4:0],sub_y[1:0],sub_x[1:0]}
 input wire cpu_req,
 input wire [13:0] cpu_addr, // original128x128 coarse slot address
 output reg cpu_ack,
 output reg [31:0] cpu_q, // {fresh_mask16,known_mask16}, row-major subregions
 output wire busy
);
 localparam IDLE=0,CLR=1,ROLL=2,AREAD=3,AWRITE=4,QREAD=5,QACC=6;
 reg [2:0] state;
 reg [13:0] ptr;
 reg signed [31:0] cx,cy,oldcx,oldcy,nextcx,nextcy;
 reg pending_frame,cap_valid;
 reg [6:0] cap_epoch,nextcap,seen_epoch;
 reg nextvalid;
 reg cpu_seen;
 reg [9:0] query_base;
 reg [3:0] query_bit;
 reg [6:0] query_epoch;
 reg [15:0] known_mask,fresh_mask;

 wire acc=s_valid && s_ready && s_upd && cap_valid;
 reg v1,v2,fw1v,fw2v,fw3v;
 reg [13:0] a1,a2,fw1a,fw2a,fw3a;
 reg [6:0] ep1,ep2;
 reg [8:0] q2,fw1d,fw2d,fw3d;
 reg m1,m2,m3;
 wire [8:0] ram_q;
 wire [8:0] old=m1?fw1d:m2?fw2d:m3?fw3d:q2;
 wire [6:0] oldage=age_epoch-old[8:2];
 wire empty=(old[1:0]==0)||(oldage>99);
 wire [1:0] count=empty?2'd1:(old[8:2]==ep2 || old[1:0]==3)?old[1:0]:old[1:0]+2'd1;
 wire [8:0] updated={ep2,count};
 wire pipe_busy=v1|v2|fw1v;
 wire need_age=seen_epoch!=age_epoch;
 assign s_ready=state==IDLE && !pending_frame && !frame_start && !need_age && !cpu_req;
 assign busy=(state==CLR)||(state==ROLL)||(state==AREAD)||(state==AWRITE)||pending_frame||frame_start||need_age||pipe_busy;

 wire signed [31:0] roll_x=cx-32'sd16+$signed({1'b0,ptr[8:4]});
 wire signed [31:0] roll_y=cy-32'sd16+$signed({1'b0,ptr[13:9]});
 wire exposed=roll_x<oldcx-16 || roll_x>=oldcx+16 || roll_y<oldcy-16 || roll_y>=oldcy+16;
 wire [13:0] roll_addr={roll_y[4:0],roll_x[4:0],ptr[3:0]};
 wire [6:0] ram_age=age_epoch-ram_q[8:2];
 wire expired=ram_q[1:0]!=0 && ram_age>99;
 wire ram_we=(state==CLR)||((state==ROLL)&&exposed)||((state==AWRITE)&&expired)||fw1v;
 wire [13:0] ram_wa=(state==CLR)?ptr:(state==ROLL)?roll_addr:(state==AWRITE)?ptr:fw1a;
 wire [8:0] ram_wd=((state==CLR)||(state==ROLL)||(state==AWRITE))?9'd0:fw1d;
 wire ram_re=acc||(state==AREAD)||(state==QREAD);
 wire [13:0] ram_ra=(state==AREAD)?ptr:(state==QREAD)?{query_base,query_bit}:s_addr;
 nav_ram_sdp #(.DW(9),.AW(14)) u_ram(.clk(clk),.we(ram_we),.waddr(ram_wa),.wdata(ram_wd),.re(ram_re),.raddr(ram_ra),.rdata(ram_q));
 wire [6:0] qdx=cpu_addr[6:0]-cx[6:0]+7'd64;
 wire [6:0] qdy=cpu_addr[13:7]-cy[6:0]+7'd64;
 wire query_inside=qdx>=48 && qdx<80 && qdy>=48 && qdy<80;
 wire [6:0] query_age=query_epoch-ram_q[8:2];
 wire is_known=ram_q[1:0]>=2 && query_age<=99;
 wire is_fresh=is_known && query_age<=4;
 wire [15:0] bit_mask=16'h1<<query_bit;
 wire [15:0] next_known=known_mask|(is_known?bit_mask:16'd0);
 wire [15:0] next_fresh=fresh_mask|(is_fresh?bit_mask:16'd0);

 always @(posedge clk) begin
  cpu_ack<=0;
  if(rst || clear) begin
   state<=CLR;ptr<=0;cx<=0;cy<=0;oldcx<=0;oldcy<=0;
   pending_frame<=0;cap_valid<=0;cap_epoch<=0;seen_epoch<=age_epoch;
   cpu_seen<=0;cpu_q<=0;v1<=0;v2<=0;fw1v<=0;fw2v<=0;fw3v<=0;m1<=0;m2<=0;m3<=0;
  end else begin
   if(!cpu_req)cpu_seen<=0;
   if(frame_start) begin pending_frame<=1;nextcx<=frame_cx;nextcy<=frame_cy;nextcap<=capture_epoch;nextvalid<=capture_valid;end
   v1<=acc;a1<=s_addr;ep1<=cap_epoch;
   v2<=v1;a2<=a1;ep2<=ep1;q2<=ram_q;
   m1<=v2 && a2==a1;m2<=fw1v && fw1a==a1;m3<=fw2v && fw2a==a1;
   fw1v<=v2;fw1a<=a2;fw1d<=updated;
   fw2v<=fw1v;fw2a<=fw1a;fw2d<=fw1d;
   fw3v<=fw2v;fw3a<=fw2a;fw3d<=fw2d;
   case(state)
    CLR: if(ptr==16383)begin ptr<=0;state<=IDLE;end else ptr<=ptr+1'b1;
    ROLL:if(ptr==16383)begin ptr<=0;state<=IDLE;end else ptr<=ptr+1'b1;
    AREAD:state<=AWRITE;
    AWRITE:if(ptr==16383)begin ptr<=0;state<=IDLE;end else begin ptr<=ptr+1'b1;state<=AREAD;end
    QREAD:state<=QACC;
    QACC:begin
      known_mask<=next_known;fresh_mask<=next_fresh;
      if(query_bit==15)begin cpu_q<={next_fresh,next_known};cpu_ack<=1;cpu_seen<=1;state<=IDLE;end
      else begin query_bit<=query_bit+1'b1;state<=QREAD;end
    end
    default:begin
     if(pending_frame && !pipe_busy)begin
      cap_epoch<=nextcap;cap_valid<=nextvalid;pending_frame<=0;
      // Invalid capture metadata cannot roll away newer retained observations.
      if(nextvalid)begin
       oldcx<=cx;oldcy<=cy;cx<=nextcx;cy<=nextcy;
       if(cx!=nextcx || cy!=nextcy)begin ptr<=0;state<=ROLL;end
      end
     end else if(need_age && !pipe_busy)begin seen_epoch<=age_epoch;ptr<=0;state<=AREAD;end
     else if(cpu_req && !cpu_seen && !pipe_busy && !pending_frame && !frame_start)begin
      if(!query_inside)begin cpu_q<=0;cpu_ack<=1;cpu_seen<=1;end
      else begin query_base<={cpu_addr[11:7],cpu_addr[4:0]};query_bit<=0;query_epoch<=age_epoch;known_mask<=0;fresh_mask<=0;state<=QREAD;end
     end
    end
   endcase
  end
 end
endmodule
