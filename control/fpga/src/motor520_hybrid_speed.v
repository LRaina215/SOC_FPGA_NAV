// Narrow M/T estimator: count Q12 multiply or period numerator/edge clocks.
// Parameters only are divided at elaboration. Runtime period division uses one
// small subtractor over two clocks/bit; publication remains exactly every2ms.
module motor520_hybrid_speed #(
 parameter integer CLK_HZ=50000000,SAMPLE_CYCLES=100000,
 parameter integer ENCODER_CPR=1320,WHEEL_DIAMETER_UM=65000,ENCODER_SIGN=1,
 parameter integer STOP_CYCLES=2500000,COUNT_THRESHOLD=4
)(input wire clk,reset_n,input wire signed[31:0] position,
 output reg signed[31:0] speed_mmps,output reg signed[15:0] speed_cms,
 output reg sample_valid);
 localparam [63:0] CIRC=(64'd1*WHEEL_DIAMETER_UM*3141593+500000)/1000000;
 localparam [63:0] PERIOD_NUM=(CIRC*CLK_HZ+ENCODER_CPR*500)/(ENCODER_CPR*64'd1000);
 localparam [63:0] COUNT_GAIN=(CIRC*CLK_HZ*4096+ENCODER_CPR*64'd500*SAMPLE_CYCLES)/(ENCODER_CPR*64'd1000*SAMPLE_CYCLES);
 localparam AB=$clog2(STOP_CYCLES+1),TB=$clog2(SAMPLE_CYCLES+1);
 localparam NB=$clog2(PERIOD_NUM+1),DW=(NB>AB ? NB : AB)+1;
 localparam [63:0] COUNT_CAP=(32767*64'd4096+COUNT_GAIN-1)/COUNT_GAIN;
 localparam CB=$clog2(COUNT_CAP+1),GB=$clog2(COUNT_GAIN+1),PB=GB+CB;
 localparam PUBLISH=2*DW+12;
 localparam IDLE=0,COUNT_MUL=1,COUNT_ROUND=2,DIV_START=3,DIV_SUB=4,DIV_SHIFT=5,BOUND=6,CMS_MUL=7,CMS_SAVE=8;
 reg signed[31:0] last_position,window_position;
 wire signed[31:0] edge_delta=position-last_position,window_delta=position-window_position;
 wire[31:0] count_abs=window_delta[31] ? -window_delta : window_delta;
 reg[AB-1:0] age,period;
 reg[TB-1:0] timer;
 reg have_edge,period_valid,direction,negative,pending;
 wire edge_now=edge_delta!=0;
 wire usable_period=period_valid && (!edge_now || edge_delta[31]==direction);
 wire[AB-1:0] effective_period=edge_now ? age+1'b1 : (age>period ? age : period);
 reg[3:0] state;
 reg[CB-1:0] captured_count;
 reg[PB-1:0] count_product;
 reg[DW-1:0] dividend,divisor,quotient,remainder;
 reg[DW+1:0] difference;
 reg[6:0] bits_left;
 reg[31:0] result;
 reg[15:0] bounded;
 reg[31:0] cms_product;
 reg signed[31:0] pending_mmps;
 reg signed[15:0] pending_cms;
 always @(posedge clk) begin
  sample_valid<=0;
  if(!reset_n) begin
   last_position<=0;window_position<=0;age<=STOP_CYCLES;period<=0;timer<=0;
   have_edge<=0;period_valid<=0;direction<=0;negative<=0;pending<=0;state<=IDLE;
   captured_count<=0;count_product<=0;dividend<=0;divisor<=1;quotient<=0;remainder<=0;
   difference<=0;bits_left<=0;result<=0;bounded<=0;cms_product<=0;
   pending_mmps<=0;pending_cms<=0;speed_mmps<=0;speed_cms<=0;
  end else begin
   last_position<=position;
   if(age<STOP_CYCLES)age<=age+1'b1;
   if(age>=STOP_CYCLES)begin have_edge<=0;period_valid<=0;end
   if(edge_now)begin
    period_valid<=have_edge && edge_delta[31]==direction && age<STOP_CYCLES;
    period<=age+1'b1;age<=0;have_edge<=1;direction<=edge_delta[31];
   end
   if(timer==SAMPLE_CYCLES-1)begin
    timer<=0;window_position<=position;pending<=1;
    if(count_abs>=COUNT_THRESHOLD)begin
     captured_count<=count_abs>COUNT_CAP ? COUNT_CAP[CB-1:0] : count_abs[CB-1:0];
     negative<=window_delta[31] ^ (ENCODER_SIGN<0);state<=COUNT_MUL;
    end else if(usable_period && age<STOP_CYCLES && effective_period!=0)begin
     // Rounded integer numerator differs <0.5mm/s even at1clock period.
     divisor<=effective_period;negative<=direction ^ (ENCODER_SIGN<0);state<=DIV_START;
    end else begin result<=0;negative<=0;state<=BOUND;end
   end else timer<=timer+1'b1;
   if(timer==PUBLISH && pending)begin
    speed_mmps<=pending_mmps;speed_cms<=pending_cms;sample_valid<=1;pending<=0;
   end
   case(state)
    COUNT_MUL:begin count_product<=captured_count*COUNT_GAIN[GB-1:0];state<=COUNT_ROUND;end
    COUNT_ROUND:begin result<=(count_product+2048)>>12;state<=BOUND;end
    DIV_START:begin dividend<=PERIOD_NUM+(divisor>>1);quotient<=0;remainder<=0;bits_left<=DW;state<=DIV_SUB;end
    DIV_SUB:begin difference<={1'b0,remainder,dividend[DW-1]}-{2'b0,divisor};state<=DIV_SHIFT;end
    DIV_SHIFT:begin
     dividend<={dividend[DW-2:0],1'b0};
     quotient<={quotient[DW-2:0],!difference[DW+1]};
     remainder<=difference[DW+1] ? {remainder[DW-2:0],dividend[DW-1]} : difference[DW-1:0];
     bits_left<=bits_left-1'b1;
     if(bits_left==1)begin result<={quotient[DW-2:0],!difference[DW+1]};state<=BOUND;end
     else state<=DIV_SUB;
    end
    BOUND:begin bounded<=result>32767 ? 16'd32767 : result[15:0];state<=CMS_MUL;end
    CMS_MUL:begin
     pending_mmps<=negative ? -$signed({16'd0,bounded}) : $signed({16'd0,bounded});
     // Exact floor((bounded+5)/10) for bounded<=32767, using16bit reciprocal.
     cms_product<=(bounded+16'd5)*16'd52429;state<=CMS_SAVE;
    end
    CMS_SAVE:begin pending_cms<=negative ? -$signed(cms_product[31:19]) : $signed(cms_product[31:19]);state<=IDLE;end
   endcase
  end
 end
endmodule
