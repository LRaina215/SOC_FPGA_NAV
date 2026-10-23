// Temporary passive feedback diagnostic; no policy or actuator controller.
// Raw encoder mapping matches original main.v: M2 is left, M1 is right.
module feedback_probe_top #(
 parameter integer CLOCK_FREQ_HZ=50000000,FRAME_HZ=100,CLKS_PER_BIT=217
)(input wire clk,reset_n,M1_ENCA,M1_ENCB,M2_ENCA,M2_ENCB,
 inout wire BMI088_SDA,BMI088_SCL,output wire uart_txd,
 output wire M1_PWM,M1_DIR1,M1_DIR2,M2_PWM,M2_DIR1,M2_DIR2,
 output wire SERVO1,SERVO2,SERVO3,SERVO4);
 assign {M1_PWM,M1_DIR1,M1_DIR2,M2_PWM,M2_DIR1,M2_DIR2,SERVO1,SERVO2,SERVO3,SERVO4}=10'd0;
 reg[7:0]startup=0;
 always@(posedge clk)if(!reset_n)startup<=0;else if(!(&startup))startup<=startup+1'b1;
 wire ready_reset_n=reset_n && (&startup);
 wire init_done,init_error,sample_valid;wire[3:0]error_code;wire[5:0]operation;
 wire[6:0]accel_address,gyro_address;wire[7:0]accel_id,gyro_id,last_read;
 wire signed[15:0]ax,ay,az,gx,gy,gz;
 bmi088 sensor(.clk(clk),.reset_n(ready_reset_n),.IIC_SDA(BMI088_SDA),.IIC_SCL(BMI088_SCL),
  .init_done(init_done),.init_error(init_error),.data_valid(sample_valid),
  .accel_x(ax),.accel_y(ay),.accel_z(az),.gyro_x(gx),.gyro_y(gy),.gyro_z(gz),
  .error_code(error_code),.operation(operation),.accel_addr(accel_address),.gyro_addr(gyro_address),
  .accel_id(accel_id),.gyro_id(gyro_id),.last_read(last_read));
 wire signed[31:0]lp,rp,ld,rd;wire signed[15:0]lc,rc;wire lv,rv;
 wire[31:0]li,ri;
 motor520_speed #(.CLK_HZ(CLOCK_FREQ_HZ),.SPEED_GATE_CYCLES(CLOCK_FREQ_HZ/100),.ENCODER_SIGN(1)) left(
  .clk(clk),.reset_n(ready_reset_n),.enc_a(M2_ENCA),.enc_b(M2_ENCB),
  .position(lp),.speed_delta(ld),.speed_cms(lc),.sample_valid(lv),.invalid_transitions(li));
 motor520_speed #(.CLK_HZ(CLOCK_FREQ_HZ),.SPEED_GATE_CYCLES(CLOCK_FREQ_HZ/100),.ENCODER_SIGN(-1)) right(
  .clk(clk),.reset_n(ready_reset_n),.enc_a(M1_ENCA),.enc_b(M1_ENCB),
  .position(rp),.speed_delta(rd),.speed_cms(rc),.sample_valid(rv),.invalid_transitions(ri));
 wire[127:0]address_acks,address_nacks,id_reads;wire[31:0]last_ids;wire[3:0]id_seen;
 i2c_id_observer observer(clk,ready_reset_n,BMI088_SCL,BMI088_SDA,address_acks,address_nacks,id_reads,last_ids,id_seen);
 reg[31:0]bmi_sequence,encoder_sequence;
 reg[15:0]bmi_age_ms,encoder_age_ms;
 localparam integer MS_CLOCKS=CLOCK_FREQ_HZ/1000;
 reg[$clog2(MS_CLOCKS+1)-1:0]ms_div;
 wire tick_ms=ms_div==MS_CLOCKS-1;
 reg encoder_mismatch;
 reg signed[15:0]sample_ax,sample_ay,sample_az,sample_gx,sample_gy,sample_gz;
 reg signed[31:0]left_delta,right_delta,left_position,right_position;
 reg[31:0]left_invalid,right_invalid;
 always@(posedge clk or negedge ready_reset_n)begin
  if(!ready_reset_n)begin
   ms_div<=0;bmi_sequence<=0;encoder_sequence<=0;bmi_age_ms<=16'hffff;encoder_age_ms<=16'hffff;
   encoder_mismatch<=0;left_delta<=0;right_delta<=0;left_position<=0;right_position<=0;left_invalid<=0;right_invalid<=0;
   sample_ax<=0;sample_ay<=0;sample_az<=0;sample_gx<=0;sample_gy<=0;sample_gz<=0;
  end else begin
   ms_div<=tick_ms?0:ms_div+1'b1;
   if(tick_ms)begin
    if(bmi_age_ms!=16'hffff)bmi_age_ms<=bmi_age_ms+1'b1;
    if(encoder_age_ms!=16'hffff)encoder_age_ms<=encoder_age_ms+1'b1;
   end
   if(sample_valid)begin
    bmi_sequence<=bmi_sequence+1'b1;bmi_age_ms<=0;
    sample_ax<=ax;sample_ay<=ay;sample_az<=az;sample_gx<=gx;sample_gy<=gy;sample_gz<=gz;
   end
   if(lv!=rv)encoder_mismatch<=1;
   if(lv && rv)begin
    encoder_sequence<=encoder_sequence+1'b1;encoder_age_ms<=0;
    left_delta<=ld;right_delta<=rd;left_position<=lp;right_position<=rp;left_invalid<=li;right_invalid<=ri;
   end
  end
 end
 // Millisecond ages are quantized with <1ms phase ambiguity; no calibration
 // or fusion-ready flag is fabricated. Position snapshots are taken at the
 // original speed-publication edge, not at an asserted exact gate boundary.
 wire[959:0]payload={16'd0,16'hbc02,
  {6'd0,encoder_mismatch,(encoder_sequence!=0)},{4'd0,id_seen},last_ids,id_reads,address_nacks,address_acks,
  right_invalid,left_invalid,right_position,left_position,right_delta,left_delta,
  encoder_age_ms,encoder_sequence,sample_gz,sample_gy,sample_gx,sample_az,sample_ay,sample_ax,bmi_age_ms,bmi_sequence,
  last_read,gyro_id,accel_id,{1'b0,gyro_address},{1'b0,accel_address},{2'd0,operation},{4'd0,error_code},
  {4'd0,(encoder_sequence!=0),(bmi_sequence!=0),init_error,init_done},48'd0};
 uart_telemetry #(.VERSION(8'hb1),.PAYLOAD_BYTES(120),.CLOCK_FREQ_HZ(CLOCK_FREQ_HZ),
  .FRAME_HZ(FRAME_HZ),.CLKS_PER_BIT(CLKS_PER_BIT))transmitter(clk,ready_reset_n,payload,uart_txd);
endmodule
