// Passive BMI088 register-0 observer. No SCL/SDA drivers or sensor control.
// Counts address ACK/NACK and completed first ID bytes at18/19/68/69.
// Counters wrap modulo2^32; a master NACK after one received byte is normal.
module i2c_id_observer(
 input wire clk,reset_n,scl,sda,
 output wire[127:0] address_ack_count,address_nack_count,id_read_count,
 output wire[31:0] last_ids,output reg[3:0] id_seen);
 reg[2:0] scl_sync,sda_sync;
 wire sc=scl_sync[1],sd=sda_sync[1];
 wire rising=!scl_sync[2] && sc;
 wire start=sda_sync[2] && !sd && sc;
 wire stop=!sda_sync[2] && sd && sc;
 reg active;reg[3:0] bit_index;
 reg[7:0] shift;
 reg[1:0] byte_index,target_index;
 reg target_known,reading,read_id;
 reg register_known;reg[6:0] register_target;reg[7:0] register_address;
 reg[31:0] acks[0:3],nacks[0:3],reads[0:3];reg[7:0] ids[0:3];
 wire known_address=shift[7:1]==7'h18 || shift[7:1]==7'h19 || shift[7:1]==7'h68 || shift[7:1]==7'h69;
 wire[1:0] decoded_index=shift[7:1]==7'h18 ? 0 : shift[7:1]==7'h19 ? 1 : shift[7:1]==7'h68 ? 2 : 3;
 genvar g;generate for(g=0;g<4;g=g+1)begin:outputs
  assign address_ack_count[g*32+:32]=acks[g];
  assign address_nack_count[g*32+:32]=nacks[g];
  assign id_read_count[g*32+:32]=reads[g];
  assign last_ids[g*8+:8]=ids[g];
 end endgenerate
 integer i;
 always@(posedge clk or negedge reset_n)begin
  if(!reset_n)begin
   scl_sync<=7;sda_sync<=7;active<=0;bit_index<=0;shift<=0;byte_index<=0;
   target_index<=0;target_known<=0;reading<=0;read_id<=0;
   register_known<=0;register_target<=0;register_address<=0;id_seen<=0;
   for(i=0;i<4;i=i+1)begin acks[i]<=0;nacks[i]<=0;reads[i]<=0;ids[i]<=0;end
  end else begin
   scl_sync<={scl_sync[1:0],scl};sda_sync<={sda_sync[1:0],sda};
   if(start)begin
    if(!active)register_known<=0;
    active<=1;bit_index<=0;shift<=0;byte_index<=0;read_id<=0;
   end else if(stop)begin
    active<=0;bit_index<=0;byte_index<=0;register_known<=0;read_id<=0;
   end else if(active && rising)begin
    if(bit_index<8)begin shift<={shift[6:0],sd};bit_index<=bit_index+1'b1;end
    else begin
     bit_index<=0;
     if(byte_index==0)begin
      reading<=shift[0];target_known<=known_address;target_index<=decoded_index;
      if(known_address)begin
       if(sd)nacks[decoded_index]<=nacks[decoded_index]+1'b1;
       else acks[decoded_index]<=acks[decoded_index]+1'b1;
      end
      read_id<=!sd && shift[0] && known_address && register_known &&
               register_target==shift[7:1] && register_address==0;
      if(!shift[0] || sd)register_known<=0;
      register_target<=shift[7:1];byte_index<=1;
     end else begin
      if(byte_index==1 && !reading)begin
       register_known<=target_known && !sd;register_address<=shift;
      end
      if(byte_index==1 && reading && read_id)begin
       reads[target_index]<=reads[target_index]+1'b1;ids[target_index]<=shift;id_seen[target_index]<=1;
      end
      if(byte_index<3)byte_index<=byte_index+1'b1;
     end
    end
   end
  end
 end
endmodule
