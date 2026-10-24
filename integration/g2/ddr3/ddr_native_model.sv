// Behavioral app-interface model, NOT a model of training or the DDR PHY.
// Independently stalled command/data channels; byte masks and ordered BL8.
module ddr_native_model #(parameter HALF_PERIOD=5) (
    input wire clk, rst, enable,
    input wire [2:0] cmd,
    input wire cmd_en,
    input wire [27:0] addr,
    output wire cmd_ready,
    input wire [255:0] wr_data,
    input wire [31:0] wr_mask,
    input wire wr_en, wr_end,
    output wire wr_ready,
    output reg [255:0] rd_data,
    output reg rd_valid, rd_end
);
    reg [255:0] mem[0:4095];
    reg [31:0] rng=32'h17c45000;
    reg command_pending=0, data_pending=0;
    reg [2:0] held_cmd;
    reg [11:0] held_addr;
    reg [255:0] held_data;
    reg [31:0] held_mask;
    integer delay_count=0;
    integer reads=0, writes=0, command_stalls=0, data_stalls=0;
    integer data_first=0, command_first=0;
    assign cmd_ready=enable && !command_pending && (rng[1:0]!=0);
    assign wr_ready=enable && !data_pending && (rng[5:3]==0 || rng[9:8]==1);
    initial for(integer i=0;i<4096;i++) mem[i]='x;
    always @(posedge clk) begin
        rng<=rng^(rng<<13)^(rng>>17)^(rng<<5);
        rd_valid<=0;rd_end<=0;
        if(rst)begin command_pending<=0;data_pending<=0;delay_count<=0;end
        else begin
            if(cmd_en&&!cmd_ready)command_stalls++;
            if(wr_en&&!wr_ready)data_stalls++;
            if(cmd_en&&cmd_ready)begin
                if(addr[2:0]!=0 || addr[27:15]!=0) $fatal(1,"bad native address %h",addr);
                if(cmd!=0&&cmd!=1)$fatal(1,"bad command");
                command_pending<=1;held_cmd<=cmd;held_addr<=addr[14:3];delay_count<=2+rng[7:6];
                if(cmd==0&&!data_pending)command_first++;
            end
            if(wr_en&&wr_ready)begin
                if(!wr_end)$fatal(1,"single beat write missing end");
                data_pending<=1;held_data<=wr_data;held_mask<=wr_mask;
                if(!command_pending)data_first++;
            end
            if(command_pending)begin
                if(delay_count>0)delay_count<=delay_count-1;
                else if(held_cmd==1)begin
                    rd_data<=mem[held_addr];rd_valid<=1;rd_end<=1;
                    command_pending<=0;reads++;
                end else if(data_pending)begin
                    for(integer b=0;b<32;b++)
                        if(!held_mask[b])mem[held_addr][b*8+:8]<=held_data[b*8+:8];
                    command_pending<=0;data_pending<=0;writes++;
                end
            end
        end
    end
endmodule
