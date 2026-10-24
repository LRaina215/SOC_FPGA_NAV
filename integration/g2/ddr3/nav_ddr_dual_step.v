// Preserve a synchronous read-first RAM step across a single request/ACK port.
// A simultaneous logical read/write is serialized READ then WRITE; all users
// advance only with step_ce. The returned Q changes on that logical edge.
module nav_ddr_dual_step #(parameter AW=14,DW=53)(
    input wire clk,rst,fault,
    input wire read_en,input wire[AW-1:0]read_addr,
    input wire write_en,input wire[AW-1:0]write_addr,input wire[DW-1:0]write_data,
    output wire step_ce,output reg[DW-1:0]q,
    output reg req,we,output reg[AW-1:0]addr,output reg[DW-1:0]data,
    input wire ack,input wire[DW-1:0]reply,output wire step_open
);
    localparam IDLE=0,READ=1,READ_GAP=2,WRITE=3,WRITE_GAP=4,COMPLETE=5;
    reg[2:0]state;
    reg held_read,held_write;reg[AW-1:0]held_wa;reg[DW-1:0]held_wd,read_result;
    assign step_ce=!fault&&(!(read_en||write_en)||state==COMPLETE);
    assign step_open=state==IDLE;
    always@(posedge clk)begin
        if(rst)begin state<=IDLE;req<=0;we<=0;addr<=0;data<=0;q<=0;
            held_read<=0;held_write<=0;held_wa<=0;held_wd<=0;read_result<=0;end
        else if(!fault)case(state)
            IDLE:if(read_en||write_en)begin
                held_read<=read_en;held_write<=write_en;held_wa<=write_addr;held_wd<=write_data;
                req<=1;we<=!read_en;addr<=read_en?read_addr:write_addr;data<=write_data;
                state<=read_en?READ:WRITE;
            end
            READ:if(ack)begin read_result<=reply;req<=0;state<=READ_GAP;end
            READ_GAP:if(!ack)begin
                if(held_write)begin req<=1;we<=1;addr<=held_wa;data<=held_wd;state<=WRITE;end
                else state<=COMPLETE;
            end
            WRITE:if(ack)begin req<=0;state<=WRITE_GAP;end
            WRITE_GAP:if(!ack)state<=COMPLETE;
            COMPLETE:if(step_ce)begin
                if(held_read)q<=read_result;
                state<=IDLE;
            end
            default:state<=IDLE;
        endcase
    end
endmodule
