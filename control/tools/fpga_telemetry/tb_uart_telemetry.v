`timescale 1ns/1ps
module tb_uart_telemetry;
    localparam CPB=8;
    reg clk=0,reset_n=0;
    always #5 clk=~clk;
    reg [799:0] payload;
    wire tx;
    uart_telemetry #(.VERSION(1),.PAYLOAD_BYTES(100),.CLOCK_FREQ_HZ(1000000),.FRAME_HZ(50),.CLKS_PER_BIT(CPB)) dut(
        .clk(clk),.reset_n(reset_n),.payload(payload),.tx(tx));
    integer i,j,count,fd;
    reg [7:0] byte_value;
    initial begin
        for(i=0;i<100;i=i+1) payload[i*8+:8]=i;
        #30 reset_n=1;
        @(posedge dut.active);
        repeat(100) @(posedge clk);
        // A changing source must never corrupt the captured first frame.
        payload={800{1'b1}};
    end
    initial begin
        fd=$fopen("uart_bytes.hex","w");count=0;
        wait(reset_n);
        while(count<318) begin
            @(negedge tx);
            #(CPB*10+CPB*5);
            for(j=0;j<8;j=j+1) begin
                byte_value[j]=tx;
                #(CPB*10);
            end
            if(tx!==1'b1) $fatal(1,"Missing stop bit");
            $fdisplay(fd,"%02x",byte_value);count=count+1;
        end
        $fclose(fd);$display("UART_WIRE_PASS 318 bytes, 8N1 stop bits valid");$finish;
    end
    initial begin #1000000;$fatal(1,"UART timeout");end
endmodule
