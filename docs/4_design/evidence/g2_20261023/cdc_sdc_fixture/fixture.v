module cdc_fixture(input clk50,rgmii_rxc,uart_rx,output led_heartbeat);
(* syn_keep=1 *) reg source=0,first=0,second=0;
always@(posedge clk50)source<=uart_rx;
always@(posedge rgmii_rxc)begin first<=source;second<=first;end
assign led_heartbeat=second;
endmodule
