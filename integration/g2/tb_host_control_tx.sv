`timescale 1ns/1ps
module tb_host_control_tx;
    reg clk=0;always #10 clk=~clk;
    reg rst=1;
    reg [959:0] payload=0;
    wire uart_new,uart_original,tv,tf,tl;
    wire [7:0] td;
    uart_telemetry #(.CLOCK_FREQ_HZ(200000),.FRAME_HZ(100),.CLKS_PER_BIT(1)) dut_uart(
        .clk(clk),.reset_n(~rst),.payload(payload),.tx(uart_new),
        .tap_valid(tv),.tap_first(tf),.tap_last(tl),.tap_data(td));
    uart_telemetry_original #(.CLOCK_FREQ_HZ(200000),.FRAME_HZ(100),.CLKS_PER_BIT(1)) original(
        .clk(clk),.reset_n(~rst),.payload(payload),.tx(uart_original));
    reg peer=0;
    reg [31:0] time_us=0;
    reg mb_go=0;
    wire mb_busy,mb_re,go,busy,pl_re;
    wire [10:0] mb_addr,pl_addr;
    reg [7:0] mb_q;
    wire [7:0] pl_q;
    wire [15:0] len;
    wire [31:0] accepted,dropped,sent;
    host_control_tx dut(.clk(clk),.rst(rst),.peer_valid(peer),.time_us(time_us),
        .tap_valid(tv),.tap_first(tf),.tap_last(tl),.tap_data(td),
        .mb_go(mb_go),.mb_len(16'd16),.mb_busy(mb_busy),.mb_re(mb_re),.mb_addr(mb_addr),.mb_q(mb_q),
        .tx_go(go),.tx_len(len),.tx_busy(busy),.pl_re(pl_re),.pl_addr(pl_addr),.pl_q(pl_q),
        .accepted(accepted),.dropped(dropped),.sent(sent));
    always @(posedge clk)if(mb_re)mb_q<=8'ha0+mb_addr;
    wire out_valid,out_last;wire [7:0] out_data;
    reg ready=0,stall=0;
    reg [31:0] random_state=32'h3157629a;
    udp_tx sender(.clk(clk),.rst(rst),.my_mac(48'h024e56000002),.my_ip(32'hc0a80264),
        .arp_req(1'b0),.arp_sha(48'd0),.arp_spa(32'd0),
        .tx_go(go),.tx_dst_mac(48'h0a1b2c3d4e5f),.tx_dst_ip(32'hc0a8020a),
        .tx_dst_port(16'd9001),.tx_src_port(16'd9000),.tx_len(len),.tx_busy(busy),
        .raw_go(1'b0),.raw_len(8'd0),.raw_busy(),.rb_re(),.rb_addr(),.rb_q(8'd0),
        .pl_re(pl_re),.pl_addr(pl_addr),.pl_q(pl_q),
        .m_valid(out_valid),.m_last(out_last),.m_data(out_data),.m_ready(ready),.st_tx());
    integer taps,frames,cycles,mailboxes,txfile,tapfile;
    reg saw_drop=0;
    always @(posedge clk)begin
        if(!rst)begin
            if(dropped!=0)saw_drop=1;
            if(uart_new!==uart_original)$fatal(1,"UART output changed");
            if(tv!==(original.active && original.ready))$fatal(1,"tap cadence changed");
            if(tv)begin
                if(td!==original.data || tf!==(original.index==0) || tl!==(original.index==125))
                    $fatal(1,"tap is not the original accepted UART byte");
                taps=taps+1;
                $fdisplay(tapfile,"%02x",td);
            end
            if(out_valid && ready)begin
                $fdisplay(txfile,"%03x",{out_last,out_data});
                if(out_last)frames=frames+1;
            end
        end
    end
    initial begin
        txfile=$fopen("tx.hex","w");tapfile=$fopen("tap.hex","w");
        taps=0;frames=0;mailboxes=0;
        repeat(8)@(negedge clk);rst=0;
        for(cycles=0;cycles<32000;cycles=cycles+1)begin
            @(negedge clk);
            time_us=time_us+1;
            random_state={random_state[30:0],random_state[31]^random_state[21]^random_state[1]^random_state[0]};
            ready=!stall && random_state[0];
            payload={payload[927:0],random_state};
            mb_go=0;
            if(cycles==4000)peer=1;
            if(cycles==7000)begin
                if(busy)$fatal(1,"unexpected active TX at initial stall");
                stall=1;
            end
            if(cycles==14000)begin stall=0;time_us=time_us+200000;end
            // A mailbox request while telemetry is pending/active must survive.
            if((cycles==7800 || cycles==16000 || cycles==22000) && !mb_busy)begin mb_go=1;mailboxes=mailboxes+1;end
            if(cycles==18000)begin
                rst=1;peer=0;$fdisplay(tapfile,"reset");
                if(busy)$fatal(1,"reset case should not truncate an Ethernet frame");
            end
            if(cycles==18008)begin rst=0;peer=1;end
        end
        if(taps<1000 || frames<5 || mailboxes!=3 || !saw_drop || sent<2)
            $fatal(1,"coverage insufficient taps=%0d frames=%0d mb=%0d dropped=%0d sent=%0d",taps,frames,mailboxes,dropped,sent);
        $display("TEST PASSED UART bit-for-bit; taps=%0d packets=%0d",taps,frames);
        $fclose(txfile);$fclose(tapfile);$finish;
    end
endmodule
