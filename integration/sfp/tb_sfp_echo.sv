`timescale 1ns/1ps
module tb_sfp_echo;
    reg clk=0, rx_clk=0, tx_clk=0, rst=1;
    always #10 clk=~clk;
    always #4 rx_clk=~rx_clk;
    initial begin #1; forever #4 tx_clk=~tx_clk; end
    reg rv=0, rl=0, re=0, tr=0;
    reg [7:0] rd=0;
    wire tv, tl;
    wire [7:0] td;
    wire [31:0] rxf, udp, err, arp, txf, echoes, drops;
    sfp_udp_echo dut (
        .clk(clk), .rst(rst), .rx_clk(rx_clk), .rx_rst(rst),
        .rx_valid(rv), .rx_data(rd), .rx_last(rl), .rx_error(re),
        .tx_clk(tx_clk), .tx_rst(rst), .tx_valid(tv), .tx_data(td),
        .tx_last(tl), .tx_ready(tr), .rx_frames(rxf), .rx_udp(udp),
        .rx_errors(err), .arp_requests(arp), .tx_frames(txf), .echoes(echoes), .echo_drops(drops)
    );
    integer output_file, received=0;
    reg [31:0] rng=32'hace12345;
    reg stalled=0;
    reg [8:0] saved;
    always @(negedge tx_clk) begin
        rng = {rng[30:0], rng[31]^rng[21]^rng[1]^rng[0]};
        tr = !rst && (rng[2:0] != 0);
    end
    always @(posedge tx_clk) begin
        if (!rst) begin
            if (stalled && (!tv || {tl,td} !== saved)) $fatal(1,"TX changed while stalled");
            stalled <= tv && !tr;
            saved <= {tl,td};
            if (tv && tr) begin
                $fdisplay(output_file,"%03x",{tl,td});
                if (tl) received <= received+1;
            end
        end
    end
    initial begin
        integer fd, scan, data, gap;
        output_file=$fopen("tx.hex","w");
        fd=$fopen("stimulus.txt","r");
        if (!fd || !output_file) $fatal(1,"Missing stimulus/output");
        repeat(20) @(negedge clk);
        rst=0;
        repeat(20) @(negedge rx_clk);
        while (!$feof(fd)) begin
            scan=$fscanf(fd,"%h %d\n",data,gap);
            if (scan==2) begin
                @(negedge rx_clk);
                rv=1; {re,rl,rd}=data[9:0];
                if (data[8]) begin
                    @(negedge rx_clk); rv=0; rl=0; re=0;
                    repeat(gap) @(negedge rx_clk);
                end
            end
        end
        repeat(20000) @(negedge clk);
        $fclose(fd); $fclose(output_file);
        if (echoes !== 5 || drops !== 4 || err !== 2 || arp !== 1 || received !== 6)
            $fatal(1,"counts echoes=%d drops=%d errors=%d arp=%d sent=%d",echoes,drops,err,arp,received);
        $display("TEST PASSED: 50MHz core, 125MHz RX/TX, backpressure, ARP, boundaries, error/drop recovery");
        $finish;
    end
    initial begin #10000000; $fatal(1,"Simulation timeout"); end
endmodule
