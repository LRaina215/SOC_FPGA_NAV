`timescale 1ns/1ps
module tb_board_rl;
    localparam D=8, TIMEOUT=1000;
    reg clk=0; always #10 clk=~clk;
    reg rxd=1; wire txd; wire [11:0] actuators;
    fc_v4_board_top #(.CLKS_PER_BIT(D),.RX_TIMEOUT_CLKS(TIMEOUT),.POR_CLKS(8)) dut(
        .clk(clk),.UART_RXD(rxd),.UART_TXD(txd),.SPI_MISO(1'b0),.MT6701_A(1'b1),.MT6701_A_R(1'b1),
        .MOTOR_EN(actuators[0]),.PWM_U(actuators[1]),.PWM_V(actuators[2]),.PWM_W(actuators[3]),
        .MOTOR_EN2(actuators[4]),.PWM_U2(actuators[5]),.PWM_V2(actuators[6]),.PWM_W2(actuators[7]),
        .SERVO1(actuators[8]),.SERVO2(actuators[9]),.SERVO3(actuators[10]),.SERVO4(actuators[11]));
    localparam [255:0] DIGEST=256'h29b12e353b84380fa57ed7c3f8928fb1d51e9df99c0fa01bd3f4aec2de5a8c03;
    reg [23:0] nominal_inputs[0:41471], nominal_layers[0:108799], nominal_raw[0:1535];
    reg [23:0] nominal_actions[0:1535], nominal_latent[0:767], nominal_sat[0:255];
    reg [23:0] stress_inputs[0:1295], stress_layers[0:3399], stress_raw[0:47];
    reg [23:0] stress_actions[0:47], stress_latent[0:23], stress_sat[0:7];
    reg [7:0] req[0:65546], wire_bytes[0:65535];
    integer received=0, consumed=0, writes=0, starts=0;
    integer k, baseline, oldwrites, oldstarts, fd, sample, stress;
    reg sticky=0;
    reg [511:0] telemetry_test;
    reg [1023:0] root;
    reg [7:0] decoded;
    integer bitno;
    // Physical pin decoder, independent of the RTL UART implementation.
    initial forever begin
        @(negedge txd);
        #(D*10);
        if(txd!==0) $fatal(1,"TX false start");
        for(bitno=0;bitno<8;bitno=bitno+1) begin #(D*20); decoded[bitno]=txd; end
        #(D*20);
        if(txd!==1) $fatal(1,"TX bad stop bit");
        wire_bytes[received]=decoded; received=received+1;
    end
    always @(posedge clk) begin
        if(actuators!==12'd0) $fatal(1,"actuator output not constant zero");
        if(dut.rst_n) begin
            if(dut.bridge.core_wr) writes=writes+1;
            if(dut.bridge.job==2) starts=starts+1;
            if(dut.bridge.core_wr && (dut.bridge.core_busy || dut.bridge.response_active))
                $fatal(1,"input/result storage ownership violation");
        end
    end
    function [15:0] crc_byte;
        input [15:0] crc; input [7:0] value;
        reg [15:0] c; integer i;
        begin c=crc^{value,8'd0}; for(i=0;i<8;i=i+1) c=c[15]?(c<<1)^16'h1021:(c<<1); crc_byte=c; end
    endfunction
    function [23:0] reply_word;
        input integer index;
        begin reply_word={wire_bytes[consumed+index+2],wire_bytes[consumed+index+1],wire_bytes[consumed+index]}; end
    endfunction
    task send_byte;
        input [7:0] value; input bad_stop;
        integer i;
        begin
            @(negedge clk); rxd=0;
            repeat(D) @(negedge clk);
            for(i=0;i<8;i=i+1) begin rxd=value[i]; repeat(D) @(negedge clk); end
            rxd=!bad_stop; repeat(D) @(negedge clk); rxd=1;
        end
    endtask
    task make_packet;
        input [7:0] ver, typ; input integer length; input [31:0] id;
        input integer is_stress, which; input bad_crc;
        integer i; reg [23:0] word_value; reg [15:0] crc;
        begin
            req[0]=8'ha5; req[1]=8'h5a; req[2]=ver; req[3]=typ;
            req[4]=length; req[5]=length>>8;
            req[6]=id; req[7]=id>>8; req[8]=id>>16; req[9]=id>>24;
            for(i=0;i<length;i=i+1) begin
                word_value=0;
                if(i<486) word_value=is_stress ? stress_inputs[which*162+i/3] : nominal_inputs[which*162+i/3];
                req[10+i]=word_value[(i%3)*8 +: 8];
            end
            crc=16'hffff;
            for(i=2;i<length+10;i=i+1) crc=crc_byte(crc,req[i]);
            req[length+10]=crc[7:0] ^ (bad_crc ? 8'h01:8'h00); req[length+11]=crc[15:8];
        end
    endtask
    task send_packet;
        input [7:0] ver, typ; input integer length; input [31:0] id;
        input integer is_stress, which; input bad_crc;
        integer i;
        begin make_packet(ver,typ,length,id,is_stress,which,bad_crc);
            for(i=0;i<length+12;i=i+1) send_byte(req[i],0);
        end
    endtask
    task check_frame;
        input [7:0] typ; input integer length; input [31:0] id;
        integer i; reg [15:0] crc; reg [31:0] got_id;
        begin
            wait(received>=consumed+length+12); #1;
            if(wire_bytes[consumed]!==8'ha5 || wire_bytes[consumed+1]!==8'h5a || wire_bytes[consumed+2]!==1 || wire_bytes[consumed+3]!==typ)
                $fatal(1,"response header mismatch offset=%0d typ=%h",consumed,typ);
            if({wire_bytes[consumed+5],wire_bytes[consumed+4]}!==length[15:0]) $fatal(1,"response length mismatch");
            got_id={wire_bytes[consumed+9],wire_bytes[consumed+8],wire_bytes[consumed+7],wire_bytes[consumed+6]};
            if(got_id!==id) $fatal(1,"frame id got=%h expected=%h",got_id,id);
            crc=16'hffff;
            for(i=2;i<length+10;i=i+1) crc=crc_byte(crc,wire_bytes[consumed+i]);
            if({wire_bytes[consumed+length+11],wire_bytes[consumed+length+10]}!==crc) $fatal(1,"wire reply CRC mismatch");
        end
    endtask
    task drain;
        begin wait(!dut.bridge.reserved); repeat(12) @(negedge clk); end
    endtask
    task check_error;
        input [7:0] code; input [31:0] id;
        begin check_frame(8'hff,1,id);
            if(wire_bytes[consumed+10]!==code) $fatal(1,"error code got=%0d expected=%0d id=%h",wire_bytes[consumed+10],code,id);
            consumed=consumed+13;
            $display("PASS error code=%0d frame_id=%0d",code,id);
        end
    endtask
    task check_ping;
        input [31:0] id;
        integer i;
        begin check_frame(8'h82,32,id);
            for(i=0;i<32;i=i+1) if(wire_bytes[consumed+10+i]!==DIGEST[255-i*8 -: 8]) $fatal(1,"PING digest mismatch");
            consumed=consumed+44;
        end
    endtask
    task check_infer;
        input [31:0] id; input integer is_stress, which; input integer capture;
        integer i, out, meta; reg [23:0] expected; reg expected_sat; reg [31:0] cycles;
        begin
            check_frame(8'h81,1340,id);
            expected_sat=is_stress ? stress_sat[which]!=0 : nominal_sat[which]!=0;
            sticky=sticky|expected_sat;
            for(i=0;i<32;i=i+1) if(wire_bytes[consumed+10+i]!==DIGEST[255-i*8 -: 8]) $fatal(1,"INFER digest mismatch");
            cycles={wire_bytes[consumed+45],wire_bytes[consumed+44],wire_bytes[consumed+43],wire_bytes[consumed+42]};
            if(cycles!==81998 || wire_bytes[consumed+46]!=={3'd0,sticky,4'd0} || wire_bytes[consumed+47]!=={7'd0,expected_sat})
                $fatal(1,"INFER cycles/status mismatch cycles=%0d flags=%h sat=%h",cycles,wire_bytes[consumed+46],wire_bytes[consumed+47]);
            for(i=0;i<6;i=i+1) begin
                expected=is_stress ? stress_actions[which*6+i] : nominal_actions[which*6+i];
                if(reply_word(48+i*3)!==expected) $fatal(1,"action mismatch %0d",i);
                expected=is_stress ? stress_raw[which*6+i] : nominal_raw[which*6+i];
                if(reply_word(75+(419+i)*3)!==expected) $fatal(1,"raw action mismatch %0d",i);
            end
            for(i=0;i<3;i=i+1) begin
                expected=is_stress ? stress_latent[which*3+i] : nominal_latent[which*3+i];
                if(reply_word(66+i*3)!==expected) $fatal(1,"latent mismatch %0d",i);
            end
            for(i=0;i<425;i=i+1) begin
                expected=is_stress ? stress_layers[which*425+i] : nominal_layers[which*425+i];
                if(reply_word(75+i*3)!==expected) $fatal(1,"debug word mismatch %0d got=%h expected=%h",i,reply_word(75+i*3),expected);
            end
            if(capture) begin
                if(is_stress) begin out=$fopen({root,"/board/reports/rtl_reply_stress.hex"},"w"); meta=$fopen({root,"/board/reports/rtl_reply_stress.json"},"w"); end
                else begin out=$fopen({root,"/board/reports/rtl_reply_nominal.hex"},"w"); meta=$fopen({root,"/board/reports/rtl_reply_nominal.json"},"w"); end
                if(!out || !meta) $fatal(1,"cannot create UART capture");
                for(i=0;i<1352;i=i+1) $fwrite(out,"%02x\n",wire_bytes[consumed+i]);
                $fwrite(meta,"{\"frame_id\":%0d,\"sample_index\":%0d,\"dataset\":\"%0s\",\"initial_sticky\":false,\"clks_per_bit\":8,\"wire_bytes\":1352}\n",id,which,is_stress?"stress":"synthetic");
                $fclose(out); $fclose(meta);
            end
            consumed=consumed+1352;
            $display("PASS INFER frame_id=%0d dataset=%0s sample_index=%0d actions6 latent3 debug425 raw6 cycles81998 flags=%h saturated=%0d capture=%0d",id,is_stress?"stress":"synthetic",which,{3'd0,sticky,4'd0},expected_sat,capture);
        end
    endtask
    task reset_board;
        begin
            @(negedge clk); force dut.rst_n=0; rxd=1;
            repeat(5) @(negedge clk);
            release dut.rst_n; repeat(10) @(negedge clk); sticky=0;
            if(dut.bridge.core_busy || dut.bridge.last_id_valid || dut.bridge.reserved || dut.bridge.core.input_valid!=0) $fatal(1,"reset state/freshness failure");
        end
    endtask
    task negative;
        input [7:0] ver,typ; input integer length; input [31:0] id;
        input bad_crc; input [7:0] code;
        integer w,s,i;
        begin
            w=writes; s=starts;
            if(code>=1 && code<=3) begin
                make_packet(ver,typ,length,id,0,0,bad_crc);
                for(i=0;i<10;i=i+1) send_byte(req[i],0);
            end else send_packet(ver,typ,length,id,0,0,bad_crc);
            check_error(code,id); drain;
            if(writes!=w || starts!=s) $fatal(1,"invalid packet wrote core input");
            send_packet(1,2,0,id+1000,0,0,0); check_ping(id+1000); drain;
        end
    endtask
    initial begin
        if(!$value$plusargs("ROOT=%s",root)) $fatal(1,"ROOT required");
        $readmemh({root,"/assets/vectors/inputs.mem"},nominal_inputs);
        $readmemh({root,"/assets/vectors/expected_layers.mem"},nominal_layers);
        $readmemh({root,"/assets/vectors/expected_raw.mem"},nominal_raw);
        $readmemh({root,"/assets/vectors/expected_clipped.mem"},nominal_actions);
        $readmemh({root,"/assets/vectors/expected_latent.mem"},nominal_latent);
        $readmemh({root,"/assets/vectors/expected_saturated.mem"},nominal_sat);
        $readmemh({root,"/assets/vectors/stress/inputs.mem"},stress_inputs);
        $readmemh({root,"/assets/vectors/stress/expected_layers.mem"},stress_layers);
        $readmemh({root,"/assets/vectors/stress/expected_raw.mem"},stress_raw);
        $readmemh({root,"/assets/vectors/stress/expected_clipped.mem"},stress_actions);
        $readmemh({root,"/assets/vectors/stress/expected_latent.mem"},stress_latent);
        $readmemh({root,"/assets/vectors/stress/expected_saturated.mem"},stress_sat);
        wait(dut.rst_n); repeat(12) @(negedge clk);
        if(writes || starts || actuators!==0) $fatal(1,"POR failure");
        send_packet(1,2,0,32'h12345678,0,0,0); check_ping(32'h12345678); drain;
        for(k=0;k<64;k=k+1) telemetry_test[k*8 +: 8]=k;
        force dut.sensor_snapshot=telemetry_test;
        make_packet(1,3,0,90,0,0,0);
        for(k=0;k<12;k=k+1) send_byte(req[k],0);
        wait(dut.bridge.normal_pending || dut.bridge.response_active);
        // Change the live bus after acceptance: the response must stay frozen.
        telemetry_test=0;
        check_frame(8'h83,64,90);
        for(k=0;k<64;k=k+1)
            if(wire_bytes[consumed+10+k]!==k[7:0]) $fatal(1,"telemetry snapshot mismatch %0d",k);
        consumed=consumed+76; drain; release dut.sensor_snapshot;
        $display("PASS SENSOR_TELEMETRY atomic/frozen/64bytes/CRC");

        oldwrites=writes; send_packet(1,1,486,1,0,0,0); check_infer(1,0,0,1); drain;
        if(writes-oldwrites!=162) $fatal(1,"must load exactly162 words");
        send_packet(1,2,0,77,0,0,0); check_ping(77); drain;
        negative(1,1,486,1,0,7); // PING did not change duplicate tracking.
        reset_board;
        send_packet(1,1,486,2,1,1,0); check_infer(2,1,1,1); drain;
        for(sample=0;sample<8;sample=sample+1) if(sample!=1) begin
            send_packet(1,1,486,100+sample,1,sample,0);
            check_infer(100+sample,1,sample,0); drain;
        end
        send_packet(1,1,486,3,0,0,0); check_infer(3,0,0,0); drain; // sticky saturation retained
        negative(2,1,486,10,0,1);
        negative(1,8'h44,0,11,0,2);
        negative(1,1,485,12,0,3);
        negative(1,1,487,13,0,3);
        negative(1,2,1,14,0,3);
        negative(1,1,486,15,1,4);
        // Impossible length is rejected after exactly10 wire bytes, without
        // consuming following magic. Valid PING immediately after is parsed
        // and correctly rejected BUSY until the preceding error drains.
        oldwrites=writes; oldstarts=starts;
        make_packet(1,1,65535,81,0,0,0);
        for(k=0;k<10;k=k+1) send_byte(req[k],0);
        send_packet(1,2,0,1081,0,0,0);
        check_error(3,81); check_error(6,1081); drain;
        if(writes!=oldwrites || starts!=oldstarts) $fatal(1,"oversized packet touched core");
        send_packet(1,2,0,1081,0,0,0); check_ping(1081); drain;
        // A truncated header and a solitary magic byte also resynchronize.
        send_byte(8'ha5,0); send_byte(8'h5a,0); send_byte(1,0);
        check_error(5,0); drain;
        send_byte(8'ha5,0); check_error(5,0); drain;
        send_packet(1,2,0,1082,0,0,0); check_ping(1082); drain;
        // False start must not create either a byte or a protocol error.
        baseline=received; @(negedge clk); rxd=0; repeat(2) @(negedge clk); rxd=1;
        repeat(100) @(negedge clk); if(received!=baseline) $fatal(1,"false start generated reply");
        send_byte(8'ha5,0); send_byte(8'h5a,0); send_byte(1,1);
        repeat(20) @(negedge clk); check_error(5,0); drain;
        send_packet(1,2,0,16,0,0,0); check_ping(16); drain;
        // Truncated payload times out without any core writes.
        oldwrites=writes; make_packet(1,1,486,80,0,0,0);
        for(k=0;k<30;k=k+1) send_byte(req[k],0);
        check_error(5,80); drain; if(writes!=oldwrites) $fatal(1,"truncated packet wrote core");
        send_packet(1,2,0,17,0,0,0); check_ping(17); drain;
        // Magic overlap and garbage resynchronize without replies.
        send_byte(8'h33,0); send_byte(8'ha5,0); send_packet(1,2,0,18,0,0,0); check_ping(18); drain;
        // Back-to-back complete requests: first reply is atomic, next is BUSY.
        send_packet(1,2,0,19,0,0,0); send_packet(1,2,0,20,0,0,0);
        check_ping(19); check_error(6,20); drain;
        // Busy inference traffic never overwrites inputs. First error queued;
        // excess errors counted/dropped, normal reply always precedes error.
        oldwrites=writes; baseline=dut.bridge.dropped_errors;
        send_packet(1,1,486,21,0,1,0);
        send_packet(1,2,0,22,0,0,0); send_packet(1,2,0,23,0,0,0); send_packet(1,2,0,24,0,0,0);
        check_infer(21,0,1,0); check_error(6,22); drain;
        if(writes-oldwrites!=162 || dut.bridge.dropped_errors-baseline!=2) $fatal(1,"busy queue/drop or write count mismatch");
        // Packet begins busy but finishes after normal TX drains: still BUSY.
        send_packet(1,1,486,25,0,0,0); baseline=consumed;
        wait(received>=baseline+1000);
        send_packet(1,1,486,26,0,1,0);
        check_infer(25,0,0,0); check_error(6,26); drain;
        // Reset midway through a real inference; no stale results/readiness.
        send_packet(1,1,486,27,0,0,0); wait(dut.bridge.core_busy); repeat(300) @(negedge clk);
        baseline=received; reset_board; repeat(83000) @(negedge clk);
        if(received!=baseline) $fatal(1,"reset inference leaked response");
        send_packet(1,1,486,27,0,1,0); check_infer(27,0,1,0); drain;
        // Same-cycle core ERROR8 + RX framing ERROR5: core wins, one drop.
        send_packet(1,1,486,28,0,0,0); wait(dut.bridge.core_busy);
        force dut.bridge.core.error_flags=5'b00001;
        wait(dut.bridge.core_done); @(negedge clk);
        baseline=dut.bridge.dropped_errors;
        force dut.bridge.rx_error=1;
        @(negedge clk); release dut.bridge.rx_error;
        check_error(8,28); drain;
        if(dut.bridge.dropped_errors-baseline!=1) $fatal(1,"simultaneous error arbitration drop count");
        $display("PASS simultaneous core8/RX5 core-priority first-error-wins drop_count1");
        release dut.bridge.core.error_flags; reset_board;
        send_packet(1,1,486,28,0,0,0); check_infer(28,0,0,0); drain;
        if(received!=consumed) $fatal(1,"unconsumed/corrupted reply bytes");
        $display("PASS BOARD_RL protocol/errors1through8/busyqueue/drops/freshness/duplicate/reset/resync/backtoback/actuators-zero UART_divisor8");
        $finish;
    end
    initial begin #100000000; $fatal(1,"global board timeout received=%0d consumed=%0d rx_state=%0d job=%0d",received,consumed,dut.bridge.rx_state,dut.bridge.job); end
endmodule
