`timescale 1ns/1ps
module tb_fc_v4_top;
    reg clk=0; always #5 clk=~clk;
    reg rst_n=0, wr_en=0, wr_history=0, start=0;
    reg [7:0] wr_addr=0;
    reg signed [23:0] wr_data=0;
    reg [31:0] frame_id=0;
    wire busy, done, saturated, dbg_valid;
    wire [4:0] error_flags;
    wire [31:0] result_frame_id, cycles;
    wire signed [23:0] action0,action1,action2,action3,action4,action5,dbg_data;
    wire [2:0] dbg_layer;
    wire [7:0] dbg_index;
    wire [143:0] actions={action5,action4,action3,action2,action1,action0};
    fc_v4_top dut(.clk(clk),.rst_n(rst_n),.wr_en(wr_en),.wr_history(wr_history),
        .wr_addr(wr_addr),.wr_data(wr_data),.start(start),.frame_id(frame_id),
        .busy(busy),.done(done),.error_flags(error_flags),.saturated(saturated),
        .result_frame_id(result_frame_id),.cycles(cycles),
        .action0(action0),.action1(action1),.action2(action2),.action3(action3),.action4(action4),.action5(action5),
        .dbg_valid(dbg_valid),.dbg_layer(dbg_layer),.dbg_index(dbg_index),.dbg_data(dbg_data));
    localparam MAX_SAMPLES=4096;
    reg [23:0] inputs [0:MAX_SAMPLES*168-1];
    reg [23:0] expected_layers [0:MAX_SAMPLES*425-1];
    reg [23:0] expected_raw [0:MAX_SAMPLES*6-1];
    reg [23:0] expected_clipped [0:MAX_SAMPLES*6-1];
    reg [23:0] expected_latent [0:MAX_SAMPLES*3-1];
    reg [23:0] expected_sat_file [0:MAX_SAMPLES-1];
    reg [1023:0] vector_dir;
    integer samples=256;
    integer s,k,count,elapsed,expected_layer,expected_index,base;
    integer saturated_frames=0;
    reg [143:0] previous_actions;
    reg [31:0] previous_frame;
    reg [143:0] expected_actions;
    reg expected_saturation [0:MAX_SAMPLES-1];
    reg expected_sticky_sat=0;
    integer vector_sample, l,n,m,ni,no,wo,bo,source_offset;
    reg signed [63:0] oracle_acc,oracle_x,oracle_w,oracle_rounded;
    task require_file;
        input [2047:0] name;
        integer fd;
        begin
            fd=$fopen(name,"r");
            if (!fd) $fatal(1,"missing memory file %0s",name);
            $fclose(fd);
        end
    endtask
    task build_saturation_oracle;
        begin
            for (s=0;s<samples;s=s+1) begin
                expected_saturation[s]=0;
                for (l=0;l<7;l=l+1) begin
                    case(l)
                        0: begin ni=140;no=128;wo=0;bo=0;source_offset=0;end
                        1: begin ni=128;no=64;wo=17920;bo=128;source_offset=0;end
                        2: begin ni=64;no=3;wo=26112;bo=192;source_offset=128;end
                        3: begin ni=31;no=128;wo=26304;bo=195;source_offset=0;end
                        4: begin ni=128;no=64;wo=30272;bo=323;source_offset=195;end
                        5: begin ni=64;no=32;wo=38464;bo=387;source_offset=323;end
                        6: begin ni=32;no=6;wo=40512;bo=419;source_offset=387;end
                    endcase
                    for (n=0;n<no;n=n+1) begin
                        oracle_acc=$signed(dut.bias_mem[bo+n]);
                        for (m=0;m<ni;m=m+1) begin
                            if(l==0) oracle_x=$signed(inputs[s*168+28+m]);
                            else if(l==3 && m<28) oracle_x=$signed(inputs[s*168+m]);
                            else if(l==3) oracle_x=$signed(expected_latent[s*3+m-28]);
                            else oracle_x=$signed(expected_layers[s*425+source_offset+m]);
                            oracle_w=$signed(dut.weight_mem[wo+n*ni+m]);
                            oracle_acc=oracle_acc+oracle_x*oracle_w;
                        end
                        if(oracle_acc<0) oracle_rounded=-((-oracle_acc+64'd2048)/64'd4096);
                        else oracle_rounded=(oracle_acc+64'd2048)/64'd4096;
                        if(oracle_rounded>64'sd8388607 || oracle_rounded < -64'sd8388608)
                            expected_saturation[s]=1;
                    end
                end
            end
        end
    endtask
    task tick;
        begin @(posedge clk); #1; end
    endtask
    task write_word;
        input integer address;
        input [23:0] value;
        begin
            @(negedge clk); wr_en=1; wr_history=(address>=28);
            wr_addr=(address>=28) ? address-28 : address; wr_data=value;
            tick;
            @(negedge clk); wr_en=0;
        end
    endtask
    task pulse_start;
        input [31:0] id;
        begin
            @(negedge clk); frame_id=id; start=1;
            tick;
            @(negedge clk); start=0;
        end
    endtask
    task reset_dut;
        begin
            @(negedge clk); rst_n=0; wr_en=0; start=0;
            tick;
            if (busy || done || error_flags || saturated || actions !== 144'd0 || result_frame_id || dbg_valid)
                $fatal(1,"reset state invalid");
            @(negedge clk); rst_n=1;
            tick;
        end
    endtask
    initial begin
        if ($value$plusargs("SAMPLES=%d",samples)) begin end
        if (!$value$plusargs("VECTOR_DIR=%s",vector_dir))
            vector_dir="/home/cyd/fpga/pt_to_fpga/full_center_v4_deploy_20261005/assets/vectors";
        if (samples<1 || samples>MAX_SAMPLES) $fatal(1,"bad sample count");
        require_file({vector_dir,"/inputs.mem"});
        require_file({vector_dir,"/expected_layers.mem"});
        require_file({vector_dir,"/expected_raw.mem"});
        require_file({vector_dir,"/expected_clipped.mem"});
        require_file({vector_dir,"/expected_latent.mem"});
        $readmemh({vector_dir,"/inputs.mem"},inputs,0,samples*168-1);
        $readmemh({vector_dir,"/expected_layers.mem"},expected_layers,0,samples*425-1);
        $readmemh({vector_dir,"/expected_raw.mem"},expected_raw,0,samples*6-1);
        $readmemh({vector_dir,"/expected_clipped.mem"},expected_clipped,0,samples*6-1);
        $readmemh({vector_dir,"/expected_latent.mem"},expected_latent,0,samples*3-1);
        #1;
        for(k=0;k<40704;k=k+1) if ((^dut.weight_mem[k])===1'bx) $fatal(1,"unknown/missing weight %0d",k);
        for(k=0;k<425;k=k+1) if ((^dut.bias_mem[k])===1'bx) $fatal(1,"unknown/missing bias %0d",k);
        for(k=0;k<3073;k=k+1) if ((^dut.elu_mem[k])===1'bx) $fatal(1,"unknown/missing ELU %0d",k);
        for(k=0;k<samples*168;k=k+1) if ((^inputs[k])===1'bx) $fatal(1,"unknown/truncated input %0d",k);
        for(k=0;k<samples*425;k=k+1) if ((^expected_layers[k])===1'bx) $fatal(1,"unknown/truncated layer %0d",k);
        for(k=0;k<samples*6;k=k+1) begin
            if ((^expected_raw[k])===1'bx) $fatal(1,"unknown/truncated raw %0d",k);
            if ((^expected_clipped[k])===1'bx) $fatal(1,"unknown/truncated clip %0d",k);
        end
        for(k=0;k<samples*3;k=k+1) if ((^expected_latent[k])===1'bx) $fatal(1,"unknown/truncated latent %0d",k);
        build_saturation_oracle;
        base=$fopen({vector_dir,"/expected_saturated.mem"},"r");
        if(base) begin
            $fclose(base);
            $readmemh({vector_dir,"/expected_saturated.mem"},expected_sat_file,0,samples-1);
            for(k=0;k<samples;k=k+1)
                if(expected_sat_file[k] !== {23'd0,expected_saturation[k]})
                    $fatal(1,"reference/oracle saturation disagreement sample %0d",k);
        end
        reset_dut;
        pulse_start(32'hffffffff);
        if (busy || done || !error_flags[3]) $fatal(1,"uninitialized start accepted");
        write_word(0,inputs[0]);
        pulse_start(32'hffffffff);
        if (busy || !error_flags[3]) $fatal(1,"partial load start accepted");
        @(negedge clk); wr_en=1; wr_history=0; wr_addr=28;
        tick;
        if (!error_flags[2]) $fatal(1,"invalid obs address not flagged");
        @(negedge clk); wr_history=1; wr_addr=140;
        tick;
        if (!error_flags[2]) $fatal(1,"invalid history address not flagged");
        @(negedge clk); wr_en=0;
        for (k=0;k<168;k=k+1) write_word(k,inputs[k]);
        @(negedge clk); wr_en=1; wr_history=0; wr_addr=0; wr_data=inputs[0]; start=1;
        tick;
        if (busy || !error_flags[1]) $fatal(1,"same-edge start/write accepted");
        @(negedge clk); wr_en=0; start=0;
        pulse_start(32'hdeadbeef);
        if (!busy) $fatal(1,"fully loaded start rejected");
        repeat(100) tick;
        reset_dut;
        pulse_start(0);
        if (busy || !error_flags[3]) $fatal(1,"reset failed to clear readiness");
        reset_dut;
        // Replay vector0 after any saturating frame: per-frame flag must clear,
        // while the sticky error flag must stay set until reset.
        for (vector_sample=0;vector_sample<=samples;vector_sample=vector_sample+1) begin
            s=(vector_sample==samples) ? 0 : vector_sample;
            for (k=0;k<168;k=k+1) write_word(k,inputs[s*168+k]);
            previous_actions=actions; previous_frame=result_frame_id;
            pulse_start(32'h10000000+s);
            if (!busy || done || saturated) $fatal(1,"start protocol failed sample %0d",s);
            count=0; elapsed=0;
            while (!done) begin
                // Illegal requests must neither disturb frozen inputs nor frame ID.
                if (elapsed==17) begin
                    @(negedge clk); wr_en=1; wr_history=1; wr_addr=0; wr_data=24'h800000;
                    start=1; frame_id=32'hffffffff;
                end
                tick;
                if ((^{busy,done,dbg_valid,saturated,error_flags,cycles,result_frame_id,actions})===1'bx)
                    $fatal(1,"unknown RTL output sample %0d",s);
                elapsed=elapsed+1;
                if (elapsed==18) begin
                    if (!error_flags[0] || !error_flags[1]) $fatal(1,"busy rejection missing");
                    @(negedge clk); wr_en=0; start=0;
                end
                if (elapsed>400000) $fatal(1,"latency exceeds limit");
                if (!done && (actions !== previous_actions || result_frame_id !== previous_frame))
                    $fatal(1,"non-atomic early result update sample %0d cycle %0d",s,elapsed);
                if (dbg_valid) begin
                    if (count>=425) $fatal(1,"too many debug words");
                    if (count<128) begin expected_layer=0; expected_index=count; end
                    else if (count<192) begin expected_layer=1; expected_index=count-128; end
                    else if (count<195) begin expected_layer=2; expected_index=count-192; end
                    else if (count<323) begin expected_layer=3; expected_index=count-195; end
                    else if (count<387) begin expected_layer=4; expected_index=count-323; end
                    else if (count<419) begin expected_layer=5; expected_index=count-387; end
                    else begin expected_layer=6; expected_index=count-419; end
                    if (dbg_layer !== expected_layer[2:0] || dbg_index !== expected_index[7:0])
                        $fatal(1,"debug ordering mismatch sample %0d word %0d",s,count);
                    if (dbg_data !== expected_layers[s*425+count])
                        $fatal(1,"layer mismatch sample %0d layer %0d neuron %0d rtl=%0d expected=%0d",s,dbg_layer,dbg_index,dbg_data,$signed(expected_layers[s*425+count]));
                    if (dbg_layer==2 && dbg_data !== expected_latent[s*3+dbg_index])
                        $fatal(1,"latent mismatch sample %0d",s);
                    if (dbg_layer==6 && dbg_data !== expected_raw[s*6+dbg_index])
                        $fatal(1,"raw mismatch sample %0d",s);
                    count=count+1;
                end
            end
            for (k=0;k<6;k=k+1) expected_actions[k*24+:24]=expected_clipped[s*6+k];
            if (busy || count!=425 || result_frame_id!==32'h10000000+s || actions!==expected_actions)
                $fatal(1,"completion mismatch sample %0d count %0d",s,count);
            if (cycles!==32'd83534 || elapsed!=83534) $fatal(1,"cycle count mismatch %0d %0d",cycles,elapsed);
            expected_sticky_sat=expected_sticky_sat | expected_saturation[s];
            if (saturated !== expected_saturation[s] || error_flags[4] !== expected_sticky_sat)
                $fatal(1,"saturation diagnostic mismatch sample=%0d got_frame=%b expected_frame=%b got_sticky=%b expected_sticky=%b",s,saturated,expected_saturation[s],error_flags[4],expected_sticky_sat);
            if (saturated && vector_sample<samples) saturated_frames=saturated_frames+1;
            if ((s%32)==0 || s==samples-1)
                $display("PASS vector %0d/%0d all425 raw6 clipped6 latent3 cycles=%0d saturated=%0d",s+1,samples,cycles,saturated);
            tick;
            if (done || dbg_valid) $fatal(1,"done/debug not a pulse");
            pulse_start(32'hbad00000+s);
            if (busy || !error_flags[3]) $fatal(1,"stale input frame accepted");
            write_word(0,inputs[s*168]);
            pulse_start(32'hbad00000+s);
            if (busy || !error_flags[3]) $fatal(1,"partial fresh frame accepted");
        end
        $display("PASS FULL_SYSTEM samples=%0d replay=1 layer_words=%0d saturated_frames=%0d latency=83534 protocol/reset/atomicity/freshness/saturation",samples,(samples+1)*425,saturated_frames);
        reset_dut;
        $finish;
    end
    initial begin #4000000000; $fatal(1,"global test timeout"); end
endmodule
