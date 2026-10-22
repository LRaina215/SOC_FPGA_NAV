`timescale 1ns/1ps
module tb_fc_v4_datapath;
    reg clk=0;
    reg signed [23:0] x;
    reg signed [15:0] w;
    reg signed [23:0] table_value;
    reg [23:0] table_mem[0:3072];
    reg signed [23:0] x_values[0:6];
    reg signed [15:0] w_values[0:6];
    reg signed [63:0] expected_product;
    integer i,j,index;
    fc_v4_top dut(.clk(clk),.rst_n(1'b0),.wr_en(1'b0),.wr_history(1'b0),
        .wr_addr(8'd0),.wr_data(24'sd0),.start(1'b0),.frame_id(32'd0));
    task check_activation;
        input signed [23:0] sample;
        begin
            x=sample;
            if (sample>=0) index=0;
            else begin index=(-sample+8)/16; if(index>3072) index=3072; end
            table_value=table_mem[index];
            #1;
            if (dut.elu_address !== index[11:0])
                $fatal(1,"ELU index x=%0d got=%0d expected=%0d",sample,dut.elu_address,index);
            if (sample>=0) begin
                if (dut.activated !== sample) $fatal(1,"ELU positive identity failed");
            end else if (dut.activated !== table_value) $fatal(1,"ELU table result failed");
        end
    endtask
    task check_clip;
        input signed [23:0] sample;
        input signed [23:0] expected;
        begin
            if (dut.clip_action(sample) !== expected) $fatal(1,"output clip %0d",sample);
        end
    endtask
    initial begin
        $readmemh("/home/cyd/fpga/pt_to_fpga/full_center_v4_deploy_20261005/assets/elu.mem",table_mem);
        force dut.normalized_q=x;
        force dut.layer=3'd0;
        force dut.elu_q=table_value;
        for(i=0;i<=49168;i=i+1) check_activation(-i);
        check_activation(-24'sd8388608);
        check_activation(-24'sd8388607);
        check_activation(24'sd1);
        check_activation(24'sd8388607);
        force dut.layer=3'd2;
        x=-24'sd8388608; #1;
        if(dut.activated!==x) $fatal(1,"linear encoder activation failed");
        force dut.layer=3'd6;
        x=-24'sd4097; #1;
        if(dut.activated!==x) $fatal(1,"linear raw action failed");
        check_clip(-24'sd8388608,-24'sd4096);
        check_clip(-24'sd4097,-24'sd4096);
        check_clip(-24'sd4096,-24'sd4096);
        check_clip(-24'sd4095,-24'sd4095);
        check_clip(24'sd0,24'sd0);
        check_clip(24'sd4095,24'sd4095);
        check_clip(24'sd4096,24'sd4096);
        check_clip(24'sd4097,24'sd4096);
        check_clip(24'sd8388607,24'sd4096);
        x_values[0]=-24'sd8388608; x_values[1]=-24'sd4096; x_values[2]=-24'sd1;
        x_values[3]=0; x_values[4]=1; x_values[5]=4096; x_values[6]=8388607;
        w_values[0]=-16'sd32768; w_values[1]=-16'sd4096; w_values[2]=-16'sd1;
        w_values[3]=0; w_values[4]=1; w_values[5]=4096; w_values[6]=32767;
        force dut.input_q=x; force dut.weight_q=w;
        for(i=0;i<7;i=i+1) for(j=0;j<7;j=j+1) begin
            x=x_values[i]; w=w_values[j];
            expected_product=$signed(x_values[i]); expected_product=expected_product*$signed(w_values[j]);
            #1;
            if($signed(dut.product)!==expected_product) $fatal(1,"signed24x16 MAC product corner failed");
        end
        $display("PASS DATAPATH ELU all49169 boundaries/clamp/identity linear clip signed-product49 corners");
        $finish;
    end
endmodule
