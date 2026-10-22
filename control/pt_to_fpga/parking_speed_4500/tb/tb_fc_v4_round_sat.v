`timescale 1ns/1ps
module tb_fc_v4_round_sat;
    reg signed [47:0] acc;
    wire signed [23:0] value;
    wire saturated;
    fc_v4_round_sat dut(.acc(acc),.value(value),.saturated(saturated));
    task check;
        input signed [47:0] a;
        input signed [23:0] expected;
        input expected_sat;
        begin
            acc=a; #1;
            if (value !== expected || saturated !== expected_sat)
                $fatal(1,"round acc=%0d got=%0d sat=%b expected=%0d sat=%b",a,value,saturated,expected,expected_sat);
        end
    endtask
    initial begin
        check(48'sd0,24'sd0,0);
        check(48'sd2047,24'sd0,0);
        check(48'sd2048,24'sd1,0);
        check(48'sd2049,24'sd1,0);
        check(-48'sd2047,24'sd0,0);
        check(-48'sd2048,-24'sd1,0);
        check(-48'sd2049,-24'sd1,0);
        check(48'sd34359736319,24'sd8388607,0);
        check(48'sd34359736320,24'sd8388607,1);
        check(-48'sd34359736319,-24'sd8388607,0);
        check(-48'sd34359736320,-24'sd8388608,0);
        check(-48'sd34359740415,-24'sd8388608,0);
        check(-48'sd34359740416,-24'sd8388608,1);
        check(48'sh7fffffffffff,24'sd8388607,1);
        check(48'sh800000000000,-24'sd8388608,1);
        $display("PASS ROUND_SAT ties-away signed24 saturation");
        $finish;
    end
endmodule
