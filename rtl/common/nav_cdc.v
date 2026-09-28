// 跨时钟域小件：电平两级同步、脉冲同步（翻转法）。
module nav_sync2 #(
    parameter integer W = 1
) (
    input  wire         clk,
    input  wire [W-1:0] d,
    output wire [W-1:0] q
);
    (* syn_keep = 1 *) reg [W-1:0] s1, s2;
    always @(posedge clk) begin
        s1 <= d;
        s2 <= s1;
    end
    assign q = s2;
endmodule

// 源域单拍脉冲 -> 目的域单拍脉冲。两次脉冲间隔需大于目的时钟 3 拍。
module nav_pulse_sync (
    input  wire sclk,
    input  wire srst,
    input  wire spulse,
    input  wire dclk,
    input  wire drst,
    output wire dpulse
);
    reg t;
    always @(posedge sclk) begin
        if (srst) t <= 1'b0;
        else if (spulse) t <= ~t;
    end
    wire ts;
    nav_sync2 #(.W(1)) u_s (.clk(dclk), .d(t), .q(ts));
    reg ts_d;
    always @(posedge dclk) begin
        if (drst) ts_d <= 1'b0;
        else ts_d <= ts;
    end
    assign dpulse = ts ^ ts_d;
endmodule

// 复位同步：异步置位，同步释放（高有效）。
module nav_rst_sync (
    input  wire clk,
    input  wire arst,
    output wire rst
);
    (* syn_keep = 1 *) reg r1, r2;
    always @(posedge clk or posedge arst) begin
        if (arst) begin
            r1 <= 1'b1;
            r2 <= 1'b1;
        end else begin
            r1 <= 1'b0;
            r2 <= r1;
        end
    end
    assign rst = r2;
endmodule
