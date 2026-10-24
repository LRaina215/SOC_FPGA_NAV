// Explicit 1024-word banks avoid rounding the 12928-word image up to
// 16384 words. Each 32-bit bank needs two physical 18-bit blocks on 138C.
// Complete zero padding is generated from the unchanged original image.
module nav_ddr_boot_rom #(
    parameter integer IMAGE_WORDS=12928,
    parameter INIT_PREFIX="boot_b"
) (
    input wire clk,
    input wire [14:0] word_addr,
    output wire [31:0] rdata
);
    localparam integer BANKS=(IMAGE_WORDS+1023)/1024;
    localparam integer SEL_BITS=$clog2(BANKS)<1 ? 1 : $clog2(BANKS);
    wire [31:0] bank_q[0:BANKS-1];
    reg [SEL_BITS-1:0] selected;
    reg in_range;
    function [7:0] hex_digit;
        input integer n;
        begin hex_digit = n<10 ? 8'h30+n : 8'h61+n-10; end
    endfunction
    genvar b;
    generate for(b=0;b<BANKS;b=b+1)begin:g_bank
        (* syn_romstyle="block_rom" *) reg [31:0] image[0:1023];
        reg [31:0] q;
        initial $readmemh({INIT_PREFIX,hex_digit(b),".hex"},image);
        always @(posedge clk) q<=image[word_addr[9:0]];
        assign bank_q[b]=q;
    end endgenerate
    always @(posedge clk)begin
        selected<=word_addr[14:10];in_range<=word_addr<IMAGE_WORDS;
    end
    assign rdata=in_range ? bank_q[selected] : 32'd0;
endmodule
