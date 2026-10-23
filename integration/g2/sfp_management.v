// Joint-image registers 0x1300..0x133c; decode/ownership is in g2_top.
// Reset/reconfiguration clears host networking only, never the 4500 controller.
module sfp_management (
    input wire clk,rst,req,we, input wire [7:0] addr, input wire [31:0] wdata,
    output wire ack, output reg [31:0] rdata,
    input wire [31:0] telemetry_accepted,telemetry_dropped,telemetry_sent,
    input wire sfp_los, output wire sfp_tx_disable,
    output wire rx_clk,rx_valid,rx_last,rx_error, output wire [7:0] rx_data,
    output wire tx_clk, input wire tx_valid,tx_last, input wire [7:0] tx_data, output wire tx_ready,
    output wire host_rst,rx_rst,tx_rst
);
    reg [2:0] cfg;
    reg [15:0] reset_hold;
    always @(posedge clk) begin
        if(rst) begin cfg<=3'b010;reset_hold<=16'hffff;end
        else if(req && we && addr==8'h04) begin cfg<=wdata[2:0];reset_hold<=16'hffff;end
        else if(reset_hold!=0) reset_hold<=reset_hold-1'b1;
    end
    wire ip_rst=rst || reset_hold!=0;
    wire [10:0] status_raw,status;
    wire [15:0] partner_raw,partner;
    wire [3:0] diagnostic_raw,diagnostic;
    assign diagnostic_raw[3]=sfp_los;
    sfp_host_mac u_mac (
        .clk(clk),.rst(ip_rst),.cfg_sgmii(cfg[0]),.cfg_an(cfg[1]),.cfg_use_los(cfg[2]),
        .sfp_los(sfp_los),.sfp_tx_disable(sfp_tx_disable),
        .rx_clk(rx_clk),.rx_valid(rx_valid),.rx_data(rx_data),.rx_last(rx_last),.rx_error(rx_error),
        .tx_clk(tx_clk),.tx_valid(tx_valid),.tx_data(tx_data),.tx_last(tx_last),.tx_ready(tx_ready),
        .status(status_raw),.partner(partner_raw),.qpll_ok(diagnostic_raw[0]),
        .rx_lock(diagnostic_raw[1]),.aligned(diagnostic_raw[2]));
    nav_sync2 #(.W(11)) u_status(.clk(clk),.d(status_raw),.q(status));
    nav_sync2 #(.W(16)) u_partner(.clk(clk),.d(partner_raw),.q(partner));
    nav_sync2 #(.W(4)) u_diag(.clk(clk),.d(diagnostic_raw),.q(diagnostic));
    // IPUG1021 Table 3-2: in SGMII, bit8 reports the partner PHY/copper link.
    // PCS synchronization/AN alone can stay up while the RJ45 cable is unplugged.
    wire link_complete=status[0] && status[1] && (!cfg[1] || status[2]) && (!cfg[0] || status[8]);
    reg [7:0] link_age;
    reg link_old;
    reg [31:0] link_losses;
    wire link_ready=&link_age;
    always @(posedge clk) begin
        if(rst) begin link_age<=0;link_old<=0;link_losses<=0;end
        else begin
            if(ip_rst || !link_complete) link_age<=0;
            else if(!link_ready) link_age<=link_age+1'b1;
            link_old<=link_ready;
            if(link_old && !link_ready) link_losses<=link_losses+1'b1;
        end
    end
    wire net_arst=ip_rst || !link_ready;
    nav_rst_sync u_hr(.clk(clk),.arst(net_arst),.rst(host_rst));
    nav_rst_sync u_rr(.clk(rx_clk),.arst(net_arst),.rst(rx_rst));
    nav_rst_sync u_tr(.clk(tx_clk),.arst(net_arst),.rst(tx_rst));
    assign ack=req;
    always @* begin
        case(addr)
            8'h00:rdata=32'h53465032; // SFP2: joint mapping, NOT standalone SFP1
            8'h04:rdata={29'd0,cfg};
            8'h08:rdata={21'd0,status};
            8'h0c:rdata={16'd0,partner};
            8'h10:rdata={26'd0,ip_rst,link_ready,diagnostic};
            8'h14:rdata=link_losses;
            8'h18:rdata=32'h00010083; // NV version 1, CONTROL_V2 type 0x83
            8'h20:rdata=telemetry_accepted;
            8'h24:rdata=telemetry_dropped;
            8'h28:rdata=telemetry_sent;
            default:rdata=32'hdeadbeef;
        endcase
    end
endmodule
