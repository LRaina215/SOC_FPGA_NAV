"""Keep the exact53-bit elevation arithmetic/forwarding under a shared CE."""


def derive(original):
    s=original
    def one(a,b):
        nonlocal s
        assert s.count(a)==1,a[:100]
        s=s.replace(a,b)
    one('module elev_map #(', 'module elev_map_ddr #(')
    one('    localparam integer AW = 2 * LOG2N;',
        '    localparam integer AW = 2 * LOG2N;\n    wire step_open;reg point_slot,cpu_slot;')
    one('    output reg         cpu_ack\n);', '''    output reg         cpu_ack,
    output wire step_ce,
    output wire mem_req,mem_we,output wire[2*LOG2N-1:0]mem_addr,
    output wire[52:0]mem_wdata,input wire mem_ack,input wire[52:0]mem_reply,
    input wire memory_fault
);''')
    one('    wire acc = s_valid & s_ready;\n    assign s_ready = ~clr_act & ~swp_sel;', '''    wire raw_acc=s_valid&~clr_act&~swp_sel;
    wire acc=step_open?raw_acc:point_slot;
    assign s_ready=~clr_act&~swp_sel&step_ce&(step_open||point_slot);''')
    one('    wire          cpu_go    = cpu_req & ~cpu_pend & ~cpu_ack & ~swp_sel & ~acc & ~clr_act;', '''    wire raw_cpu_go=cpu_req&~cpu_pend&~cpu_ack&~swp_sel&~raw_acc&~clr_act;
    wire cpu_go=step_open?raw_cpu_go:cpu_slot;''')
    a=s.index('    nav_ram_sdp #(.DW(53), .AW(AW)) u_ram (')
    b=s.index('    assign swp_rdata = q;',a)
    s=s[:a]+'''    // A point/CPU request arriving during an older write-only logical
    // step belongs to the NEXT step; it cannot be accepted without a read.
    always@(posedge clk)begin
        if(rst)begin point_slot<=0;cpu_slot<=0;end
        else if(step_open)begin point_slot<=raw_acc;cpu_slot<=raw_cpu_go;end
    end
    nav_ddr_dual_step #(.AW(AW),.DW(53)) u_memory(
        clk,rst,memory_fault,ram_re,ram_raddr,ram_we,ram_waddr,ram_wdata,
        step_ce,q,mem_req,mem_we,mem_addr,mem_wdata,mem_ack,mem_reply,step_open);
    reg clear_pending;
    always@(posedge clk)begin
        if(rst)clear_pending<=0;
        else if(clear&&!clr_act)clear_pending<=1;
        else if(step_ce&&clear_pending)clear_pending<=0;
    end
''' + s[b:]
    one('    assign busy = clr_act | v1 | v2 | fw1_v;',
        '    assign busy = clr_act | v1 | v2 | fw1_v | mem_req | !step_ce | clear_pending;')
    one('''        else begin
            cpu_pend <= cpu_go;''', '''        else if(step_ce)begin
            cpu_pend <= cpu_go;''')
    one('''        end else begin
            if (clear && !clr_act) begin''', '''        end else if(step_ce)begin
            if ((clear||clear_pending) && !clr_act) begin''')
    return s


def derive_traversability(original):
    s=original
    changes=[('module trav_v3 #(', 'module trav_v3_ce #('),
        ('    input  wire        rst,','    input  wire        rst,\n    input wire ce,'),
        ('.we(bm_we)', '.we(bm_we&&ce)'),('.re(bm_re)', '.re(bm_re&&ce)'),
        ('.we(ct_we)', '.we(ct_we&&ce)'),('.re(ct_re)', '.re(ct_re&&ce)'),
        ("        em_we <= 1'b0; cm_we <= 1'b0; bm_we <= 1'b0; done <= 1'b0; ct_we <= 1'b0;",
         "        done<=0;\n        if(ce)begin em_we<=0;cm_we<=0;bm_we<=0;ct_we<=0;end"),
        ("        if (rst) begin\n            ph <= P_IDLE;",
         "        if (rst) begin\n            em_we<=0;cm_we<=0;bm_we<=0;ct_we<=0;\n            ph <= P_IDLE;"),
        ('        end else begin\n            // ---------------- 扫描计数',
         '        end else if(ce)begin\n            // ---------------- 扫描计数')]
    for a,b in changes:
        assert s.count(a)==1,a
        s=s.replace(a,b)
    return s


def derive_map(original):
    s=original
    def one(a,b):
        nonlocal s
        assert s.count(a)==1,a[:100]
        s=s.replace(a,b)
    one('module map_subsys #(', 'module map_subsys_ddr #(')
    one('    output reg         map_ready        // 单拍：新代价图已切到前台', '''    output reg map_ready,
    output wire mem_req,mem_we,output wire[2*LOG2N-1:0]mem_addr,
    output wire[52:0]mem_wdata,input wire mem_ack,input wire[52:0]mem_reply,
    input wire memory_fault''')
    one('    wire        swp_busy;', '    wire em_step_ce;wire scan_ce;\n    wire        swp_busy;')
    one('    elev_map #(.LOG2N(LOG2N), .TAGB(TAGB)) u_em (',
        '    elev_map_ddr #(.LOG2N(LOG2N), .TAGB(TAGB)) u_em (')
    one('        .cpu_req(e_req), .cpu_raddr(e_addr), .cpu_ack(e_ack)', '''        .cpu_req(e_req), .cpu_raddr(e_addr), .cpu_ack(e_ack),
        .step_ce(em_step_ce),.mem_req(mem_req),.mem_we(mem_we),.mem_addr(mem_addr),
        .mem_wdata(mem_wdata),.mem_ack(mem_ack),.mem_reply(mem_reply),.memory_fault(memory_fault)''')
    one('    trav_v3 #(.LOG2N(LOG2N), .TAGB(TAGB)) u_tv (', '''    assign scan_ce=!swp_busy||em_step_ce;
    trav_v3_ce #(.LOG2N(LOG2N), .TAGB(TAGB)) u_tv (''')
    one('        .clk(clk), .rst(rst), .start(tv_start), .cx(sw_cx), .cy(sw_cy), .fc(frame_cnt),',
        '        .clk(clk), .rst(rst), .ce(scan_ce), .start(tv_start), .cx(sw_cx), .cy(sw_cy), .fc(frame_cnt),')
    one('    wire       re0 = front ? cm_re : (a_re | b_serve);',
        '    wire       re0 = front ? (cm_re&&scan_ce) : (a_re | b_serve);')
    one('    wire       re1 = front ? (a_re | b_serve) : cm_re;',
        '    wire       re1 = front ? (a_re | b_serve) : (cm_re&&scan_ce);')
    one('.we(cm_we & front)', '.we(cm_we & front & scan_ce)')
    one('.we(cm_we & ~front)', '.we(cm_we & ~front & scan_ce)')
    return s
