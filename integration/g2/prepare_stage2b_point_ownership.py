"""Prepare the single-frame point ownership candidate; never launches tools."""
from pathlib import Path
import argparse, difflib, json, shutil
from prepare_baseline_fusion_retime import hardware_inputs, metadata, relocate

BASE=Path('F:/nav_integration_g2_20261023')
SOURCE=BASE/'baseline_fusion50_ring_MAX_syn40'
POINT=BASE/'baseline_fusion50_pointplanes_prepared1'
HERE=Path(__file__).parent


def replace(text, old, new):
    assert text.count(old)==1, old
    return text.replace(old,new,1)


def transform_rx(text):
    text=replace(text,'    input  wire        pose_commit,    // 单拍', '''    input  wire        read_admit,     // owner admits a whole pass before its first pixel
    output wire        frame_accept,   // combinational: accepted raw-pass start at this edge
    input  wire        pose_commit,    // 单拍''')
    text=replace(text,'    reg         retain_frame;', '''    reg         retain_frame;
    reg         pose_pending;         // one request, original single-frame stable-config protocol
    assign frame_accept = rd_wait && read_admit && (!cfg_wait_pose || pose_commit || pose_pending);''')
    text=replace(text,"            rb <= 1'b0; rd_act <= 1'b0; rd_wait <= 1'b0; rp <= 0; rd_done <= 1'b0; retain_frame <= 1'b0;", "            rb <= 1'b0; rd_act <= 1'b0; rd_wait <= 1'b0; rp <= 0; rd_done <= 1'b0; retain_frame <= 1'b0; pose_pending <= 1'b0;")
    text=replace(text,"            if (!rd_act && !rd_wait && full[rb] && !clr_full) begin", "            if (pose_commit) pose_pending <= 1'b1;\n            if (!rd_act && !rd_wait && full[rb] && !clr_full) begin")
    text=replace(text,'            if (rd_wait && (!cfg_wait_pose || pose_commit)) begin', "            if (frame_accept) begin\n                pose_pending <= 1'b0;")
    return text


def transform_match(text):
    text=replace(text,'    input  wire        frame_en,            // 本帧提取（帧开始时采样）', '''    input  wire        frame_begin,         // same-edge raw start acceptance, before project/transform
    input  wire        frame_hold,          // MATCH_ONLY keeps this generation until UPDATE/cancel
    input  wire        match_enable,        // disable releases ownership after ongoing work drains
    output wire        frame_can_start,
    input  wire        frame_en,            // 本帧提取（raw受理时采样）''')
    text=replace(text,'    reg  signed [31:0] base_x, base_y;       // 点缓冲的平移基准（本帧 t）', '''    reg  signed [31:0] base_x, base_y;       // 点缓冲的平移基准（本帧 t）
    reg points_pending, point_owned, hold_prev;
    reg extract_armed, accepted_extract_en, wait_release;
    // Accepted capture always completes; a later disable never truncates its points.
    wire extract_en = extract_armed ? accepted_extract_en : frame_en;
    wire sof_extract = s_fire && s_sof && extract_en;
    wire producer_pending = points_pending || ex_busy || pb_we || (frame_begin && frame_en) || sof_extract;''')
    text=replace(text,'.clk(clk), .rst(rst), .en(frame_en),', '.clk(clk), .rst(rst), .en(extract_en),')
    text=replace(text,'S_T0W = 25, S_LUTW = 26, S_DBG2 = 28, S_UC2 = 29;', 'S_T0W = 25, S_LUTW = 26, S_WAIT_POINTS = 27, S_DBG2 = 28, S_UC2 = 29;')
    text=replace(text,'    assign busy = (st != S_IDLE) || lf_busy || ex_busy;', '''    assign busy = (st != S_IDLE) || lf_busy || ex_busy;
    assign frame_can_start = (st == S_IDLE) && !lf_busy && !points_pending && !ex_busy && !pb_we &&
                             !point_owned && !cmd_eval && !cmd_update && !cmd_clear;''')
    text=replace(text,"            st <= S_CLR; clr_a <= 0; pts_ready <= 1'b0; lfo_x <= -(N/2); lfo_y <= -(N/2); pw_pend <= 1'b0;", """            st <= S_CLR; clr_a <= 0; pts_ready <= 1'b0; lfo_x <= -(N/2); lfo_y <= -(N/2); pw_pend <= 1'b0;
            points_pending <= 1'b0; point_owned <= 1'b0; hold_prev <= 1'b0;
            extract_armed <= 1'b0; accepted_extract_en <= 1'b0; wait_release <= 1'b0;""")
    text=replace(text,"            if (s_fire && s_sof && frame_en) begin base_x <= cfg_tx; base_y <= cfg_ty; end\n            if (ex_done) pts_ready <= 1'b1;", """            hold_prev <= frame_hold;
            if (!match_enable || (hold_prev && !frame_hold)) point_owned <= 1'b0;
            if (ex_done) begin pts_ready <= 1'b1; points_pending <= 1'b0; end
            if (frame_begin) begin
                extract_armed <= 1'b1; accepted_extract_en <= frame_en;
                pts_ready <= 1'b0;
                if (frame_en) begin points_pending <= 1'b1; point_owned <= frame_hold; end
            end
            if (s_fire && s_sof) extract_armed <= 1'b0;
            if (sof_extract) begin
                base_x <= cfg_tx; base_y <= cfg_ty;
                points_pending <= 1'b1; pts_ready <= 1'b0;
            end""")
    text=replace(text,'                if (cmd_clear) begin st <= S_CLR; clr_a <= 0; end', "                if (cmd_clear) begin st <= S_CLR; clr_a <= 0; point_owned <= 1'b0; pts_ready <= 1'b0; end")
    text=replace(text,"                else if (cmd_update && cfg_prior) pts_ready <= 1'b0;        // 有图：只释放点缓冲", """                else if (cmd_update && cfg_prior) begin
                    if (producer_pending) begin wait_release <= 1'b1; st <= S_WAIT_POINTS; end
                    else begin pts_ready <= 1'b0; point_owned <= 1'b0; end
                end""")
    text=replace(text,'                    nsnap <= n_pts;', "                    point_owned <= 1'b1; wait_release <= 1'b0;\n                    nsnap <= n_pts;")
    text=replace(text,"dx_c <= 0; dy_c <= 0; dyaw_c <= 0; st <= S_LUT;", "dx_c <= 0; dy_c <= 0; dyaw_c <= 0; st <= producer_pending ? S_WAIT_POINTS : S_LUT;")
    text=replace(text,"dx_c <= upd_dx; dy_c <= upd_dy; dyaw_c <= upd_dyaw; st <= S_LUT;", "dx_c <= upd_dx; dy_c <= upd_dy; dyaw_c <= upd_dyaw; st <= producer_pending ? S_WAIT_POINTS : S_LUT;")
    text=replace(text,'            S_DBG:  st <= S_DBG2;', """            S_WAIT_POINTS: if (!producer_pending) begin
                if (wait_release) begin
                    pts_ready <= 1'b0; point_owned <= 1'b0; st <= S_IDLE;
                end else begin
                    nsnap <= n_pts;
                    stride <= (n_pts <= 1024) ? 3'd1 : (n_pts <= 2048) ? 3'd2 : (n_pts <= 3072) ? 3'd3 : 3'd4;
                    if (md == MD_UPD) pts_ready <= 1'b0;
                    st <= S_LUT;
                end
            end
            S_DBG:  st <= S_DBG2;""")
    text=replace(text,'            S_LFW: if (lf_done) st <= S_IDLE;', "            S_LFW: if (lf_done) begin st <= S_IDLE; point_owned <= 1'b0; end")
    return text


def transform_top(text):
    text=replace(text,'    wire        x_valid, x_ready, x_sof, x_eof, frm_start, frm_ready, pose_wait;', '''    wire        x_valid, x_ready, x_sof, x_eof, frm_start, frm_ready, pose_wait;
    wire raw_frame_accept, point_frame_admit;''')
    text=replace(text,'.cfg_raw_inv(r_LIDAR_VALID_CFG[31:16]), .cfg_wait_pose(r_LIDAR_CTRL[3]), .pose_commit(p_LIDAR_POSE_COMMIT),', '''.cfg_raw_inv(r_LIDAR_VALID_CFG[31:16]), .cfg_wait_pose(r_LIDAR_CTRL[3]), .pose_commit(p_LIDAR_POSE_COMMIT),
        .read_admit(r_LIDAR_CTRL[6] || point_frame_admit), .frame_accept(raw_frame_accept),''')
    text=replace(text,'        .frame_en(mt_frame_en),', '''        .frame_begin(raw_frame_accept && !fuse_only), .frame_hold(match_only),
        .match_enable(r_MATCH_CTRL[0]), .frame_can_start(point_frame_admit),
        .frame_en(mt_frame_en),''')
    # Coverage starts at the accepted pass, not an earlier held commit pulse.
    # The old immediate path is the same edge. Deferred path uses stable per-frame config.
    text=replace(text,'.frame_start(p_LIDAR_POSE_COMMIT & !match_only)', '.frame_start(raw_frame_accept & !match_only)')
    return text


def prepare(out: Path):
    assert not out.exists(), out
    manifest=json.loads((SOURCE/'manifest.json').read_text(encoding='utf-8-sig'))
    old_inputs=hardware_inputs(SOURCE,manifest)
    assert len(old_inputs)==63
    shutil.copytree(POINT,out,ignore=shutil.ignore_patterns('validation','__pycache__','*.patch','*.log','prepared_source_review.json','storage_contract.json','memory_inventory.json','storage_layout.json','synthesis_summary.json'))
    manifest=relocate(manifest,SOURCE,out)
    (out/'manifest.json').write_text(json.dumps(manifest,ensure_ascii=False,indent=2)+'\n',encoding='utf-8')
    changes={
        'nav/rtl/match/slf_match.v':transform_match,
        'nav/rtl/lidar/xt_frame_rx.v':transform_rx,
        'nav/rtl/top/nav_subsystem.v':transform_top,
    }
    diffs={}
    for rel,fn in changes.items():
        before=(POINT/rel).read_text(encoding='utf-8');after=fn(before)
        (out/rel).write_text(after,encoding='utf-8')
        patch='\n'.join(difflib.unified_diff(before.splitlines(),after.splitlines(),rel+' before',rel+' after',n=3))+'\n'
        diffs[rel]=patch
    inputs=hardware_inputs(out,manifest)
    rows=[]
    assert set(old_inputs)==set(inputs)
    for rel in sorted(old_inputs):
        rows.append(dict(relative=rel,same_bytes=old_inputs[rel].read_bytes()==inputs[rel].read_bytes()))
    changed=sorted(r['relative'] for r in rows if not r['same_bytes'])
    assert changed==sorted([*changes,'nav/rtl/common/nav_ram_sdp.v']), changed
    (out/'source_delta.patch').write_text(''.join(diffs.values()),encoding='utf-8')
    contract=dict(preparation_only=True,RTL_simulation_started=False,synthesis_started=False,PNR_started=False,
        source=str(SOURCE),parent_pointplanes=str(POINT),prepared=str(out),
        additional_BSRAM=0,additional_control_registers_source_prediction=7,
        new_ports=dict(xt_frame_rx=['read_admit','frame_accept'],slf_match=['frame_begin','frame_hold','match_enable','frame_can_start']),
        normal_transaction='Completed-ready EVAL/UPDATE enters original S_LUT on original edge. Existing bus FSM/ACK unchanged. PB read state schedule unchanged;4096x36 capacity retained.',
        raw_accept='frame_accept is the combinational actual raw-read-start condition. Same edge reserves points_pending, before project/transform.',
        boundary_commit='A single pose_commit pending bit survives lack of rd_wait or admission. Immediate eligible request bypasses this bit and starts on old edge.',
        protocol_limit='Original single-frame stable-configuration protocol only. Multiple commits are not a new FIFO; no346bit configuration snapshot. Client keeps per-frame configuration stable until accepted pass starts; no arbitrary mode/config rewrites during that pass.',
        early_commands='Only accepted/raw/extraction/final-write pending causes WAIT_POINTS; low ready alone does not. Existing work registers capture command data; snapshot count/stride is refreshed at true drain. No producer pending retains original empty-command behavior.',
        ownership='MATCH_ONLY captures retain ownership through UPDATE. EVAL also claims ownership. Online UPDATE releases after LF; prior UPDATE releases after producer drain. Normal unused frame capture is not permanently held.',
        cancellation='Reset clears all. IDLE soft-clear releases owner and ready. Match disable or falling HOLD releases ownership but does not truncate accepted extraction or stop an in-flight consumer; st/pending/ex_busy/pb_we continue blocking new writing passes until drain.',
        visible_changes=['ready clears at raw accepted frame, earlier than its SOF','Early command busy waits for true producer drain','Held commit acts later with original stable config','Coverage frame_start follows actual accepted pass; identical on ordinary rd_wait+admitted commit'],
        physical_pending='Actual CEB/BLKSEL must disable PB physical reads when re=0; source proof and RTL miter cannot prove this.',
        changed_hardware=changed,byte_differences=rows,
        validation='validation inputs added separately before any run; normal timing miter + early/cancel/zero/raw pending targeted cases')
    (out/'ownership_contract.json').write_text(json.dumps(contract,ensure_ascii=False,indent=2)+'\n',encoding='utf-8')
    review=dict(prepared=True,validation_not_run=True,bindings=[metadata(inputs[rel])for rel in sorted(inputs)],validation_bindings=[],source=str(SOURCE),prepared_directory=str(out),hardware_input_count=len(inputs),changed_hardware=changed)
    (out/'prepared_source_review.json').write_text(json.dumps(review,ensure_ascii=False,indent=2)+'\n',encoding='utf-8')
    print(json.dumps(dict(out=str(out),changed=changed,additional_BSRAM=0,simulation_started=False),ensure_ascii=False))


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--out',type=Path,required=True)
    prepare(p.parse_args().out)
