"""Read-only BMI-to-NAV interface/coordinate contract and HDL-string sketch.

No RTL or C file can be materialized by this entry. The new attitude conversion
is deliberately not called equivalent to the old WIT estimator.
"""
from pathlib import Path
from datetime import datetime
import argparse,json,yaml
from prepare_baseline_fusion_retime import metadata,save
BASE=Path('F:/nav_integration_g2_20261023');REPO=Path(__file__).resolve().parents[2]
HW=BASE/'SLF128_ring_stride32_SYN44_prepared1'
FW=BASE/'P11_quiesce_FW_sin_MMIO_prepared1'

REGISTERS=[
 ('BMI_RAW_SEQ',0x6030,'raw publication generation'),
 ('BMI_RAW_TS',0x6034,'NAV time_us when real BMI sample_valid accepted'),
 ('BMI_RAW_AX_AY',0x6038,'two original signed16 raw words:AY high,AX low'),
 ('BMI_RAW_AZ_GX',0x603c,'GX high,AZ low'),
 ('BMI_RAW_GY_GZ',0x6040,'GZ high,GY low'),
 ('BMI_RAW_STAT',0x6044,'health flags captured with raw data'),
 ('BMI_FUS_SEQ',0x6048,'fusion publication generation'),
 ('BMI_FUS_SAMPLE_TS',0x604c,'original state2 sample-accept time, not completion time'),
 ('BMI_FUS_PUBLISH_TS',0x6050,'real fusion valid publication time'),
 ('BMI_FUS_WX',0x6054,'original signed24 F12 rad/s, sign-extended32'),
 ('BMI_FUS_WY',0x6058,'same'),('BMI_FUS_WZ',0x605c,'same'),
 ('BMI_FUS_GX',0x6060,'original signed24 F12 projected DOWN gravity'),
 ('BMI_FUS_GY',0x6064,'same'),('BMI_FUS_GZ',0x6068,'same'),
 ('BMI_FUS_STAT',0x606c,'captured health and accel-rejected flags'),
 ('BMI_LIVE_STAT',0x6070,'current source reset/ready/error/calibrating/bias flags; never infer health only from old snapshot')]

# Interface-only sketch. Connections through core/mem_policy/imu_fusion and
# regfile are intentionally still an explicit integration obligation.
SNAPSHOT_MODULE=r'''// FUTURE source sketch: all ports are on the actual50MHz clock.
// raw_valid is bmi088.data_valid; fusion_accept is the original state2 sample
// acceptance event exported through real module ports, not a hierarchical force.
module nav_bmi_snapshot(
 input wire clk,rst,source_reset_n,input wire[31:0]time_us,
 input wire raw_valid,fusion_accept,fusion_valid,
 input wire sensor_ready,sensor_error,calibrating,bias_suspect,accel_rejected,
 input wire signed[15:0]ax,ay,az,gx,gy,gz,
 input wire signed[23:0]wx,wy,wz,fgx,fgy,fgz,
 output reg[31:0]raw_seq,raw_ts,raw_ax_ay,raw_az_gx,raw_gy_gz,raw_stat,
 output reg[31:0]fus_seq,fus_sample_ts,fus_publish_ts,fus_stat,
 output reg signed[23:0]fus_wx,fus_wy,fus_wz,fus_gx,fus_gy,fus_gz,
 output wire[31:0]live_stat
);
 wire[31:0]health={26'd0,accel_rejected,bias_suspect,calibrating,
                       sensor_error,sensor_ready,source_reset_n};
 reg[31:0]accepted_sample_ts;
 reg have_fusion_accept;
 assign live_stat=health;
 always @(posedge clk)begin
   if(rst||!source_reset_n)begin
     raw_seq<=0;raw_ts<=0;raw_ax_ay<=0;raw_az_gx<=0;raw_gy_gz<=0;raw_stat<=0;
     fus_seq<=0;fus_sample_ts<=0;fus_publish_ts<=0;fus_stat<=0;
     fus_wx<=0;fus_wy<=0;fus_wz<=0;fus_gx<=0;fus_gy<=0;fus_gz<=0;
     accepted_sample_ts<=0;have_fusion_accept<=0;
   end else begin
     if(raw_valid&&sensor_ready&&!sensor_error&&raw_seq!=32'hffffffff)begin
       raw_seq<=raw_seq+1'b1;raw_ts<=time_us;
       raw_ax_ay<={ay,ax};raw_az_gx<={gx,az};raw_gy_gz<={gz,gy};raw_stat<=health;
     end
     if(fusion_valid&&have_fusion_accept&&!calibrating&&!bias_suspect&&
        sensor_ready&&!sensor_error&&fus_seq!=32'hffffffff)begin
       fus_seq<=fus_seq+1'b1;fus_sample_ts<=accepted_sample_ts;fus_publish_ts<=time_us;
       fus_wx<=wx;fus_wy<=wy;fus_wz<=wz;fus_gx<=fgx;fus_gy<=fgy;fus_gz<=fgz;
       fus_stat<=health;have_fusion_accept<=0;
     end
     // A previous publication and the next state2 acceptance can coincide.
     // Publication consumes OLD accepted_sample_ts; new acceptance must win
     // the pending flag for the following frame, never be cleared by it.
     if(fusion_accept)begin accepted_sample_ts<=time_us;have_fusion_accept<=1;end
   end
 end
endmodule
'''

def ref(p,s):
    t=p.read_text(encoding='utf-8-sig');assert s in t,(p,s)
    return dict(path=str(p),line=t[:t.index(s)].count('\n')+1,text=s)

def main():
    ap=argparse.ArgumentParser(description=__doc__);ap.add_argument('--out',type=Path,required=True);a=ap.parse_args()
    assert not a.out.exists()
    paths={name:HW/rel for name,rel in dict(top='g2_top.v',core='control4500_core.v',
      sensor='control/bmi088.v',fusion='control/imu_fusion.v',policy='control/mem_policy.v',
      nav='nav/rtl/top/nav_subsystem.v',config='configuration_source/config/regmap.yaml').items()}
    paths.update({name:FW/rel for name,rel in dict(parser='fw/nav/imu_parse.c',parser_h='fw/nav/imu_parse.h',
      pose='fw/nav/pose.c',app='fw/app/nav_app.c',hal='fw/drv/hal_fpga.c',math='fw/nav/fixmath.c').items()})
    bindings={k:metadata(p)for k,p in paths.items()}
    cfg=yaml.safe_load(paths['config'].read_text(encoding='utf-8-sig'))
    occupied={b['base']+r['off']for b in cfg['blocks']for r in b['regs']}
    assert all(addr not in occupied for _,addr,_ in REGISTERS)
    assert not any(w['base']<=addr<w['base']+w['size']for _,addr,_ in REGISTERS for w in cfg['windows'])
    refs=dict(
      top_raw=ref(paths['top'],'.imu_rx(imu_rxd), .imu_tx(),'),
      top_only_odo=ref(paths['top'],'.odo_valid(odo_valid), .odo_vx(odo_vx), .odo_vy(16\'d0), .odo_wz(odo_wz),'),
      nav_odo=ref(paths['nav'],'odo_v <= {odo_vy, odo_vx}; odo_w <= {16\'d0, odo_wz}; odo_ts <= time_us;'),
      HAL_odo=ref(paths['hal'],'uint32_t cnt = nav_rd(NAV_SAFE_ODO_CNT);'),
      P11_skip_WIT=ref(paths['app'],'!p11_protocol && !p11_source_pending && i < 64 && hal_imu_byte'),
      WITgyro=ref(paths['parser'],"if (p->buf[1] == 0x52)"),
      WITatt=ref(paths['parser'],"if (p->buf[1] != 0x53) return false;"),
      WITaxes=ref(paths['parser_h'],'机身系 x 前 y 左 z 上'),
      accelrange=ref(paths['sensor'],"6'd7: config_item = {1'b0, 8'h41, 8'h00};"),
      gyrorange=ref(paths['sensor'],"6'd9: config_item = {1'b1, 8'h0f, 8'h00};"),
      rawpub=ref(paths['sensor'],'accel_x <= {accel_pending[39:32], accel_pending[47:40]};'),
      fusedunit=ref(paths['fusion'],'Outputs are physical units in signed 24-bit F12'),
      fusedaxes=ref(paths['fusion'],'x=right, y=forward, z=up'),
      fusedaccept=ref(paths['fusion'],"5'd2: if (sample_valid) begin"),
      fusedvalid=ref(paths['fusion'],'ang_vel_x <= avx;'),
      core_odoW=ref(paths['core'],'wire signed [47:0] feedback_gyro=$signed(telemetry_wz)*48\'sd1000;'),
      originalgyroconversion=ref(paths['pose'],'p->gyro_rr = (int32_t)gx * 32 / 9;'),
      originalatt=ref(paths['pose'],'void pose_imu_att(pose_est_t *p, angle_t yaw, angle_t pitch, angle_t roll, bool yaw_valid, uint32_t t_us)'),
      yawfallback=ref(paths['pose'],'if (!p->imu_att_valid || (uint32_t)(t_us - p->imu_t_us) > 100000)'),
      CORDIC=ref(paths['math'],'angle_t fx_atan2(int32_t y, int32_t x)'))
    r=dict(at=datetime.now().astimezone().isoformat(),phase='read-only interface/coordinate plan; not equivalent or accepted',
      bindings=bindings,references=refs,
      confirmed_gap=dict(control_feedback='real motor feedback_vx and fused telemetry_wz feed NAV SAFE_ODO; hal_odom still integrates horizontalpose/yaw',
        missing='no g2_top core BMI or gravity ports feed NAV. P11command mode explicitly skips WIT decoder; NAV pitch/roll,gyrohistory,attitudejump timestamps stop updating',
        P11_TX='outbound V2 telemetry is not internally fed to NAV; no assumption that RK echoes it',
        current_WIT='0x52 gyro and0x53 Euler consumed;0x51 accelerometer checksum counted but ignored; this is not a three-type attitudeinput',
        deployment_blocker=True),
      actual_units=dict(raw='originalsigned16 axis values, little-endian hardware extraction kept; atomic six-axis publication only afterbothbursts succeed',
        accel=dict(register='0x41=0',range_g=3,nominal_LSB_per_g=10920),
        gyro=dict(register='0x0f=0',range_dps=2000,nominal_LSB_per_dps=16.384,
          WIT_raw_scaling='same signedraw scale32768/2000dps, but coordinate mapping andbias are not automaticallysame'),
        official_sources=['https://www.bosch-sensortec.com/media/boschsensortec/downloads/datasheets/bst-bmi088-ds001.pdf',
          'https://www.bosch-sensortec.com/en/products/motion-sensors/imus/bmi088'],
        fusion='originalsigned24 F12 rad/s andDOWNgravity, startup200samplebias, latchedbias_suspect, individualaccelreject; noabsoluteheading'),
      sensor_time=dict(raw='same50MHz NAV time_us at accepted real BMI sample_valid publication; physical accel andgyro were acquired in sequential I2Cbursts, not simultaneouslysampled',
        fusion='must export existingstate2 sampleaccept through realports andcapturetime there; validcompletiontime also exposed separately',
        no_false_pairing='currentfusion laterstates readlive accel/gyro if rawupdateswhilebusy. Accepted TS anchors itsoriginalFSM event; do notclaim one six-axisrawsnapshot was fullylatched or change thatalgorithm silently',
        no_CDC='selectedsource isactual50MHz; ifanotherclockcandidate isselected thiscontractmustbereviewed again',
        source_reset='exportactualdevice_reset_n/startup; commonNAV reset clears snapshots, source-resetclearsgeneration; CPUHOLD alone doesnotclear them',
        stale='sequence doesnotadvance onstoppedsource; FWmustreadrealnow andlivehealth andapply existingfreshness/stopcontracts; neveruseCPUdequeue asmeasurementtime'),
      planned_CSR=[dict(name=n,address=hex(addr),description=d,acc='ro')for n,addr,d in REGISTERS],
      publication=dict(raw='threepacked32bits+TS+STAT+SEQ atomic',
        fused='six24bitwords signextended atCSR +acceptTS+publishTS+STAT+SEQ atomic',
        reads='SEQfirst→allrequiredwords/TS/STAT→SEQlast; equalnonzero nonsaturatedgeneration only; otherwise boundedretry nextservice, no fabricatedfreshness',
        health='LIVE_STAT independently exposes error/calibration/reset; previouslyhealthyfrozenpacket doesnotimplycurrenthealth',
        saturation='FFFF_FFFF freezesbank andFWrejects; resetepochrejectedbygeneration/ownerlogic, noautoarm',
        same_edge='previousfusionvalid plusnextstate2accept: publisholdsampleTS thennewacceptwins pendingflag; musttestthisactuallegalboundary'),
      coordinate_and_algorithm_decision=dict(
        NAV='imu_parse.h requiresbody xforward,yleft,zup; pose appliesRzRyRx. WITatt alreadyEuler,notgravity',
        control='imu_fusion sourceclaims xright,yforward,zup andDOWNgravity; citesIMU_CALIBRATION.md butthatbenchfile wasnotfoundinthisfrozeninput/repo search',
        candidate_rotation='IF sourceaxisdescription isphysicallyconfirmed, properrotation NAV=(controlY,-controlX,controlZ),det+1; must beexplicitandvalidatednot silentlyidentity',
        Euler='forNAVDOWNgravity transformedandnormaldomain, conventionalRzRyRx implies pitch=atan2(gx,sqrt(gy²+gz²)),roll=atan2(-gy,-gz); sign/inverted/near±90ambiguity andexactrounding requireindependentfixtures',
        reuse='existingfx_atan2/fx_isqrt canimplementconversion; rawgyro canthenfeedexistingpose_imu_gyro afterexplicitaxismap, preservingrawscale',
        difference='BMIgravityfusion→Euler→oldposecomplementaryfilter is a newsourceconversion, not provenidenticaltoexternalWITEuler; fullheadingunavailable, keepyaw_validfalse andoriginalodo/fusedwzheadingfallback',
        boundary='do notsubstitutearbitrarygyrotoattitude orrawaccelnormalization fororiginalfusion; no new estimator silentlyselected',
        required_before_implementation=['confirmmechanicalaxispermutationwithoriginalbenchposesornewboundedread-onlysensorcapture',
          'choose/recordpitch-rollwrapnearupside-downandatanroundingcontract','reviewsourcefreshness+invalidresetbehaviorbeforeposeupdate',
          'comparelevel/noseup/down/left-rightroll/rate/jump/zero/allquadrant syntheticvectors withindependentreferenceandactual50MHzsnapshotordering']),
      minimal_files=['control/imu_fusion.v: output existingstate2 accept-event only',
        'control/mem_policy.v: exposeaccept andexistingfusiondata/status, noalgorithmrewrite',
        'control4500_core.v: exportrealraw/fused/status/reset signals',
        'g2_top.v: same-clockports toNAV','nav/rtl/top/nav_subsystem.v+newnav_bmi_snapshot.v',
        'originalregmapgenerator input+generatedreadonlyCSRfiles','FWdriverboundedreader+poseingestionbranch; WITfallbackkept'],
      resource_and_budget=dict(module_string_source_bits=3*32+3*32+6*24+5*32+1,
        note='sourcebitcount isnotactualmappedFF/LUT orfinalminimalimplementation; existingtelemetrycopiesmaybeshareable butnotassumed',
        sourcehas_no_RAM_no_newclock=True,physicalallocation_unknown=True,
        FW='current1924B sparebelongspriorquiesce/snapshot/sineFW; no newBMIreader/Euler/CORDICcallcost/link/stack/WCETiscompiled here',
        originalFW_not_recompiled=True),
      source_sketch=dict(Python_string_bytes=len(SNAPSHOT_MODULE.encode()),materializer_available=False,
        remaining='noactualportpatch/regmaporFWselection; fullsourcehandoffmustbeboundtonewbaseaftercurrentbuilds'),
      RTL_generated=False,EDA_started=False,tests_started=False,deployment_ready=False)
    a.out.mkdir();save(a.out/'result.json',r)
    save(REPO/'docs/4_design/evidence/g2_20261023/BMI_NAV_internal_snapshot_contract_20261026.json',r)
    print(json.dumps(dict(output=str(a.out),gap=True,RTL=False,EDA=False,CSR_no_overlap=True)))

if __name__=='__main__':main()
