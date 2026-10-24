"""Share observation scaling after capture while retaining every output cycle.

Only frozen pipelined rl_observation_v5 is accepted. Keep the original wheel
pipeline, 64-step divider, rounding, saturation, history, reset and commit.
Raw inputs replace the parallel expanded numerator/denominator arrays.
"""
from pathlib import Path

DECL='''    reg signed [63:0] numerators [0:27];
    reg [63:0] denominators [0:27];'''
NEW_DECL='''    // Captured raw inputs; only consumed after a complete IDLE capture.
    // No reset is needed for private data that is overwritten before use.
    reg [71:0] angular_captured,gravity_captured;
    reg [143:0] actions_captured;
    reg signed [23:0] linear_captured,yaw_captured,height_captured;
    reg signed [63:0] left_numerator,right_numerator;'''
SELECT='''    // Select the raw lane first, then share exact constant scaling.
    // All normal inputs are signed24 and |scale|<=13: signed28 is sufficient.
    // The wheel path retains its original signed64 calculation separately.
    reg signed [23:0] lane_value;
    reg [3:0] lane_scale;
    reg [63:0] selected_denominator;
    wire signed [27:0] lane_extended={{4{lane_value[23]}},lane_value};
    reg signed [27:0] lane_scaled;
    wire signed [63:0] selected_numerator=lane==18 ? left_numerator :
        lane==21 ? right_numerator : {{36{lane_scaled[27]}},lane_scaled};
    always @* begin
        lane_value=0;lane_scale=1;selected_denominator=1;
        case(lane)
            0:begin lane_value=angular_captured[23:0];lane_scale=13;end
            1:begin lane_value=angular_captured[47:24];lane_scale=13;end
            2:begin lane_value=angular_captured[71:48];lane_scale=13;end
            3:begin lane_value=gravity_captured[23:0];lane_scale=3;end
            4:begin lane_value=gravity_captured[47:24];lane_scale=3;end
            5:begin lane_value=gravity_captured[71:48];lane_scale=3;end
            6:begin lane_value=linear_captured;lane_scale=2;end
            7:begin lane_value=yaw_captured;lane_scale=13;end
            8:begin lane_value=height_captured;lane_scale=5;end
            10:begin lane_value=actions_captured[23:0];lane_scale=3;end
            11:begin lane_value=actions_captured[47:24];lane_scale=3;end
            13:begin lane_value=actions_captured[95:72];lane_scale=3;end
            14:begin lane_value=actions_captured[119:96];lane_scale=3;end
            22:lane_value=actions_captured[23:0];
            23:lane_value=actions_captured[47:24];
            24:lane_value=actions_captured[71:48];
            25:lane_value=actions_captured[95:72];
            26:lane_value=actions_captured[119:96];
            27:lane_value=actions_captured[143:120];
            default:begin end
        endcase
        case(lane)
            0,1,2,7:selected_denominator=10;
            10,11,13,14:selected_denominator=100;
            18,21:selected_denominator=WHEEL_DEN;
            default:begin end
        endcase
        case(lane_scale)
            13:lane_scaled=(lane_extended<<<3)+(lane_extended<<<2)+lane_extended;
            5:lane_scaled=(lane_extended<<<2)+lane_extended;
            3:lane_scaled=(lane_extended<<<1)+lane_extended;
            2:lane_scaled=lane_extended<<<1;
            default:lane_scaled=lane_extended;
        endcase
    end
    integer j;'''
CAPTURE='''                    for(i=0;i<28;i=i+1) begin numerators[i]<=0;denominators[i]<=1;end
                    for(i=0;i<3;i=i+1) begin
                        numerators[i]<=$signed(angular_velocity_q12[i*24+:24])*64'sd13;
                        denominators[i]<=10;
                        numerators[3+i]<=$signed(gravity_q12[i*24+:24])*64'sd3;
                    end
                    numerators[6]<=$signed(linear_command_q12)*64'sd2;
                    numerators[7]<=$signed(yaw_command_q12)*64'sd13;denominators[7]<=10;
                    numerators[8]<=$signed(height_command_q12)*64'sd5;
                    for(i=0;i<6;i=i+1) begin
                        if(i!=2 && i!=5) begin
                            numerators[10+i]<=$signed(last_executed_actions[i*24+:24])*64'sd3;
                            denominators[10+i]<=100;
                        end
                        numerators[22+i]<=$signed(last_executed_actions[i*24+:24]);
                    end
                    left_captured<=left_delta;right_captured<=right_delta;
                    wheel_phase<=2'b01;
                    denominators[18]<=WHEEL_DEN;denominators[21]<=WHEEL_DEN;'''
NEW_CAPTURE='''                    angular_captured<=angular_velocity_q12;gravity_captured<=gravity_q12;
                    actions_captured<=last_executed_actions;
                    linear_captured<=linear_command_q12;yaw_captured<=yaw_command_q12;
                    height_captured<=height_command_q12;
                    left_numerator<=0;right_numerator<=0;
                    left_captured<=left_delta;right_captured<=right_delta;
                    wheel_phase<=2'b01;'''


def transform(text,prefetch=False):
    assert 'output reg initialized /* synthesis syn_maxfan = 128 */' in text
    pairs=[(DECL,NEW_DECL),('    integer i,j;',SELECT),(CAPTURE,NEW_CAPTURE),
           ('numerators[18]<=left_lo+(left_hi<<<18);','left_numerator<=left_lo+(left_hi<<<18);'),
           ('numerators[21]<=right_lo+(right_hi<<<18);','right_numerator<=right_lo+(right_hi<<<18);')]
    changed=text
    for old,new in pairs:
        assert changed.count(old)==1, ('unexpected frozen observation fragment',old[:80])
        changed=changed.replace(old,new,1)
    assert changed.count('numerators[lane]')==5 and changed.count('denominators[lane]')==3
    changed=changed.replace('numerators[lane]','selected_numerator').replace('denominators[lane]','selected_denominator')
    assert 'numerators[' not in changed and 'denominators[' not in changed
    if prefetch:
        direct='''    wire signed [63:0] selected_numerator=lane==18 ? left_numerator :
        lane==21 ? right_numerator : {{36{lane_scaled[27]}},lane_scaled};'''
        staged='''    // Prepare the next operand on the preceding SAVE/PREPARE edge.
    // The first lane is prepared directly from the live capture input.
    // No state, LOAD count, COMMIT or external output cycle is added.
    reg signed [63:0] selected_numerator;
    wire [4:0] fetch_lane=state==IDLE ? 5'd0 : lane+5'd1;
    wire signed [63:0] fetched_numerator=fetch_lane==18 ? left_numerator :
        fetch_lane==21 ? right_numerator : {{36{lane_scaled[27]}},lane_scaled};
    wire fetch_update=(state==IDLE && capture) || (lane!=27 &&
        (state==SAVE || (state==PREPARE && selected_denominator==1)));
    always @(posedge clk)if(rst_n && fetch_update)selected_numerator<=fetched_numerator;'''
        assert changed.count(direct)==1;changed=changed.replace(direct,staged,1)
        first='''        case(lane)
            0:begin lane_value=angular_captured[23:0];lane_scale=13;end'''
        ahead='''        case(fetch_lane)
            0:begin lane_value=state==IDLE ? angular_velocity_q12[23:0] : angular_captured[23:0];lane_scale=13;end'''
        assert changed.count(first)==1;changed=changed.replace(first,ahead,1)
    return changed


def generate(path,prefetch=False):
    path=Path(path);path.write_text(transform(path.read_text(encoding='utf-8'),prefetch),encoding='utf-8',newline='\n')
