# 轮腿车（car）强化学习训练配置
#
# 本配置由 sf_robot_config.py 改写而来，适配 car_open.urdf（五连杆等效开链模型）。
# 关节命名与 sf1 不同，增益靠子串匹配，务必注意大小写。
#
# 关节索引按 IsaacGym 返回的名称查询，不能依赖 URDF 文本顺序。

from wheel_legged_gym.envs.base.legged_robot_config import (
    LeggedRobotCfg,
    LeggedRobotCfgPPO,
)


class CarRobotCfg(LeggedRobotCfg):

    class env(LeggedRobotCfg.env):
        num_envs = 4096
        # 27 = 角速度3 + 重力3 + 指令3 + 关节角6 + 关节速6 + 动作6
        # 2026-10-10: +1 = 第 4 路指令【跳跃】(0/1)
        num_observations = 28
        num_privileged_obs = num_observations + 7 * 11 + 3 + 6 * 5 + 3 + 3
        obs_history_length = 5
        obs_history_dec = 1
        num_actions = 6
        env_spacing = 3.0
        send_timeouts = True
        episode_length_s = 20
        dof_vel_use_pos_diff = False
        fail_to_terminal_time_s = 0.05
        reset_velocity_scale = 0.1
        # 2026-10-10：出生点从"中心 ±1m"改成"整块 ±spawn_radius"。
        # 目的：让一部分机器人直接出生在坡上 —— 没有平地可站，堵死"原地躺平"
        # 这条捷径。_reset_root_states 会按落点的真实地形高度修正 z。
        spawn_radius = 2.5
        # 倾角超过此角度（度）即判定摔倒。
        # 基类用 84°（projected_gravity[2] > -0.1），但本车机身扁平，
        # 侧躺 55~60° 很稳定，策略会学会躺平。收紧到 30°。
        tilt_fail_deg = 30.0

    class terrain(LeggedRobotCfg.terrain):
        # 2026-10-10：plane -> heightfield，要起伏路段，不要无限大平地。
        # 三条实测结论，改之前先知道：
        #   1) actor 观测里【没有】地形高度（num_observations 全给本体感觉+指令），策略是"盲"的，
        #      只能靠本体感觉事后反馈，别指望它预判前方障碍 —— 想让它"看见"要把
        #      measured_points 加进 compute_proprioception_observations()。
        #   2) horizontal_scale 原为 0.3，即一个高度格 30cm，比轮径 33.65mm 还大，
        #      起伏会被抹平。改回 legged_gym 标准的 0.1。
        #   3) env_origins 会重叠（几千个环境落在 num_rows×num_cols 块地形上），
        #      但 create_actor 的碰撞组按环境编号隔离（legged_robot.py:1387 传 group=i），
        #      不同 env 的机器人物理上互不干扰，只是视觉上叠在一起。
        # ================= 两阶段训练（2026-10-10 晚定）=================
        # 阶段 1（当前）：平地，先学会【基本控制】—— 前进/后退、转向、升降底盘、跳跃。
        #   理由：直接上 heightfield + 50% 离散障碍，等于同时学 4 个技能 + 地形；
        #   而且出问题时分不清是"平衡本身没学会"还是"地形太难"。
        # 阶段 2：把下面这行改回 "heightfield"，用
        #   --resume --load_run=<阶段1的目录> --checkpoint=<N>
        # 从阶段 1 的 checkpoint 微调，让已学会的平衡能力去适应地形。
        # ==============================================================
        # ==== 阶段 2（2026-10-10 晚）：已切回 heightfield ====
        # 阶段 1（平地）收敛在 Mean length 1850（14.8 秒），从 it100 起 1100 轮
        # 都在 1740~1910 之间晃，没有净进步 —— 平地已经学满了。
        # 现在用 --resume 从 Oct10_21-47-16_/model_1200.pt 继续，让它带着
        # 已学会的平衡能力适应地形。预期会掉一截，关键是看能不能涨回去。
        # ==== 2026-10-11 晚：跳跃轮 —— 回平地 ====
        # 地形轮（model_3000）收官后发现 heightfield 加载 bug（见 legged_robot.py 的
        # 2026-10-11 修复注释），那轮实际没见过设计地形；用户决定不再重训地形，
        # 直接在平地上叠加跳跃。地形代码保留，下一轮想再训地形改回 "heightfield"
        # 即可（修复已就位，届时是真地形）。
        mesh_type = "plane"

        horizontal_scale = 0.1
        # 2026-10-10：0.005 -> 0.001。高度场一格 = vertical_scale，
        # 原来 5mm/格，而我们要的粗糙起伏只有 ±5mm —— 会被 round 成 0 格（=平地）。
        # 改成 1mm/格后，±5mm 正好是 ±5 格。
        vertical_scale = 0.001

        # ---- 四种定制地形（2026-10-10 定）----
        # 由 terrain.py 的 make_terrain_custom() 实现，走这个开关。
        # 上坡 / 下坡 / 粗糙地面 / 飞坡，占比各 25%。
        custom_terrains = True
        slope_deg = 10.0      # 坡度 [度]。上坡/下坡/飞坡都用它
        rough_height = 0.005  # 粗糙地面起伏 ±[m]（配 vertical_scale=0.001 才能表示）
        # 坡从块内 2/3 处（4m）才开始 —— 因为 env_origin_z 取中心 2m 的最大高度，
        # 坡若从 3m 起，机器人会悬空 17.6cm 出生再往下掉。
        border_size = 25
        # 2026-10-11：改回【课程模式】—— 行=难度、列=类型（4 条长条）。
        # 前提：make_terrain_custom 已改为吃 difficulty（行号/num_rows），
        # 最难行满难度（10°/5mm/264mm 断崖），最易行约 1/4 难度（2.5°/1.25mm/65mm）。
        # 参照项目（DreamWaQ/LocoWheeledLegged/祖先）全部开课程从易到难 ——
        # 上轮 terrain_level 倒退就是"全难度随机 + 躺平"的信号。
        # 出发点用 max_init_terrain_level 控制（见下）。
        curriculum = True
        static_friction = 0.6
        dynamic_friction = 0.6
        restitution = 0.15
        measure_heights = True
        measured_points_x = [-0.5, -0.4, -0.3, -0.2, -0.1, 0.0, 0.1, 0.2, 0.3, 0.4, 0.5]
        measured_points_y = [-0.3, -0.2, -0.1, 0.0, 0.1, 0.2, 0.3]
        selected = False
        terrain_kwargs = None
        # 课程模式下的出生行：randint(0, level+1)。num_rows=5 → 行号只能 0~4，
        # 【不能填 5】（会越界索引 terrain_origins）。填 2 = 从最易的 3 行里出生，
        # 走得够远会被 _update_terrain_curriculum 升到难行，摔了降回来。
        max_init_terrain_level = 2
        terrain_length = 6.0
        terrain_width = 6.0
        num_rows = 5
        # 20 列才能对上 legged_robot.py:1455 写死的类别划分
        # [flat(0-3) 缓坡(4-7) 粗糙坡(8-11) 台阶上(12-13) 台阶下(14-17) 离散障碍(18-19)]
        num_cols = 20
        # 累积比例带，索引 = [flat, 缓坡, 粗糙坡, 台阶下, 台阶上, 离散障碍]
        # 2026-10-10 实测（见对话）：IsaacGym 的 pyramid_sloped_terrain 末尾有一句
        #   np.clip(h, min_h, max_h)   # min/max 取自平台角点
        # 把"坡"削成了缓碗，所以 rough slope 实际只是"高度不同的缓碗 + ±2cm 噪声"，
        # 出生区 2m 内高差仅 3cm，给不了颠簸。真正的起伏要用【离散障碍】：
        #   discrete obstacles -> ±7~13cm 方块，且出生平台是平的（适合起步）。
        # 现为：30% 平地（保底练平衡） + 20% 缓碗 + 50% 离散障碍（起伏路段）。
        # 想更温和把最后一项降到 0.3，想更狠提到 0.7。
        # 2026-10-10：只在 custom_terrains=True 时有意义。
        # 2026-10-11：4 -> 5 种地形（新增"山丘"= 连续上下坡，用户要求连贯翻山，
        # 原上坡块到顶是断崖）。5 个元素 = 5 种地形，课程布局下列号分段：
        #   列0-3 上坡 | 4-7 下坡 | 8-11 粗糙地面 | 12-15 飞坡 | 16-19 山丘
        terrain_proportions = [0.2, 0.2, 0.2, 0.2, 0.2]
        slope_treshold = 0.75

    class commands(LeggedRobotCfg.commands):
        curriculum = False
        # 2026-10-11：把扩张上限钉在 ±0.6（用户规格值）。指令课程分支跟随
        # terrain.curriculum 打开而激活（见 legged_robot.reset_idx），若保持
        # 2.5/1.5 会把速度指令扩张到远超规格；钉死后扩张无效果，保持 False 即可。
        basic_max_curriculum = 0.6
        advanced_max_curriculum = 0.6
        curriculum_threshold = 0.7
        # 保持 3：基类按 num_commands+1 分配 self.commands，所以实际是 4 列。
        # 第 3 列原本是 heading 槽，heading_command=False 时一直空着 —— 现在改作【跳跃指令】。
        num_commands = 3
        resampling_time = 5.0
        heading_command = False
        standing_fraction = 0.25
        boundary_sampling = True
        # 每次重采样时，这个比例的环境拿到跳跃指令 1（其余为 0）。
        # 太大会让策略一直在跳、学不会平稳行驶；太小则学不到跳跃。
        # 2026-10-11 晚：跳跃第二轮 0.15 -> 0.25（第一轮激励不足，跳跃分不动）。
        jump_fraction = 0.25

        class ranges(LeggedRobotCfg.commands.ranges):
            # 前进/后退：正值前进，负值后退。用户指定上限 ±0.6 m/s
            lin_vel_x = [-0.6, 0.6]
            # 左转/右转：用户指定与参考一致 ±2.9 rad/s
            ang_vel_yaw = [-2.9, 2.9]
            # 几何回代检查通过的 10.5cm 指令行程；动态行程需独立测量。
            height = [0.085, 0.19]
            heading = [-3.14, 3.14]
            # 跳跃指令只是 0/1 电平，范围写死；实际由 jump_fraction 采样
            jump = [0.0, 1.0]

    class init_state(LeggedRobotCfg.init_state):
        # 初始机身高度：几何计算的"轮底恰好贴地"高度 = 130.5 mm
        # （car_closed.xml 里写的是 140.2 mm，会比贴地高 9.7mm，落地有冲击）
        pos = [0.0, 0.0, 0.1305]
        # 标称姿态：左右腿对称（CAD 装配姿态本身不对称，见 MODEL_STATUS.md）
        #   CAD 零位: 右 hip=-63.4752deg knee=-82.9411deg
        #             左 hip=-30.1958deg knee=-108.1556deg
        #   对称目标:      hip=-46.8355deg knee=-95.5484deg
        # 关节轴: RightHip=+X, LeftHip=-X, Knee 都是 +X
        default_joint_angles = {
            "RightHip_joint": 0.290418,     # -46.8355 - (-63.4752) deg
            "RightKnee_joint": -0.220039,   # -95.5484 - (-82.9411) deg
            "RightWheel_joint": 0.0,
            "LeftHip_joint": 0.290418,      # -30.1958 - (-46.8355) deg
            "LeftKnee_joint": 0.220039,     # -95.5484 - (-108.1556) deg
            "LeftWheel_joint": 0.0,
        }

    class control(LeggedRobotCfg.control):
        control_type = "P"
        # 原实现实际上没有轮速反馈，是开环力矩近似；显式命名避免误解。
        wheel_control = "torque"
        height_reference = True
        height_reference_rate = 0.05
        motion_controller = "policy"
        # 2026-10-10: 打开。注意 request_jump() 原本只被评估脚本调用，训练里没人触发，
        # 所以另加了 _dispatch_jumps()（把指令位的上升沿转成一次 request_jump）。
        # 2026-10-11 白天: 地形阶段临时关闭。
        # 2026-10-11 晚: 跳跃轮 —— 重新打开，平地上从 model_3000 resume 微调。
        jump_enabled = True
        balance_kp = 0.6
        balance_kd = 0.05
        speed_kp = 0.08
        speed_ki = 0.01
        speed_reference_rate = 0.6
        yaw_kp = 0.015
        yaw_ki = 0.002
        forward_torque_limit = 0.12
        turn_torque_limit = 0.05
        velocity_source = "base"
        jump_crouch_s = 0.55
        jump_push_s = 0.08
        jump_push_hold_s = 0.0
        jump_push_height = 0.19
        jump_air_height = 0.09
        jump_leg_y = -0.03
        jump_leg_y_speed_gain = 0.0
        jump_air_torque_limit = 0.02
        min_reference_height = 0.085
        pos_action_scale = 0.03  # 高度 IK 前馈之外的小幅腿角修正 [rad]
        # torque 模式下该数值与 Wheel 阻尼相乘，实际是力矩动作增益。
        # 2026-10-10 晚：原值 4.0 -> 满幅 0.1 N·m，可救回的倾角只有
        #   sin(θ_max) = (0.1/0.03365) / (2.171*9.81) = 0.140  ->  θ_max = 8.0°
        # 而终止线是 30° —— 8~30° 全是必死区。训练 648 迭代奖励纹丝不动
        # （548 迭代只动 10%、action noise 0.08->0.03 塌缩）就是这个原因：
        # 物理上做不到，不是学不会。老车 car_open 只有 0.676 kg，同样 0.1 N·m
        # 下 θ_max = 26.6°，所以能学会。
        # 20 -> 0.5 N·m -> θ_max = 44.2°，远超 30° 终止线，无必死区。
        # 20*0.025 = 0.5 N·m，正好等于 URDF 里轮子的 effort —— 即把
        # JGB37-520 的堵转力矩 (5 kg·cm = 0.49 N·m) 全部释放。
        # 额定是 3.5 kg·cm = 0.343 N·m，真机连续跑在这之上要靠驱动器限流兜底。
        vel_action_scale = 20.0  # 开环力矩模式：20*0.025=0.5 N·m 满幅
        decimation = 4
        # PD 增益：键名靠【子串匹配】关节名，注意大小写
        # 当前开链仿真的 PD 设置，尚未经实物舵机响应标定。
        stiffness = {"Hip": 60.0, "Knee": 60.0, "Wheel": 0}
        # torque 模式中轮子力矩满幅 = 0.025*4 = 0.1 N·m。
        # 该仿真动作增益尚未经真实驱动板响应标定。
        damping = {"Hip": 0.06, "Knee": 0.06, "Wheel": 0.025}
        leg_integral_gain = 0.0
        integral_windup_limit = 1.5

    class sim(LeggedRobotCfg.sim):
        dt = 0.002
        substeps = 1

    class asset(LeggedRobotCfg.asset):
        # chy = 新车（all1 CAD：加了 FPGA/雷达/RK3588，前大腿改了）。
        # 质量来自 all1，运动学仍为 car_closed 推导的等效 2R（轮心差 12.8mm，待重导 CAD 修正）。
        file = "{WHEEL_LEGGED_GYM_ROOT_DIR}/resources/robots/chy/urdf/chy.urdf"
        name = "CarRobot"
        # CAD 原点位于右侧，轴中点 X=-0.08075m；旧策略使用 0.0。
        forward_reference_x = -0.08075
        foot_name = "Wheel_link"       # 轮子作为足端
        offset = 0.02
        l1 = 0.075                      # 大腿长度 [m]
        l2 = 0.125                      # 小腿长度 [m]
        penalize_contacts_on = ["Lower", "base"]
        terminate_after_contacts_on = ["base"]
        # 0 = 开启自碰撞，1 = 关闭。必须是 1！
        # 2026-10-10 实测：填 0 时 chy 的碰撞体（base 网格 + 两条腿的半径 5mm 细圆柱
        # + 轮子）在装配姿态下互相穿插，IsaacGym 每帧在 base 上产生 ~500 N 的【自接触力】。
        # 而 terminate_after_contacts_on = ["base"]，于是 fail_buf 每 6.25 步就连续超阈值，
        # 回合平均只活 0.06 秒 —— 训练根本跑不起来（实测 reset 率 15%/步）。
        # 改成 1 后：悬空接触力 500N -> 0.0N，reset 率 15% -> 肉眼可见的正常水平。
        self_collisions = 1
        flip_visual_attachments = False
        fix_base_link = False

    class normalization(LeggedRobotCfg.normalization):
        class obs_scales(LeggedRobotCfg.normalization.obs_scales):
            ang_vel = 1.3
            gravity = 3.0
            dof_vel = 0.15
            lin_vel = 2.0
            dof_pos = 1.0
            dof_acc = 0.0025
            height_measurements = 5.0
            torque = 0.05

        clip_observations = 100.0
        clip_actions = 1.0

    class domain_rand(LeggedRobotCfg.domain_rand):
        randomize_base_com = False
        randomize_Kp = False
        randomize_Kd = False
        randomize_inertia = False
        randomize_inertia_range = [0.9, 1.1]
        randomize_action_delay = False
        delay_ms_range = [0, 7]
        randomize_motor_torque = False
        randomize_motor_torque_range = [0.9, 1.1]
        randomize_friction = False
        friction_range = [0.1, 0.8]
        randomize_restitution = False
        restitution_range = [0.0, 0.6]
        randomize_base_mass = False
        added_mass_range = [-0.00, 0.08]
        push_robots = False
        push_interval_s = 0.5
        max_push_vel_xy = 0.05
        randomize_default_dof_pos = False
        randomize_default_dof_pos_range = [-0.05, 0.05]

    class rewards(LeggedRobotCfg.rewards):
        class scales(LeggedRobotCfg.rewards.scales):
            # ================= 2026-10-11 奖励重设计（参照 DreamWaQ_Go2W / LocoWheeledLegged / 本家祖先）=================
            # 核心变化：① 删掉 upright 正奖励（躺平的收入来源）；② 跟踪奖励乘"直立门控"
            # （不直立就拿不到跟踪分，见 car_robot.py）；③ base_height 改 L2 惩罚；
            # ④ 补上保底负项（力矩/加速度/限位/碰撞正则）。
            # 参照项目的共同结构：正收入只有"完成任务"一条路，姿态只罚不奖。
            # ==============================================================================================
            tracking_lin_vel = 3.0   # ×直立门控。原来 4.0 是为了压过 upright 3.0 的军备竞赛，
            #                            upright 删掉后不需要那么高，回到"跟踪是主要收入"的量级。
            tracking_lin_vel_enhance = 0
            tracking_ang_vel = 1.5   # ×直立门控。本车指令 ±2.9 rad/s 远比参照项目激进，保持 1.5
            tracking_ang_vel_enhance = 0

            base_height = -10.0      # L2 平方误差惩罚（实现见 car_robot.py 的覆写）。
            #                            exp(σ²=0.001) 弃用：系统性偏差 4~10mm 时 e^-16~-100 ≈ 0，
            #                            平地上就已经拿不到分。-10×err² 在小偏差温和、大偏差施压，
            #                            参照 DreamWaQ(-10)/LocoWheeledLegged(-10)。参考系是相对地形的（基类
            #                            base_height = root_z − 周围地形均值），坡上不失效。
            base_height_enhance = 0
            nominal_state = 0.0
            lin_vel_z = -1.0         # 垂直速度惩罚（防弹跳；飞坡冲坡 z 速度 ~0.1m/s 只罚 ~0.01/步，不挡路）
            ang_vel_xy = -0.05       # 横滚/俯仰角速度正则（参照项目均为 -0.05）
            # 姿态只罚不奖：祖先配置就是 orientation=-10（同族机器人在 trimesh 地形上验证过）。
            # 平衡车过坡时机身应保持世界水平（腿去贴合坡面），罚重力 xy 分量在坡上语义正确。
            orientation = -10.0
            upright = 0.0            # ★ 2026-10-11 删除：站着白拿 3.0/步 = 躺平的收入来源
            forward_error = -2.0     # 速度误差平方惩罚保留 —— 站着不动(cmd=0.6)时 -0.72/步，进一步堵躺平
            yaw_error = -0.3
            leg_together = 0.0

            dof_vel = 0.0            # 不开：会罚轮子转速，和行驶目标冲突
            dof_acc = -2.5e-7        # 腿关节加速度（实现只算腿 4 维）
            wheel_acc = -2.5e-9      # 轮子加速度，比腿轻 100 倍 —— 轮子要快速救平衡，别捆住它
            torques = -0.0001        # 力矩正则（祖先值）
            action_rate = -0.01      # 动作变化率，实现里轮子维 ×0.01（参照 LocoWheeledLegged 的分权重思路）
            action_smooth = 0.0

            collision = -1.0         # 碰撞保底（ancestor 值；chy 的 penalize_contacts_on 里 "Lower" 不匹配任何
            #                            link，实际只罚 base 触地，和终止条件互补）
            dof_pos_limits = -1.0    # 腿关节软限位（实现按名称索引，只算腿）

            # 摔倒不设一次性罚款（参照项目均无此项）：摔倒的真实代价 = 剩余回合的跟踪收入流中断。
            # 原来 -2.0 一次性（框架乘 dt 后实际 -0.016）本来就微不足道，删掉少一个混淆变量。
            termination = 0.0

            # ---- 跳跃奖励：2026-10-11 晚（跳跃轮）恢复 ----
            # 第一轮(2.0/30.0/-2.0) 1500 迭代跳跃分停在 0.004 —— 经济学上"跳一次赚~1 分、
            # 摔一次亏 ~30 分收入流"，没练过跳的 model_3000 算过账来选择不跳。
            # 第二轮加倍诱惑 + 提高指令占比：airborne 2→5、height 30→60、fraction 0.15→0.25。
            jump_airborne = 5.0
            jump_height = 60.0
            jump_landing = -2.0

        only_positive_rewards = False
        clip_single_reward = 100
        # 2026-10-11：0.04 -> 0.09。0.04 太尖（±0.15m/s 的地形波动就掉到 0.57），
        # 地形上速度天然起伏，需要给"不完美跟踪"留梯度；仍比参照项目的 0.25 尖，
        # 保持"跟踪=真的在动"的语义。
        tracking_sigma = 0.09
        soft_dof_pos_limit = 0.97
        soft_dof_vel_limit = 1.0
        soft_torque_limit = 1.0
        max_contact_force = 30.0

    class noise(LeggedRobotCfg.noise):
        add_noise = False
        noise_level = 1.0

        class noise_scales(LeggedRobotCfg.noise.noise_scales):
            dof_pos = 0.01
            dof_vel = 0.4
            lin_vel = 0.1
            ang_vel = 0.2
            gravity = 0.1
            height_measurements = 0.1


class CarRobotCfgPPO(LeggedRobotCfgPPO):
    class algorithm(LeggedRobotCfgPPO.algorithm):
        learning_rate = 1e-4
        schedule = "fixed"
        gamma = 0.995
        # 2026-10-10 晚：从 0.001 提到 0.01。
        # 2026-10-10 晚，两次调整的完整记录：
        #   0.001 -> Mean action noise std 塌缩 0.08 -> 0.02，早熟收敛到"小扰动稳定器"，
        #            Mean length 卡在 125 步（1 秒）不再涨。
        #   0.01  -> std 失控涨到 0.89，动作被噪声淹没，Mean length 从峰值 1397 掉回 1180。
        #   0.003 -> std 从 0.18 一路涨到 0.46（但轮子那两维只有 0.03/0.04，
        #            噪声都在腿上，实测不影响平衡），性能在 1740~1910 平了。
        #   0.002 -> 阶段 2 是微调，噪声该往回收了。
        # 2026-10-11：奖励表重设计后是【全新一轮从头训】（不 resume），回到
        # 从头训验证过的 0.003。
        # 2026-10-11 晚：跳跃轮是 resume 微调，噪声往回收，用 0.002。
        entropy_coef = 0.002

    class policy(LeggedRobotCfgPPO.policy):
        # 2026-10-10 晚：0.08 -> 0.25。初始动作噪声加大，才有机会在训练早期
        # 就采样到"轮子猛发力"的样本，否则策略一开始就锁死在小幅修正上。
        init_noise_std = 0.25
        # 【必须覆盖】：基类算的是
        #   LeggedRobotCfg.env.obs_history_length * LeggedRobotCfg.env.num_observations
        # 引用的是【基类】LeggedRobotCfg，不是 CarRobotCfg —— 所以它恒等于 5x27=135，
        # 不会跟着 car_robot 的 num_observations 走。2026-10-10 把观测从 27 加到 28
        # 就是栽在这：encoder 期望 135 收到 140，RuntimeError 直接崩在第一次前向。
        # 以后改 num_observations，这里要同步（这里用 CarRobotCfg 引用，自动跟随）。
        num_encoder_obs = CarRobotCfg.env.obs_history_length * CarRobotCfg.env.num_observations

    class runner(LeggedRobotCfgPPO.runner):
        max_iterations = 5000
        experiment_name = "car_robot"
        resume = False
