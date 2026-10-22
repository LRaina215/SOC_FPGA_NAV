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
        num_observations = 27
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
        # 倾角超过此角度（度）即判定摔倒。
        # 基类用 84°（projected_gravity[2] > -0.1），但本车机身扁平，
        # 侧躺 55~60° 很稳定，策略会学会躺平。收紧到 30°。
        tilt_fail_deg = 30.0

    class terrain(LeggedRobotCfg.terrain):
        mesh_type = "plane"
        horizontal_scale = 0.3
        vertical_scale = 0.005
        border_size = 25
        curriculum = False
        static_friction = 0.6
        dynamic_friction = 0.6
        restitution = 0.15
        measure_heights = True
        measured_points_x = [-0.5, -0.4, -0.3, -0.2, -0.1, 0.0, 0.1, 0.2, 0.3, 0.4, 0.5]
        measured_points_y = [-0.3, -0.2, -0.1, 0.0, 0.1, 0.2, 0.3]
        selected = False
        terrain_kwargs = None
        max_init_terrain_level = 1
        terrain_length = 6.0
        terrain_width = 6.0
        num_rows = 5
        num_cols = 10
        terrain_proportions = [0.5, 0.0, 0.5, 0.0, 0.0, 0.0]
        slope_treshold = 0.75

    class commands(LeggedRobotCfg.commands):
        curriculum = False
        basic_max_curriculum = 2.5
        advanced_max_curriculum = 1.5
        curriculum_threshold = 0.7
        num_commands = 3
        resampling_time = 5.0
        heading_command = False
        standing_fraction = 0.25
        boundary_sampling = True

        class ranges(LeggedRobotCfg.commands.ranges):
            # 前进/后退：正值前进，负值后退。用户指定上限 ±0.6 m/s
            lin_vel_x = [-0.6, 0.6]
            # 左转/右转：用户指定与参考一致 ±2.9 rad/s
            ang_vel_yaw = [-2.9, 2.9]
            # 几何回代检查通过的 10.5cm 指令行程；动态行程需独立测量。
            height = [0.085, 0.19]
            heading = [-3.14, 3.14]

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
        min_reference_height = 0.085
        pos_action_scale = 0.03  # 高度 IK 前馈之外的小幅腿角修正 [rad]
        # torque 模式下该数值与 Wheel 阻尼相乘，实际是力矩动作增益。
        vel_action_scale = 4.0  # 开环力矩模式：4*0.025=0.1 N·m 满幅
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
        file = "{WHEEL_LEGGED_GYM_ROOT_DIR}/resources/robots/car/urdf/car_open.urdf"
        name = "CarRobot"
        # CAD 原点位于右侧，轴中点 X=-0.08075m；旧策略使用 0.0。
        forward_reference_x = -0.08075
        foot_name = "Wheel_link"       # 轮子作为足端
        offset = 0.02
        l1 = 0.075                      # 大腿长度 [m]
        l2 = 0.125                      # 小腿长度 [m]
        penalize_contacts_on = ["Lower", "base"]
        terminate_after_contacts_on = ["base"]
        self_collisions = 0
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
            tracking_lin_vel = 2.0
            tracking_lin_vel_enhance = 0
            tracking_ang_vel = 1.5
            tracking_ang_vel_enhance = 0

            base_height = 1.0
            base_height_enhance = 0
            nominal_state = 0.0
            lin_vel_z = 0.0
            ang_vel_xy = 0.0
            # 平滑姿态奖励及全域速度误差；不用旧的 ±dt 单项裁剪上限。
            orientation = -3.0
            upright = 3.0
            forward_error = -2.0
            yaw_error = -0.3
            leg_together = 0.0

            dof_vel = 0.0
            dof_acc = 0.0
            torques = 0.0
            action_rate = -0.005
            action_smooth = 0.0

            collision = 0.0
            dof_pos_limits = 0.0

            # 摔倒一次性惩罚。_reward_termination() 返回 reset_buf*~time_out_buf（0或1），
            # 框架先把权重乘 dt，实际终止贡献为 -5*dt（约 -0.04）。
            # 需配合姿态奖励和独立倾角评估，不能单凭该项判定学会站立。
            termination = -5.0

        only_positive_rewards = False
        clip_single_reward = 100
        tracking_sigma = 0.04
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
        # 使用较小探索噪声和固定学习率，指标以独立评估为准。
        entropy_coef = 0.001

    class policy(LeggedRobotCfgPPO.policy):
        init_noise_std = 0.08

    class runner(LeggedRobotCfgPPO.runner):
        max_iterations = 5000
        experiment_name = "car_robot"
        resume = False
