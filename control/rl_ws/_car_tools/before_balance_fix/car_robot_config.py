# 轮腿车（car）强化学习训练配置
#
# 本配置由 sf_robot_config.py 改写而来，适配 car_open.urdf（五连杆等效开链模型）。
# 关节命名与 sf1 不同，增益靠子串匹配，务必注意大小写。
#
# 关节顺序（URDF 中定义顺序）：
#   RightHip_joint, RightKnee_joint, RightWheel_joint,
#   LeftHip_joint,  LeftKnee_joint,  LeftWheel_joint

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
        dof_vel_use_pos_diff = True
        fail_to_terminal_time_s = 1
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
        curriculum = True
        basic_max_curriculum = 2.5
        advanced_max_curriculum = 1.5
        curriculum_threshold = 0.7
        num_commands = 3
        resampling_time = 5.0
        heading_command = True

        class ranges(LeggedRobotCfg.commands.ranges):
            # 前进/后退：正值前进，负值后退。用户指定上限 ±0.6 m/s
            lin_vel_x = [-0.6, 0.6]
            # 左转/右转：用户指定与参考一致 ±2.9 rad/s
            ang_vel_yaw = [-2.9, 2.9]
            # 底盘升降：先用小行程（几何极限约 5~20cm，但可平衡区间远小于此）
            # 待仿真实测可用范围后再扩大
            height = [0.11, 0.15]
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
        pos_action_scale = 0.1   # 腿部位置增量缩放 [rad]
        # 轮速缩放：按真机 2208 110KV@12V 定
        #   空载 110*12 = 1320 rpm = 138 rad/s -> 线速度 4.49 m/s
        #   取 20 rad/s 覆盖 0.65 m/s（> 目标的 0.6），留余量
        vel_action_scale = 20.0
        decimation = 4
        # PD 增益：键名靠【子串匹配】关节名，注意大小写
        # kp=60 是仿真扫描出的最优值：关节跟踪误差最小(0.19°)，力矩仅用 9%
        # 比 kp=10 硬 6 倍（塌陷 0.19° vs 1.1°），接近真机闭环的刚性
        stiffness = {"Hip": 60.0, "Knee": 60.0, "Wheel": 0}
        # 轮子阻尼：决定最大轮力矩 = damping * vel_action_scale
        #   质心在轮轴前 20mm，静平衡需 0.0663 N·m/轮
        #   damping=0.025 -> 最大 0.5 N·m (=5.8A)，是需求的 7.5 倍
        #   旧值 0.004 只能给 0.08 N·m，物理上撑不住（这是站不住的真因）
        damping = {"Hip": 0.06, "Knee": 0.06, "Wheel": 0.025}
        leg_integral_gain = 0.1
        integral_windup_limit = 1.5

    class sim(LeggedRobotCfg.sim):
        dt = 0.002
        substeps = 1

    class asset(LeggedRobotCfg.asset):
        file = "{WHEEL_LEGGED_GYM_ROOT_DIR}/resources/robots/car/urdf/car_open.urdf"
        name = "CarRobot"
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
        clip_actions = 100.0

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
        push_robots = True
        push_interval_s = 0.5
        max_push_vel_xy = 0.05
        randomize_default_dof_pos = False
        randomize_default_dof_pos_range = [-0.05, 0.05]

    class rewards(LeggedRobotCfg.rewards):
        class scales(LeggedRobotCfg.rewards.scales):
            tracking_lin_vel = 1.2
            tracking_lin_vel_enhance = 0
            tracking_ang_vel = 3.2
            tracking_ang_vel_enhance = 0

            base_height = 1.0
            base_height_enhance = 0
            nominal_state = -0.1
            lin_vel_z = -1.0
            ang_vel_xy = -0.05
            # 姿态惩罚。注意 clip_single_reward = 1 会把每项限制在 ±dt = ±0.008/步，
            # 所以 -60 和 -10 在大部分情况下都会被截断到同一上限，调大反而无意义。
            # 保持与参考仓库一致，真正的站直激励靠 base_height 跟踪。
            orientation = -10
            leg_together = -40.0

            dof_vel = -5e-5
            dof_acc = -2.5e-7
            torques = -0.0001
            action_rate = -0.01
            action_smooth = -0.01

            collision = -1.0
            dof_pos_limits = -1.0

            # 摔倒一次性惩罚。_reward_termination() 返回 reset_buf*~time_out_buf（0或1），
            # 配合负权重 => 摔倒时扣 20 分。
            # 基类默认不启用这一项，导致"躺平"几乎无代价（实测策略因此学会侧躺）。
            termination = -20.0

        only_positive_rewards = False
        clip_single_reward = 1
        tracking_sigma = 0.01
        soft_dof_pos_limit = 0.97
        soft_dof_vel_limit = 1.0
        soft_torque_limit = 1.0
        max_contact_force = 30.0

    class noise(LeggedRobotCfg.noise):
        add_noise = True
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
        gamma = 0.995
        # 实测：0.02 会让 action noise std 从 0.5 一路涨到 2.47，策略变乱动、
        # Mean length 从 2159 崩到 161。降到 0.002 抑制噪声爆炸。
        entropy_coef = 0.002

    class policy(LeggedRobotCfgPPO.policy):
        init_noise_std = 0.5

    class runner(LeggedRobotCfgPPO.runner):
        max_iterations = 5000
        experiment_name = "car_robot"
        resume = False
