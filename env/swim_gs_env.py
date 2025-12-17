#!/usr/bin/env python3
"""
Genesis swimming simulation with sine-wave joint trajectories (class-based)
"""

import os
import argparse
import numpy as np
import genesis as gs
import hashlib


class OscillatingPaddleDisturbance:
    """振荡桨叶扰动器 - Z轴上下运动产生波浪"""

    def __init__(self, scene, n_paddles=10, water_bounds=None, swim_zone=None,
                 oscillation_freq=3.5, oscillation_amplitude=0.10,
                 paddle_size=(0.55, 0.25, 0.1), paddle_mass=3000.0, seed=42):
        """
        Args:
            scene: Genesis场景对象（必须在scene.build()之前传入）
            n_paddles: 桨叶数量
            water_bounds: 水域边界 (min_x, max_x, min_y, max_y, min_z, max_z)
            swim_zone: X轴游泳区域 (min_x, max_x) - paddle不会放置在这里
            oscillation_freq: 振荡频率 (Hz)
            oscillation_amplitude: 振荡幅度 (m)
            paddle_size: 桨叶尺寸
            paddle_mass: 桨叶密度
            seed: 随机种子
        """
        if seed is not None:
            np.random.seed(seed)

        self.scene = scene
        self.n_paddles = n_paddles
        self.water_bounds = water_bounds if water_bounds else (0.0, 2.0, 0.0, 2.0, 0.0, 0.4)
        self.swim_zone = swim_zone if swim_zone else (0.3, 1.5)
        self.oscillation_freq = oscillation_freq
        self.oscillation_amplitude = oscillation_amplitude
        self.paddle_size = paddle_size
        self.paddle_mass = paddle_mass
        self.paddles = []
        self.enabled = True

        # 定义左右两侧的X轴区域（避开游泳区域）
        swim_min_x, swim_max_x = self.swim_zone
        left_x_range = (self.water_bounds[0] + 0.2, swim_min_x - 0.15)
        right_x_range = (swim_max_x + 0.15, self.water_bounds[1] - 0.2)

        # 将Y轴分成若干段
        y_total = self.water_bounds[3] - self.water_bounds[2]
        n_y_segments = max(2, (n_paddles + 1) // 2)
        y_segment_size = y_total / n_y_segments

        # 为每个paddle生成参数
        self.paddle_params = []
        for i in range(n_paddles):
            # 交替左右放置
            if i % 2 == 0:
                x_range = left_x_range
            else:
                x_range = right_x_range

            # 分配到不同的Y段
            y_segment_idx = i // 2
            if y_segment_idx < n_y_segments:
                y_start = self.water_bounds[2] + y_segment_idx * y_segment_size + 0.15
                y_end = self.water_bounds[2] + (y_segment_idx + 1) * y_segment_size - 0.15
            else:
                y_start = self.water_bounds[2] + 0.2
                y_end = self.water_bounds[3] - 0.2

            x_pos = np.random.uniform(x_range[0], x_range[1])
            y_pos = np.random.uniform(y_start, y_end)
            z_pos = (self.water_bounds[4] + self.water_bounds[5]) / 2

            params = {
                'center_x': x_pos,
                'center_y': y_pos,
                'center_z': z_pos,
                'freq': oscillation_freq * np.random.uniform(0.8, 1.2),
                'amplitude': oscillation_amplitude * np.random.uniform(0.8, 1.2),
                'phase': np.random.uniform(0, 2 * np.pi),
            }
            self.paddle_params.append(params)

        print(f"\n[OscillatingPaddleDisturbance] Configured with {n_paddles} paddles")
        print(f"  Paddle size: {paddle_size}")
        print(f"  Oscillation: freq={oscillation_freq}Hz, amp={oscillation_amplitude}m, direction=Z")
        print(f"  Swim zone (X-axis excluded): x ∈ [{swim_min_x:.2f}, {swim_max_x:.2f}]")

    def add_paddles_to_scene(self):
        """在scene.build()之前调用此方法添加paddle实体"""
        for i, params in enumerate(self.paddle_params):
            paddle = self.scene.add_entity(
                material=gs.materials.Rigid(rho=self.paddle_mass),
                morph=gs.morphs.Box(
                    pos=(params['center_x'], params['center_y'], params['center_z']),
                    size=self.paddle_size,
                    euler=(0.0, 0.0, 0.0),
                ),
                surface=gs.surfaces.Default(color=(0.9, 0.3, 0.1, 0.9)),
            )
            self.paddles.append(paddle)

        print(f"[OscillatingPaddleDisturbance] Added {len(self.paddles)} paddles to scene")

    def update(self, current_time):
        """每个仿真步调用此方法更新paddle位置"""
        if not self.enabled or len(self.paddles) == 0:
            return

        for i, paddle in enumerate(self.paddles):
            params = self.paddle_params[i]

            # 计算Z轴振荡偏移
            offset = params['amplitude'] * np.sin(
                2 * np.pi * params['freq'] * current_time + params['phase']
            )

            # 新位置（只在Z轴移动）
            new_pos = np.array([
                params['center_x'],
                params['center_y'],
                params['center_z'] + offset
            ])

            try:
                paddle.set_pos(new_pos)
            except AttributeError:
                pass  # 如果set_pos不存在，忽略

    def enable(self):
        """启用扰动"""
        self.enabled = True

    def disable(self):
        """禁用扰动"""
        self.enabled = False


class GenesisSwimmingSimulation:
    """Genesis swimming simulation for eel robot with optional paddle disturbance."""

    def __init__(self, vis=True, enable_disturbance=False, disturbance_config=None,
                 record=False, record_dir=None):
        """
        Args:
            vis: 是否启用可视化
            enable_disturbance: 是否启用paddle扰动
            disturbance_config: paddle扰动配置字典，例如:
                {
                    'n_paddles': 6,
                    'oscillation_freq': 1.5,
                    'oscillation_amplitude': 0.10,
                    'paddle_size': (0.15, 0.15, 0.05),
                    'paddle_mass': 2000.0,
                    'swim_zone': (0.6, 0.9),
                }
            record: 是否录制视频和流体速度数据
            record_dir: 录制文件保存目录 (默认为 "./recordings")
        """
        self.vis = vis
        self.enable_disturbance = enable_disturbance
        self.record = record
        self.record_dir = record_dir if record_dir else "./recordings"
        self.SIM_DT = 0.05
        self.SUBSTEPS = 100
        self.PARTICLE_SIZE = 0.03
        self.WATER_BOUNDS = (0.0, 0.0, 0.0), (2.0, 2.0, 1.0)
        self.SPH_BOUNDS = (0.0, 0.0, 0.0), (2.0, 2.0, 1.0)

        # 用于追踪仿真时间
        self.sim_time = 0.0

        # 流体速度数据缓冲区
        self.fluid_velocity_buffer = [] if record else None

        # 创建录制目录
        if self.record:
            os.makedirs(self.record_dir, exist_ok=True)

        # Initialize Genesis
        gs.init(backend=gs.cuda, seed=0, precision="32", logging_level="info")

        # Setup video recording path if enabled
        if self.record:
            import time
            timestamp = time.strftime("%Y%m%d_%H%M%S")
            video_filename = f"swim_recording_{timestamp}.mp4"
            self.video_path = os.path.join(self.record_dir, video_filename)
            print(f"[INFO] Video recording will be saved to: {self.video_path}")
        else:
            self.video_path = None

        # Build scene
        self.scene = gs.Scene(
            sim_options=gs.options.SimOptions(dt=self.SIM_DT, substeps=self.SUBSTEPS, gravity=(0,0,-9.81)),
            mpm_options=gs.options.MPMOptions(lower_bound=self.WATER_BOUNDS[0], upper_bound=self.WATER_BOUNDS[1]),
            sph_options=gs.options.SPHOptions(lower_bound=self.SPH_BOUNDS[0], upper_bound=self.SPH_BOUNDS[1], particle_size=self.PARTICLE_SIZE),
            vis_options=gs.options.VisOptions(visualize_sph_boundary=False, visualize_mpm_boundary=False),
            viewer_options=gs.options.ViewerOptions(camera_pos=(1.0,1.0,5.0), camera_lookat=(1.0,1.0,0.0), camera_fov=40, max_FPS=200),
            show_viewer=self.vis
        )

        # Add ground plane
        self.scene.add_entity(gs.morphs.Plane())

        # Add water (store reference for potential future use)
        self.water = self.scene.add_entity(
            material=gs.materials.SPH.Liquid(rho=1000.0, mu=0.0001, sampler="regular"),
            morph=gs.morphs.Box(pos=(1.0,1.0,0.2), size=(2.0,2.0,0.4), fixed=False),
            surface=gs.surfaces.Default(color=(0.2,0.6,1.0,0.05))
        )

        # Load eel robot
        # Use relative path from repository root
        script_dir = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
        urdf_path = os.path.join(script_dir, "urdf", "eelrobotv2_urdf.urdf")
        self.robot = self.scene.add_entity(
            material=gs.materials.Rigid(needs_coup=True, coup_friction=0.1),
            morph=gs.morphs.URDF(file=urdf_path, pos=(1.0,1.0,0.5), euler=(0,0,0), fixed=False)
        )

        # Initialize paddle disturbance (before scene.build())
        self.disturbance = None
        if enable_disturbance:
            print(f"\n{'='*70}")
            print(f"🌊 Initializing Paddle Disturbance")
            print(f"{'='*70}")
            
            # 默认配置
            default_config = {
                'n_paddles': 10,
                'oscillation_freq': 1.5,
                'oscillation_amplitude': 0.10,
                'paddle_size': (0.2, 0.2, 0.1),
                'paddle_mass': 3000.0,
                'swim_zone': (0.6, 0.9),
            }
            
            # 合并用户配置
            if disturbance_config:
                default_config.update(disturbance_config)
            
            self.disturbance = OscillatingPaddleDisturbance(
                scene=self.scene,
                water_bounds=(0.0, 2.0, 0.0, 2.0, 0.0, 0.4),
                **default_config
            )
            
            # 添加paddle到场景（必须在build之前）
            self.disturbance.add_paddles_to_scene()
            print(f"{'='*70}\n")

        # Add recording camera if enabled (MUST be before scene.build())
        self.camera = None
        if self.record and self.video_path:
            print("[INFO] Adding recording camera...")
            self.camera = self.scene.add_camera(
                res=(1280, 960),
                pos=(1.0, 1.0, 5.0),
                lookat=(1.0, 1.0, 0.0),
                fov=40,
                GUI=False  # No display window
            )
            print(f"[INFO] Recording camera added")

        # Build scene (MUST be after adding all entities including cameras)
        print("[INFO] Building scene...")
        self.scene.build()
        # self.scene.build(n_envs=2)

        print("[INFO] Scene built successfully")

        # Start camera recording if enabled
        if self.record and self.camera:
            self.camera.start_recording()
            self.flow_frame_dir = os.path.join(self.record_dir, "flow_frames")
            os.makedirs(self.flow_frame_dir, exist_ok=True)
            self.frame_id = 0
            print(f"[INFO] Camera recording started (will save to: {self.video_path})")
        else:
            self.flow_frame_dir = None
            print("[INFO] Video recording disabled")



        # Warm-up for stability
        warmup_steps = int(1.0 / self.SIM_DT)
        print(f"[INFO] Running warm-up ({warmup_steps} steps)...")
        for step in range(warmup_steps):
            if self.disturbance:
                self.disturbance.update(self.sim_time)
            self.scene.step()
            self.sim_time += self.SIM_DT
        print("[INFO] Warm-up complete")

        # Joint info
        self.joint_names = ["joint6","joint5","joint4","joint3","joint2","joint1"]
        self.dofs_idx = [self.robot.get_joint(name).dof_idx_local for name in self.joint_names]

    def get_base_xyz(self):
        link = self.robot.get_link("base_link")
        return np.array(link.get_pos().cpu().numpy())

    def step_simulation(self):
        """执行一步仿真（包括更新paddle）"""
        # 更新paddle位置
        if self.disturbance:
            self.disturbance.update(self.sim_time)

        # 执行物理仿真
        self.scene.step()

        # Render frame for video recording (if enabled)
        if self.record and self.camera:
            self.camera.render()

        # 采集流体速度数据（如果启用录制）
        if self.record and self.fluid_velocity_buffer is not None:
            self._collect_fluid_velocity_data()
            self._save_fluid_speed_frame()

        # 更新仿真时间
        self.sim_time += self.SIM_DT

    def _collect_fluid_velocity_data(self):
        """采集当前帧的SPH流体粒子速度数据"""
        try:
            # 获取SPH粒子的速度
            # Genesis中SPH粒子数据存储在entity的state中
            particle_velocities = self.water.get_state().vel.cpu().numpy()
            particle_positions = self.water.get_state().pos.cpu().numpy()

            # 计算统计信息以减少数据量
            frame_data = {
                "time": float(self.sim_time),
                "step": len(self.fluid_velocity_buffer),
                "num_particles": len(particle_velocities),
                # 速度统计
                "vel_mean": particle_velocities.mean(axis=0).tolist(),
                "vel_std": particle_velocities.std(axis=0).tolist(),
                "vel_max": particle_velocities.max(axis=0).tolist(),
                "vel_min": particle_velocities.min(axis=0).tolist(),
                "speed_mean": float(np.linalg.norm(particle_velocities, axis=1).mean()),
                "speed_max": float(np.linalg.norm(particle_velocities, axis=1).max()),
                # 位置统计（用于空间分析）
                "pos_mean": particle_positions.mean(axis=0).tolist(),
                "pos_std": particle_positions.std(axis=0).tolist(),
            }

            self.fluid_velocity_buffer.append(frame_data)
        except Exception as e:
            # 如果采集失败，记录警告但不中断仿真
            if len(self.fluid_velocity_buffer) == 0:
                print(f"[WARNING] Failed to collect fluid velocity data: {e}")


    def _save_fluid_speed_frame(self):
        """Save a CFD-like top-view velocity field with stable log scale, Gaussian smoothing, and quiver arrows."""
        if not self.record:
            return

        try:
            import matplotlib
            matplotlib.use('Agg')  # Use non-interactive backend to avoid Tkinter threading issues
            import matplotlib.pyplot as plt
            from scipy.interpolate import griddata
            from scipy.ndimage import gaussian_filter
            import numpy as np
            import os

            state = self.water.get_state()
            pos = state.pos.cpu().numpy()
            vel = state.vel.cpu().numpy()

            surface_mask = (pos[:, 2] > 0.1) & (pos[:, 2] < 0.3)
            pos = pos[surface_mask]
            vel = vel[surface_mask]

            if len(pos) == 0:
                return

            x = pos[:, 0]
            y = pos[:, 1]
            u = vel[:, 0]
            v = vel[:, 1]
            speed = np.sqrt(u**2 + v**2)

            # Log scale
            log_speed = np.log(1.0 + speed) / np.log(1.05)

            # Create regular grid
            grid_x, grid_y = np.mgrid[x.min():x.max():200j, y.min():y.max():200j]

            # Interpolate speed and velocity
            grid_speed = griddata((x, y), log_speed, (grid_x, grid_y), method='cubic')
            grid_u = griddata((x, y), u, (grid_x, grid_y), method='cubic')
            grid_v = griddata((x, y), v, (grid_x, grid_y), method='cubic')

            # Gaussian smoothing
            grid_speed = gaussian_filter(np.nan_to_num(grid_speed), sigma=1)
            grid_u = gaussian_filter(np.nan_to_num(grid_u), sigma=1)
            grid_v = gaussian_filter(np.nan_to_num(grid_v), sigma=1)

            self._flow_vmax = 8 # max： log（1+7.22）/ log(1.2) = 11.55, mean： log（1+0.39）/log（1.2）=1.8
            norm_speed = np.clip(grid_speed / (self._flow_vmax + 1e-8), 0, 1)

            # Plot heatmap
            fig, ax = plt.subplots(figsize=(6, 6))
            im = ax.imshow(norm_speed.T, origin='lower',
                        extent=(x.min(), x.max(), y.min(), y.max()),
                        cmap='jet', alpha=0.9,
                        vmin=0, vmax=1)

            # Quiver arrows colored by local speed
            # local_speed = np.sqrt(grid_u**2 + grid_v**2)
            # local_speed = gaussian_filter(local_speed, sigma=1)
            # quiver_colors = plt.cm.jet(np.clip(local_speed / (self._flow_vmax + 1e-8), 0, 1))
            # step = 8
            # quiver_colors_list = quiver_colors[::step, ::step].reshape(-1, 4)

            # ax.quiver(
            #     grid_x[::step, ::step], grid_y[::step, ::step],
            #     grid_u[::step, ::step].T, grid_v[::step, ::step].T,
            #     color=quiver_colors_list,
            #     pivot='middle',
            #     scale=20
            # )

            ax.set_axis_off()
            plt.margins(0, 0)
            plt.subplots_adjust(0, 0, 1, 1)

            frame_path = os.path.join(self.flow_frame_dir, f"frame_{self.frame_id:06d}.png")
            plt.savefig(frame_path, dpi=150, bbox_inches='tight', pad_inches=0)
            plt.close()
            self.frame_id += 1

        except Exception as e:
            print(f"[WARNING] Failed to save surface speed frame: {e}")




    def save_fluid_velocity_data(self, filename=None):
        """保存流体速度数据到JSON文件"""
        if not self.record or self.fluid_velocity_buffer is None:
            return

        if len(self.fluid_velocity_buffer) == 0:
            print("[WARNING] No fluid velocity data to save")
            return

        if filename is None:
            import time
            timestamp = time.strftime("%Y%m%d_%H%M%S")
            filename = f"fluid_velocity_{timestamp}.json"

        output_path = os.path.join(self.record_dir, filename)

        import json
        with open(output_path, 'w') as f:
            json.dump({
                "metadata": {
                    "sim_dt": self.SIM_DT,
                    "particle_size": self.PARTICLE_SIZE,
                    "total_frames": len(self.fluid_velocity_buffer),
                    "total_time": self.sim_time,
                },
                "frames": self.fluid_velocity_buffer
            }, f, indent=2)

        print(f"[INFO] Fluid velocity data saved: {output_path}")
        return output_path

    def reset_fluid_velocity_buffer(self):
        """重置流体速度数据缓冲区（用于新的episode）"""
        if self.record:
            self.fluid_velocity_buffer = []

    def stop_recording(self):
        """Stop camera recording & generate fluid speed video."""
        if self.record and self.camera and self.video_path:
            fps = int(1.0 / self.SIM_DT)
            self.camera.stop_recording(save_to_filename=self.video_path, fps=fps)
            print(f"[INFO] Video recording saved to: {self.video_path}")

        # Build fluid-speed MP4 video
        if self.record and self.flow_frame_dir:
            try:
                import imageio.v2 as imageio
                flow_video_path = os.path.join(self.record_dir, "fluid_speed.mp4")

                print("[INFO] Generating fluid-speed MP4...")

                frames = sorted(
                    [os.path.join(self.flow_frame_dir, f)
                     for f in os.listdir(self.flow_frame_dir)
                     if f.endswith(".png")]
                )

                # Write video
                writer = imageio.get_writer(flow_video_path, fps=int(1.0/self.SIM_DT))
                for f in frames:
                    writer.append_data(imageio.imread(f))
                writer.close()

                print(f"[INFO] Fluid speed video saved: {flow_video_path}")

            except Exception as e:
                print(f"[ERROR] Failed to generate fluid-speed video: {e}")

    def run(self, duration=3.0, freq=1.0, amp_deg=45.0):
        dt = self.SIM_DT
        link_num = 7
        num_joint = link_num - 1
        amplitude = np.deg2rad(amp_deg)
        bias = np.zeros(num_joint)
        joint_phases = np.linspace(1 / link_num, (link_num - 1) / link_num, num_joint)

        # Generate control trajectory
        T_control = np.arange(0.0, duration, dt)
        u_traj = [amplitude * np.sin(2*np.pi*(freq*t + joint_phases)) + bias for t in T_control]

        trajectory_xyz = []

        # Main loop
        for qpos in u_traj:
            self.robot.control_dofs_position(qpos, self.dofs_idx)
            self.step_simulation()  # 使用新的step方法
            trajectory_xyz.append(self.get_base_xyz())

        trajectory_xyz = np.array(trajectory_xyz)

        # Save trajectory
        save_path = os.path.expanduser("~/robot_xyz_trajectory.npy")
        np.save(save_path, trajectory_xyz)
        print(f"[INFO] Trajectory saved at: {save_path}")

        # Print stats (保持原有的信息输出)
        self._print_stats()

    def _print_stats(self):
        """打印仿真统计信息"""
        try:
            print("Genesis version:", gs.__version__)
        except:
            pass
        
        print("=== Scene Integrator ===")
        print("dt(actual):", self.scene.sim_options.dt)
        print("substeps(actual):", self.scene.sim_options.substeps)
        
        if self.disturbance:
            print("\n=== Paddle Disturbance ===")
            print(f"Status: ENABLED")
            print(f"Paddles: {self.disturbance.n_paddles}")
            print(f"Total simulation time: {self.sim_time:.2f}s")


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("-v", "--vis", action="store_true", help="Enable visualization")
    parser.add_argument("--disturbance", action="store_true", help="Enable paddle disturbance")
    args = parser.parse_args()

    # config paddle disturbance
    disturbance_config = {
        'n_paddles': 6,
        'oscillation_freq': 1.5,
        'oscillation_amplitude': 0.10,
        'paddle_size': (0.15, 0.15, 0.05),
        'paddle_mass': 2000.0,
        'swim_zone': (0.6, 0.9),
    }

    sim = GenesisSwimmingSimulation(
        vis=args.vis, 
        enable_disturbance=args.disturbance,
        disturbance_config=disturbance_config if args.disturbance else None
    )
    sim.run(duration=3.0, freq=1.0, amp_deg=45.0)