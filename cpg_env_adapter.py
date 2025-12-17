#!/usr/bin/env python3
"""
cpg_env_adapter.py

Gym-like environment wrapper for GenesisSwimmingSimulation.
Supports direct joint control and CPG-based control.

Reward objectives:
- Swim along +Y axis (world coordinates)
- Maintain appropriate depth
- Reduce lateral/vertical oscillations
- Control energy consumption
"""

import numpy as np
from typing import Tuple, Optional
from env.swim_gs_env import GenesisSwimmingSimulation
from env.cpg import PhaseOscillatorNetwork, MatsuokaNetwork
from fluid_disturbance import FluidDisturbance


class SwimmingCPGEnv:
    def __init__(
            self,
            control_mode: str = "direct",
            cpg_model_type: int = 0,
            couple: bool = True,
            couple_w_forward: float = 1.0,
            couple_w_backward: float = 1.0,
            learn_phase_offset: bool = True,
            obs_joint_vel: bool = False,
            obs_linear_vel: bool = False,
            obs_phase_diff: bool = False,
            debug: bool = False,
            record: bool = False,
            record_dir: Optional[str] = None,
            target_speed: float = 0.3,
            depth_target: float = 0.5,
            max_steps: int = 200,
            fluid_disturbance: Optional[str] = None,
            disturbance_intensity: float = 1.0,
            cpg_freq: float = 2.0,
            cpg_alpha: float = 1.5,
            cpg_dt: Optional[float] = None,
            res_rl_residual_scale: float = 0.1,
            res_rl_residual_penalty: float = 0.0,
            terminate_on_x_boundary: bool = True,
            x_left_boundary: float = 0.4,
            x_right_boundary: float = 1.8,
            x_boundary_penalty: float = 50.0,
            # NEW: Paddle disturbance parameters
            enable_paddle_disturbance: bool = False,
            paddle_disturbance_config: Optional[dict] = None,
    ):
        """
        Initialize swimming environment.

        Args:
            control_mode: "direct" for RL control, "cpg" for pure CPG, "cpg_rl" for RL-modulated CPG
            cpg_model_type: 0 for PhaseOscillator, 1 for Matsuoka
            couple: Enable CPG oscillator coupling
            couple_w_forward: Initial forward coupling weight (learnable in cpg_rl mode)
            couple_w_backward: Initial backward coupling weight (learnable in cpg_rl mode)
            learn_phase_offset: Whether total_phase_offset is learnable (default: True)
            obs_joint_vel: Add joint velocities (6) to observations (default: False)
            obs_linear_vel: Add linear velocity (3) to observations (default: False)
            obs_phase_diff: Add phase differences (5) to observations (default: False)
            debug: Enable debug output
            record: Enable video recording
            record_dir: Directory to save recordings (default: "./recordings")
            target_speed: Target forward speed (m/s along +Y)
            depth_target: Target depth (z coordinate)
            max_steps: Maximum steps per episode
            fluid_disturbance: Disturbance mode (None, "constant", "turbulent", "vortex", "oscillating", "mixed")
            disturbance_intensity: Disturbance strength multiplier [0, 10]
            cpg_freq: CPG base frequency in Hz (default: 2.0)
            cpg_alpha: CPG coupling strength (default: 1.5)
            cpg_dt: CPG time step (default: None, will use env dt)
            res_rl_residual_scale: Maximum residual scale for res_rl mode (default: 0.1)
            res_rl_residual_penalty: L2 penalty weight on residuals for res_rl mode (default: 0.0)
            terminate_on_x_boundary: Terminate episode when x goes outside boundaries (default: True)
            x_left_boundary: Left boundary for x position (default: -0.3)
            x_right_boundary: Right boundary for x position (default: 0.8)
            x_boundary_penalty: Penalty applied when hitting x boundary (default: 50.0)
            enable_paddle_disturbance: Enable paddle disturbance (default: False)
            paddle_disturbance_config: Paddle disturbance configuration dict (default: None)
        """
        print(f"\n{'='*70}")
        print(f"🔧 SwimmingCPGEnv.__init__() called")
        print(f"{'='*70}")
        print(f"  Control mode: {control_mode}")
        print(f"  CPG model: {cpg_model_type} ({'PhaseOscillator' if cpg_model_type==0 else 'Matsuoka'})")
        print(f"  Debug: {debug}")
        print(f"  Target speed: {target_speed} m/s")
        print(f"  Depth target: {depth_target} m")
        print(f"  Max steps: {max_steps}")
        print(f"  X boundary termination: {'ENABLED' if terminate_on_x_boundary else 'DISABLED'}")
        if terminate_on_x_boundary:
            print(f"    - X boundaries: [{x_left_boundary:.2f}, {x_right_boundary:.2f}]")
            print(f"    - Boundary penalty: {x_boundary_penalty:.1f}")
        
        # NEW: Print paddle disturbance status
        print(f"  Paddle disturbance: {'ENABLED' if enable_paddle_disturbance else 'DISABLED'}")
        if enable_paddle_disturbance and paddle_disturbance_config:
            print(f"    - Config: {paddle_disturbance_config}")

        assert control_mode in ["direct", "cpg", "cpg_rl", "res_rl"]
        self.mode = control_mode
        self.debug = debug
        self.record = record
        self.learn_phase_offset = learn_phase_offset
        self.obs_joint_vel = obs_joint_vel
        self.obs_linear_vel = obs_linear_vel
        self.obs_phase_diff = obs_phase_diff
        self.steps = 0
        self._last_forward_speed = 0.0
        self._last_forward_accel = 0.0


        # NEW: Initialize Genesis simulation with paddle disturbance
        print(f"  Initializing Genesis simulation...")
        self.sim = GenesisSwimmingSimulation(
            vis=True,
            enable_disturbance=enable_paddle_disturbance,
            disturbance_config=paddle_disturbance_config,
            record=record,
            record_dir=record_dir if record_dir else "./recordings"
        )

        # Number of joints
        self.n_joints = len(getattr(self.sim, "dofs_idx", list(range(6))))

        # Observation/action dimensions
        # Base: joints + position(x,y,z)
        self.obs_dim = self.n_joints + 3
        print(f"  Base observation dim: {self.obs_dim} (joints={self.n_joints} + pos=3)")
        if obs_joint_vel:
            self.obs_dim += self.n_joints  # +6 joint velocities
            print(f"  + joint velocities: {self.n_joints}, new total: {self.obs_dim}")
        if obs_linear_vel:
            self.obs_dim += 3  # +3 linear velocity
            print(f"  + linear velocity: 3, new total: {self.obs_dim}")
        if obs_phase_diff:
            self.obs_dim += (self.n_joints - 1)  # +5 phase differences
            print(f"  + phase differences: {self.n_joints - 1}, new total: {self.obs_dim}")
        print(f"  Final observation dimension: {self.obs_dim}")

        # Action dimension depends on control mode
        if control_mode == "cpg_rl":
            if learn_phase_offset:
                self.action_dim = 6  # freq_mod, amp_mod, phase_shift, w_forward, w_backward, total_phase_offset
            else:
                self.action_dim = 5  # freq_mod, amp_mod, phase_shift, w_forward, w_backward
        else:
            self.action_dim = self.n_joints
            print(self.action_dim)
            # import pdb;pdb.set_trace() # direct joint control

        # Time step
        try:
            self.dt = float(self.sim.scene.sim_options.dt)
        except Exception:
            self.dt = 0.05

        # CPG dt should match environment dt if not specified
        if cpg_dt is None:
            cpg_dt = self.dt

        # Target parameters
        self.target_speed = float(target_speed)
        self.depth_target = float(depth_target)
        self.max_steps = int(max_steps)

        # X boundary termination parameters
        self.terminate_on_x_boundary = bool(terminate_on_x_boundary)
        self.x_left_boundary = float(x_left_boundary)
        self.x_right_boundary = float(x_right_boundary)
        self.x_boundary_penalty = float(x_boundary_penalty)

        # State tracking
        self._last_pos = np.zeros(3, dtype=np.float32)
        self._last_joints = np.zeros(self.n_joints, dtype=np.float32)
        self._stepcount = 0
        self.no_move_count = 0
        self._current_total_phase_offset = 2.0 * np.pi  # Default value for cpg_rl mode

        # res_rl mode specific tracking
        self._residual_scale = float(res_rl_residual_scale)
        self._residual_penalty_weight = float(res_rl_residual_penalty)
        self._last_residual = np.zeros(self.n_joints, dtype=np.float32)

        # Initialize CPG if needed
        # print("DOFS IDX =", self.sim.dofs_idx)
        if control_mode in ["cpg", "cpg_rl","res_rl"]:
            print(f"  Initializing CPG network...")
            if cpg_model_type == 0:
                self.cpg = PhaseOscillatorNetwork(
                    num_joints=self.n_joints,
                    alpha=cpg_alpha,
                    # freq=cpg_freq,
                    couple=couple,
                    couple_w_forward=couple_w_forward,
                    couple_w_backward=couple_w_backward,
                    dt=cpg_dt
                )
                print(f"    ✓ PhaseOscillatorNetwork created (n_joints={self.n_joints})")
                # print(f"    ✓ Freq: {cpg_freq} Hz, Alpha: {cpg_alpha}, dt: {cpg_dt}")
                print(f"    ✓ Coupling: {couple}, w_forward={couple_w_forward}, w_backward={couple_w_backward}")
            else:
                self.cpg = MatsuokaNetwork(num_joints=self.n_joints)
                print(f"    ✓ MatsuokaNetwork created (n_joints={self.n_joints})")

            if control_mode == "cpg_rl":
                print(f"    ✓ RL will modulate CPG parameters (freq, amp, phase, coupling)")
        else:
            self.cpg = None
            print(f"  No CPG (using direct RL joint control)")

        # Initialize fluid disturbance (original disturbance system)
        if fluid_disturbance is not None and fluid_disturbance != "none":
            print(f"  Initializing fluid disturbance...")
            self.disturbance = FluidDisturbance(
                mode=fluid_disturbance,
                intensity=disturbance_intensity,
                enable=True,
                randomize=True
            )
            print(f"    ✓ FluidDisturbance created (mode={fluid_disturbance}, intensity={disturbance_intensity})")
            print(f"    ⚠️  WARNING: randomize=True 會導致每個 epoch 結果不同！")
        else:
            self.disturbance = None
            print(f"  ✅ No fluid disturbance (deterministic mode)")

        # Display disturbance status summary
        print(f"\n  📊 Disturbance Status:")
        print(f"    - Fluid disturbance: {'ENABLED (randomize=True)' if self.disturbance else 'DISABLED'}")
        print(f"    - Paddle disturbance: {'ENABLED' if enable_paddle_disturbance else 'DISABLED'}")

        print(f"{'='*70}\n")

    def reset(self):
        """Reset robot to initial pose and simulation state."""

        print(f"\n{'='*70}")
        print(f"🔄 SwimmingCPGEnv.reset() called")
        print(f"{'='*70}")

        # Save and reset fluid velocity data (if recording enabled)
        if self.sim.record and self._stepcount > 0:
            print(f"  Saving fluid velocity data from previous episode...")
            self.sim.save_fluid_velocity_data()
            self.sim.reset_fluid_velocity_buffer()
            print(f"    ✓ Fluid velocity buffer reset")

        # Define spawn position and orientation
        spawn_pos = np.array([1.0, 0.75, self.depth_target], dtype=np.float32)
        spawn_euler = np.array([0, 0, np.pi / 2], dtype=np.float32)  # Yaw=90°, head facing +Y

        print(f"  Target spawn position: [{spawn_pos[0]:.3f}, {spawn_pos[1]:.3f}, {spawn_pos[2]:.3f}]")
        print(f"  Target spawn orientation (euler): [{spawn_euler[0]:.3f}, {spawn_euler[1]:.3f}, {spawn_euler[2]:.3f}] rad")

        # Convert Euler to quaternion (Genesis format: w,x,y,z)
        from scipy.spatial.transform import Rotation as R
        spawn_quat_xyzw = R.from_euler('xyz', spawn_euler).as_quat()  # [x,y,z,w]
        spawn_quat_wxyz = np.array([spawn_quat_xyzw[3], spawn_quat_xyzw[0],
                                    spawn_quat_xyzw[1], spawn_quat_xyzw[2]],
                                   dtype=np.float32)  # [w,x,y,z]

        # Prepare complete qpos: [x,y,z, qw,qx,qy,qz, joint1...joint6]
        # Initialize joints with a sinusoidal wave pattern (total phase span = 2π)
        phase_offsets = np.linspace(0, 2*np.pi, self.n_joints, endpoint=False, dtype=np.float32)
        amplitude = 0.3  # Initial wave amplitude in radians (~17 degrees)
        init_joints = amplitude * np.sin(phase_offsets)

        print(f"  Initial joint pattern (sinusoidal wave):")
        print(f"    - Phase offsets: {phase_offsets}")
        print(f"    - Joint angles: {init_joints}")

        spawn_qpos = np.concatenate([spawn_pos, spawn_quat_wxyz, init_joints])

        print(f"  Resetting robot state...")
        print(f"    - Setting qpos (pos + quat + joints)")
        print(f"    - Zeroing all velocities ({self.sim.robot.n_dofs} DOFs)")

        try:
            # Set complete state using qpos (most reliable method)
            self.sim.robot.set_qpos(spawn_qpos)
            # Also set velocities to zero (important for proper reset)
            self.sim.robot.set_dofs_velocity(
                np.zeros(self.sim.robot.n_dofs, dtype=np.float32),
                np.arange(self.sim.robot.n_dofs)
            )
            self._last_joints = init_joints.copy()
            print(f"    ✓ Robot state reset successful")

        except Exception as e:
            print(f"    ✗ Failed to reset robot pose: {e}")

        # Physics warm-up with repeated pose setting (ensure reset takes effect)
        # Apply initial joint pattern during warm-up to maintain wave shape
        print(f"  Running physics warm-up (30 steps)...")
        for i in range(30):
            # Re-apply qpos and zero velocity frequently
            if i % 3 == 0:  # Every 3 steps
                self.sim.robot.set_qpos(spawn_qpos)
                self.sim.robot.set_dofs_velocity(
                    np.zeros(self.sim.robot.n_dofs, dtype=np.float32),
                    np.arange(self.sim.robot.n_dofs)
                )
            # Apply initial joint pattern to maintain wave shape
            self.sim.robot.control_dofs_position(init_joints, self.sim.dofs_idx)
            # NEW: Use step_simulation() instead of scene.step() to update paddles
            self.sim.step_simulation()
        print(f"    ✓ Warm-up complete")

        # Verify reset success (optional, only in debug mode)
        if self.debug:
            try:
                # Get position and orientation from qpos
                qpos_after = self.sim.robot.get_qpos().cpu().numpy()
                pos_after = qpos_after[0:3].astype(np.float32)
                quat_after = qpos_after[3:7].astype(np.float32)  # [w,x,y,z]

                # Convert quaternion to Euler for display
                from scipy.spatial.transform import Rotation as R
                # scipy expects [x,y,z,w], Genesis uses [w,x,y,z]
                quat_xyzw = np.array([quat_after[1], quat_after[2], quat_after[3], quat_after[0]])
                euler_after = R.from_quat(quat_xyzw).as_euler('xyz')

                print(f"\n{'=' * 60}")
                print(f"Reset verification:")
                print(f"  Target pos:  {spawn_pos}")
                print(f"  Actual pos:  {pos_after}")
                print(f"  Pos error:   {np.linalg.norm(pos_after - spawn_pos):.4f} m")
                print(f"  Target euler: {spawn_euler} rad")
                print(f"  Actual euler: {euler_after} rad")
                print(f"  Euler error:  {np.linalg.norm(euler_after - spawn_euler):.4f} rad")
                if np.linalg.norm(pos_after - spawn_pos) < 0.01:
                    print(f"  ✓ Reset successful")
                else:
                    print(f"  ⚠️  WARNING: Reset may have failed!")
                print(f"{'=' * 60}\n")

            except Exception as e:
                print(f"Warning: Could not verify reset: {e}")

        # Reset logs and counters
        print(f"  Resetting counters and logs...")
        self.sim.joint_angles_log = []
        self.sim.robot_positions_log = []
        self._stepcount = 0
        self.no_move_count = 0
        self._last_forward_speed = 0.0
        self._last_forward_accel = 0.0

        print(f"    ✓ Counters reset (stepcount={self._stepcount}, no_move_count={self.no_move_count})")

        # Reset CPG state
        if self.cpg is not None:
            print(f"  Resetting CPG state...")
            if hasattr(self.cpg, "reset"):
                self.cpg.reset()
                print(f"    ✓ CPG.reset() called")
            elif hasattr(self.cpg, "theta"):
                self.cpg.theta[:] = np.random.uniform(
                    0, 2 * np.pi, size=self.n_joints
                ).astype(np.float32)
                print(f"    ✓ CPG theta randomized")
                                      
        # Reset fluid disturbance (original disturbance system)
        if self.disturbance is not None:
            print(f"  Resetting fluid disturbance...")
            self.disturbance.reset(dt=self.dt)
            info = self.disturbance.get_current_info()
            print(f"    ✓ Disturbance reset: mode={info['mode']}, intensity={info['intensity']:.2f}")

        # Get initial observation
        obs = self._get_obs()
        self._last_pos = obs[self.n_joints:self.n_joints+3].copy()

        print(f"  Initial observation:")
        print(f"    - Joints: {obs[:self.n_joints]}")
        pos_idx = self.n_joints
        print(f"    - Position: [{obs[pos_idx]:.4f}, {obs[pos_idx+1]:.4f}, {obs[pos_idx+2]:.4f}]")
        print(f"{'='*70}\n")

        return obs

    def _get_obs(self):
        """Return current observation: joint angles + robot position + optional extras."""
        # Get joint angles from qpos (most reliable)
        qpos = self.sim.robot.get_qpos().cpu().numpy()
        joints = qpos[7:7+self.n_joints].astype(np.float32)

        # Get robot position from qpos (most reliable)
        pos = qpos[0:3].astype(np.float32)

        # Base observation
        obs_list = [joints, pos]

        # Optional: joint velocities
        if self.obs_joint_vel:
            qvel = self.sim.robot.get_dofs_velocity(self.sim.dofs_idx).cpu().numpy()
            joint_velocities = qvel.astype(np.float32)
            obs_list.append(joint_velocities)

        # Optional: linear velocity [vx, vy, vz]
        if self.obs_linear_vel:
            # Method 1: Try get_vel() which returns (linear_vel, angular_vel)
            try:
                vel_raw = self.sim.robot.get_vel()
                if isinstance(vel_raw, tuple) and len(vel_raw) >= 1:
                    linear_velocity = vel_raw[0].cpu().numpy().flatten().astype(np.float32)
                else:
                    linear_velocity = vel_raw.cpu().numpy().flatten().astype(np.float32)

                # Debug output (only print once)
                if not hasattr(self, '_vel_debug_printed'):
                    print(f"\n🔍 Debug: Linear velocity from get_vel()")
                    print(f"   vel_raw type: {type(vel_raw)}")
                    if isinstance(vel_raw, tuple):
                        print(f"   vel_raw[0] (linear) shape: {vel_raw[0].shape}")
                    print(f"   linear_velocity shape: {linear_velocity.shape}")
                    print(f"   linear_velocity value: {linear_velocity}")
                    self._vel_debug_printed = True

            except Exception as e:
                print(f"⚠️ WARNING: get_vel() failed: {e}")
                # Fallback: compute velocity from position change
                linear_velocity = np.zeros(3, dtype=np.float32)

            # Ensure exactly 3 dimensions [vx, vy, vz]
            if linear_velocity.shape[0] != 3:
                print(f"⚠️ WARNING: linear_velocity has shape {linear_velocity.shape}, expected (3,)")
                if linear_velocity.shape[0] < 3:
                    # Pad with zeros
                    linear_velocity = np.pad(linear_velocity, (0, 3 - linear_velocity.shape[0]), constant_values=0.0)
                    print(f"   Padded to: {linear_velocity}")
                else:
                    # Take first 3 elements
                    linear_velocity = linear_velocity[:3]
                    print(f"   Truncated to: {linear_velocity}")

            obs_list.append(linear_velocity)

        # Optional: phase differences
        if self.obs_phase_diff:
            phase_diffs = np.diff(joints).astype(np.float32)
            obs_list.append(phase_diffs)

        obs = np.concatenate(obs_list).astype(np.float32)

        # Debug: verify observation dimension matches expected
        if obs.shape[0] != self.obs_dim:
            print(f"⚠️  WARNING: Observation dimension mismatch!")
            print(f"    Expected: {self.obs_dim}")
            print(f"    Got: {obs.shape[0]}")
            print(f"    Components: joints={len(obs_list[0])}, pos={len(obs_list[1])}", end="")
            if len(obs_list) > 2:
                for i, comp in enumerate(obs_list[2:], start=2):
                    print(f", extra_{i}={len(comp)}", end="")
            print()

        return obs

    def step(self, action: np.ndarray, sim_steps: int = 1) -> Tuple[np.ndarray, float, bool, dict]:
        """Apply action and step simulation."""
        action = np.array(action, dtype=np.float32)

        # Mode-specific action handling
        if self.mode == "cpg" and self.cpg is not None:
            # Pure CPG: ignore RL action, use CPG output
            joints = self.cpg.step()

        elif self.mode == "cpg_rl" and self.cpg is not None:
            # CPG-RL: RL outputs CPG modulation parameters
            freq_mod = float(np.clip(action[0], -0.5, 0.5))  # ±50% frequency
            # amp_mod = float(np.clip(action[1], -0.5, 0.5))   # ±50% amplitude
            amp_mod = float(np.clip(action[1], -0.3, 2.0))   # ±50% amplitude

            phase_shift = float(np.clip(action[2], -np.pi, np.pi))  # ±π phase
            w_forward = float(np.clip(action[3], 0.0, 6.0))  # Forward coupling weight [0, 3]
            w_backward = float(np.clip(action[4], 0.0, 6.0))  # Backward coupling weight [0, 3]

            if self.learn_phase_offset:
                # action = [freq_mod, amp_mod, phase_shift, w_forward, w_backward, total_phase_offset]
                total_phase_offset = float(np.clip(action[5], 0.0, 4*np.pi))  # Total phase offset [0, 4π]
                self._current_total_phase_offset = total_phase_offset
            else:
                # action = [freq_mod, amp_mod, phase_shift, w_forward, w_backward]
                total_phase_offset = None  # Use CPG's default

            # Get CPG output with modulation
            joints = self.cpg.step(
                freq_mod=freq_mod,
                amp_mod=amp_mod,
                phase_shift=phase_shift,
                w_forward=w_forward,
                w_backward=w_backward,
                total_phase_offset=total_phase_offset
            )
        elif self.mode == "res_rl":
            joints = self.cpg.step()
            # Apply soft constraint with tanh to keep residuals bounded
            residual = self._residual_scale * np.tanh(action)
            joints += residual
            # Store residual for logging
            self._last_residual = residual.copy()
        else:
            # Direct mode: RL directly controls joints
            joints = action
            # print(action[-1])
            # limit = 10 * np.pi / 180

            # # sign(0) = 0, so fix that:
            # sign = np.sign(action)
            # sign[sign == 0] = 1

            # # enforce |action| >= limit
            # action = np.where(np.abs(action) < limit, sign * limit, action)
            if self.steps<1000:

                t = self._stepcount * self.dt        # current simulation time
                freq = 2                           # Hz, adjust as needed
                amp = 1                          # radians, adjust amplitude
                joints[1] = amp*np.sin(2*np.pi*freq*t)

                joints[-1] = amp*np.sin(2*np.pi*freq*t+np.pi/2)


        # Clip to joint limits
        max_ang = 1  # ~45 degrees
        joints = np.clip(joints, -max_ang, max_ang)

        # Apply joint commands
        try:
            if hasattr(self.sim, "apply_joint_actions"):
                self.sim.apply_joint_actions(joints)
            else:
                self.sim.robot.control_dofs_position(joints, self.sim.dofs_idx)
        except Exception as e:
            if self.debug:
                print(f"Warning: Failed to apply joint actions: {e}")

        # Apply fluid disturbance (original disturbance system)
        if self.disturbance is not None:
            self.disturbance.apply_disturbance(self.sim.water)

        # NEW: Step physics using step_simulation() to update paddles
        for _ in range(sim_steps):
            self.sim.step_simulation()

        # Get new observation
        obs = self._get_obs()
        pos = obs[self.n_joints:self.n_joints+3].copy()

        # Compute kinematics
        dpos = pos - self._last_pos
        dt_env = max(1e-6, sim_steps * self.dt)
        vel = dpos / dt_env

        # Forward progress along +Y axis (desired direction)
        forward_disp_y = float(dpos[1])
        forward_speed_y = float(vel[1])

        # Lateral displacement along X axis (unwanted)
        lateral_disp_x = float(dpos[0])
        lateral_disp_y = float(dpos[1])

        # Lateral/vertical speed (should be minimized)
        lateral_speed = float(np.linalg.norm([vel[0], vel[2]]))

        # Depth error
        depth_err = float(pos[2] - self.depth_target)

        # Energy consumption
        joint_delta = joints - self._last_joints
        mean_abs_angle = float(np.mean(np.abs(joints))) if joints.size > 0 else 0.0
        mean_abs_delta = float(np.mean(np.abs(joint_delta))) if joints.size > 0 else 0.0
        energy_term = mean_abs_angle + mean_abs_delta

        # Update no-move counter
        if forward_disp_y <= 0.0:
            self.no_move_count += 1
        else:
            self.no_move_count = 0

        # ========== Reward computation ==========

        # Survival reward
        alive_reward = 0.0008 * self._stepcount

        # Forward displacement reward along +Y (main objective)
        w_forward = 15.0
        forward_reward = w_forward * forward_disp_y

        # Penalize lateral displacement along X (prevent drifting)
        w_lateral = 3.0
        lateral_penalty = -w_lateral * abs(lateral_disp_x - 1.0)

        # Speed shaping (encourage target speed along +Y)
        w_speed = 0.5
        if forward_speed_y < self.target_speed * 0.7:
            speed_shaping = -w_speed * (self.target_speed - forward_speed_y) ** 2
        elif forward_speed_y > self.target_speed * 1.3:
            speed_shaping = -0.3 * w_speed * (forward_speed_y - self.target_speed) ** 2
        else:
            speed_shaping = 0.1  # Small bonus in good range

        # Depth constraint
        w_depth = 2.0
        depth_penalty = -w_depth * (depth_err ** 2)

        # Lateral/vertical stability
        w_stab = 0.5
        vel_stab_penalty = -w_stab * (lateral_speed ** 2)

        # Energy penalty
        w_energy = 1
        energy_penalty = -w_energy * energy_term

        # No-move penalty
        w_no_move = 0.1
        no_move_penalty = -w_no_move * float(self.no_move_count)
        amplitude = 0.6
        phase_offsets = np.linspace(0, np.pi, self.n_joints)
        target_pattern = amplitude * np.sin(phase_offsets + self._stepcount * 0.3)

        # --- Compute state updates ---
        dpos = pos - self._last_pos
        vel = dpos / (sim_steps * self.dt + 1e-6)
        forward_speed = float(vel[1])  # +Y direction

        # --- 1. Forward motion term ---
        # Target speed = desired average swimming speed
        # target_speed = self.target_speed
        target_speed = 0.8
        forward_error = target_speed - forward_speed
        r_forward = -0.05+ np.exp(-5.0 * (forward_error ** 2))

        # --- 2. Phase coordination term ---
        # Adjacent joint angle differences
        

        """
        phase_diff = self.cpg.theta[1:] - self.cpg.theta[0]
        ideal_offsets = np.arange(1, self.n_joints) * (self._current_total_phase_offset / self.n_joints)
        phase_error = np.mean((phase_diff - ideal_offsets) ** 2)
        r_phase = -0.9 + np.exp(-8.0 * phase_error)
        """
        # --- Phase-wave reward (exp version, correct) ---
        # temporal phase extraction
        # phase_diff = np.mod(self.cpg.theta[1:] - self.cpg.theta[0], 2*np.pi)
        # ideal_offsets = np.arange(1, self.n_joints) * (self._current_total_phase_offset / self.n_joints)
        # phase_error = np.mean((phase_diff - ideal_offsets) ** 2)
        # r_phase = -np.exp(-8.0 * phase_error)   # tuned


        # --- 3. Smooth temporal motion term ---
        # Encourage small joint-angle changes (smoothness)
        # Use actual joint angles (not action) for smoothness calculation
        # --- Smoothness reward (jerk-based with exp) ---
        vel_joints = joints - self._last_joints
        if not hasattr(self, '_last_vel'):
            self._last_vel = np.zeros_like(vel_joints)

        jerk = vel_joints - self._last_vel
        smooth_error = float(np.mean(jerk ** 2))
        

        # scaled so that jerk=0 does NOT give max reward
        scaled_err = 5.0 * smooth_error

        r_smooth = -0.9+ np.exp(-scaled_err)

        self._last_vel = vel_joints.copy()


        # --- Combine all components ---
        # reward = (0.5 * r_forward) + (0.3 * r_phase) +0.2*r_smooth
        # reward = r_forward

        # reward = (0.7 * r_forward) + (0.1 * r_phase) +0.2*r_smooth

        # Total reward
        mean_abs_delta = float(np.mean(np.abs(joints - self._last_joints)))
        r_motion = 1.0 * mean_abs_delta

        corr = 0
        for i in range(self.n_joints - 1):
            corr += vel_joints[i] * vel_joints[i+1]
        corr /= (self.n_joints - 1)
        # Normalize correlation to [-1,1] roughly
        corr_error = (1 - corr) * 0.5   # 0 error if corr=1

        r_gait = np.exp(-3.0 * corr_error) - 0.9


        amp = np.abs(vel_joints)
        weights = np.linspace(1.0, 2.0, self.n_joints)  # tail > head
        weighted_amp = np.mean(amp * weights)

        # Ideal envelope amplitude ~0.3–0.6 rad/s → pick target
        target_amp = 0.4
        amp_error = (weighted_amp - target_amp)**2

        r_amp = np.exp(-4.0 * amp_error) - 0.9

        head_vel = abs(vel_joints[-1])   # joint 0 or the first 2 joints if head has multiple
        target_head_motion = 0.05  # small, not big

        head_error = (head_vel - target_head_motion)**2

        r_head = head_vel
        # wave parameters
        t = self._stepcount * self.dt

        # number of joints
        N = self.n_joints

        # wave parameters
        A_target = 0.4
        omega = 2 * np.pi * 2.0    # 2 Hz
        phase_shift = np.pi / 6    # phase difference

        # ideal wave shape for each joint
        i = np.arange(N)
        ideal_wave = A_target * np.sin(omega * t + i * phase_shift)
        A_target = 0.4                       # amplitude
        frequency_hz = 2                  # frequency of oscillation
        omega = 2 * np.pi * frequency_hz     # convert to rad/s
        phase_shift = np.pi / 6              # phase offset per joint

        wave_error = np.mean((joints - ideal_wave)**2)
        r_wave = -0.9+np.exp(-wave_error) 

        theta = joints.copy()  # shape: (n_joints,)
        # Target phase offset between joints
        target_phase = 0.35  # radians, typical for lateral undulation

        # Phase differences between consecutive joints
        phase_diff = theta[1:] - theta[:-1]

        # Reward: high if phase_diff close to target_phase
        phase_error = np.mean((phase_diff - target_phase) ** 2)
        r_phase = -0.9+np.exp(-phase_error) 
                

        
        # reward = (
        # 4.0 * r_forward
        # + 1.0 * r_smooth
        # + 0.7 * r_gait
        # # + r_wave
        # + 0.4 * r_head
        # )
        
        
        
        # res_rl residual penalty (only applied in res_rl mode)
        residual_penalty = 0.0
        if self.mode == "res_rl" and self._residual_penalty_weight > 0:
            residual_penalty = -self._residual_penalty_weight * float(np.mean(self._last_residual ** 2))

        


        ############################################################
        # 2. AMPLITUDE MATCHING (curvature strength)
        ############################################################
        # Typical water snake lateral undulation amplitude ~0.4–0.6 rad
        target_amp = 1
        amp_error = np.mean((np.abs(joints) - target_amp)**2)

        r_amp = -0.9+float(np.exp(-amp_error / 0.1))


        ############################################################
        # 3. SMOOTH WAVE PROPAGATION (reduce jerk, enforce sinusoidal motion)
        ############################################################
        # smooth_error = np.mean(joint_vel**2)

        # r_smooth = -0.9+float(np.exp(-smooth_error / 0.02))
        # reward = r_smooth + r_amp + r_phase + 2* r_forward + 0.3*(-1+pos[1])
        

        forward_disp_y = float(dpos[1])
        r_forward_disp = 20.0 * max(0, forward_disp_y)

        # Lateral displacement from center (x=1.0)
        lateral_disp = float(abs(pos[0] - 1.0))

        x_center = 1.0
        x_deviation = abs(pos[0] - x_center)
        lateral_penalty= 1.5*(1+np.log(1+(0.15-abs(x_deviation))/5))
        # Joint deltas (velocity)
        joint_delta = joints - self._last_joints

        # Magnitude of joint motion (encourage movement)
        motion_mag = np.mean(np.abs(joint_delta))  # mean absolute change

        # Optionally weight joints (tail joints contribute more to propulsion)
        weights = np.linspace(1.0, 2.0, self.n_joints)  # tail > head
        weighted_motion = np.mean(np.abs(joint_delta) * weights)

        # Smooth energy reward (log scaling prevents runaway)
        energy_reward = -1+np.log1p(weighted_motion)  # log1p smooths for small motions

  
        reward = (
                + alive_reward
                + 2* r_forward
                # + 1 * r_smooth
                + 25 * (pos[0] - 1.0) ** 2 # lateral displacement
                + r_forward_disp
                + 0.2* energy_penalty
                )
            
            # print(energy_reward)
            # reward = (
            # #         # alive_reward
                    #4* r_forward
                    #  + 1 * r_smooth
                    #  + 0.1*lateral_penalty
                    # -0.001*self.steps
                   # + 0.8 * abs(-1+pos[1])
                  #  + r_amp
                    #  -3*abs(0.7-pos[0])**2
                 #   + r_phase
                # + energy_reward
        # #         # + speed_shaping
        # #         # + depth_penalty
        # #         # + vel_stab_penalty
                # -0.2*energy_penalty
                #  + no_move_penalty
                #  + residual_penalty  # Add residual penalty for res_rl mode
        # )
        
        # --- acceleration ---
        accel = (forward_speed - self._last_forward_speed) / dt_env

        # --- jerk (change in acceleration) ---
        jerk_forward = accel - self._last_forward_accel

        # penalize jerk
        reward -= 0.1*abs(jerk_forward)

        

        # ========== Termination conditions ==========
        self._stepcount += 1
        self.steps += 1
        done = False

        # Out of bounds (Y direction)
        if pos[1] < -0.3:
            reward -= 500
            done = True

        elif pos[1] > 2.0:
            reward += 100
            done = True    # optional

        # Out of bounds (X direction) - NEW
        if self.terminate_on_x_boundary:
            if pos[0] < self.x_left_boundary or pos[0] > self.x_right_boundary:
                done = True
                reward -= self.x_boundary_penalty
                if self.debug:
                    print(f"⚠️  X boundary violation: x={pos[0]:.3f}, bounds=[{self.x_left_boundary:.3f}, {self.x_right_boundary:.3f}]")

        # elif pos[1] > 1:
        #     reward += 1
        # print(energy_penalty)
        # print(r_smooth)

        # Stuck for too long
        if self.no_move_count > 300:
            done = True
            reward -= 2000.0

        # Max steps reached
        if self._stepcount >= self.max_steps:
            done = True

        # ========== Update state ==========
        self._last_pos = pos.copy()
        self._last_joints = joints.copy()
        self._last_forward_speed = forward_speed
        self._last_forward_accel = accel

        # ========== Info dictionary ==========
        info = {
            "pos": pos,
            "dpos": dpos,
            "forward_disp_y": forward_disp_y,
            "lateral_disp_x": lateral_disp_x,
            "lateral_disp": lateral_disp,  # NEW: lateral displacement from center (x=1.0)
            "forward_speed": forward_speed_y,
            "lateral_speed": lateral_speed,
            "depth_err": depth_err,
            "reward_components": {
                "alive": alive_reward,
                "forward": forward_reward,
                "lateral_pen": lateral_penalty,
                "speed_shape": speed_shaping,
                "depth_pen": depth_penalty,
                "vel_stab_pen": vel_stab_penalty,
                "energy_pen": energy_penalty,
                "no_move_pen": no_move_penalty,
            },
            "reward_terms": {
                "r_forward": float(r_forward),
                # "r_phase": float(r_phase),
                "r_smooth": float(r_smooth),
                "r_gait": float(r_gait),
                "r_amp": float(r_amp),
                "r_head": float(r_head),
                "r_wave": float(r_wave),
                "lateral_penalty": float(lateral_penalty),
                "residual_penalty": float(residual_penalty),
                "energy_reward": float(energy_reward),  # NEW: energy reward from joint motion
                "r_forward_disp": float(r_forward_disp),  # NEW: forward displacement reward
            },
            "residual_stats": {
                "residual_mean": float(np.mean(np.abs(self._last_residual))),
                "residual_max": float(np.max(np.abs(self._last_residual))),
                "residual_std": float(np.std(self._last_residual)),
            },
            "head_position": {
                "x": float(pos[ 0]),
                "y": float(pos[1]),
                "z": float(pos[2]),
            },
            "no_move_count": self.no_move_count,
            "step": self._stepcount,
        }

        # Add phase data for CPG modes
        if self.cpg is not None and hasattr(self.cpg, 'theta'):
            info["phase_data"] = {
                "phases": self.cpg.theta.copy().tolist(),  # Phase angles for all joints
                "phase_mean": float(np.mean(self.cpg.theta)),
                "phase_std": float(np.std(self.cpg.theta)),
            }

        print(info)

        return obs, float(reward), done, info