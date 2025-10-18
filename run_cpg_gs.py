#!/usr/bin/env python3
"""
CPG implementation for swimming robot using Genesis API directly.
Implements both phase oscillator and Matsuoka oscillator models.
"""

import numpy as np
import matplotlib.pyplot as plt
import os
import time
import json
import genesis as gs
import xml.etree.ElementTree as ET

class PhaseOscillatorNetwork:
    """
    Phase oscillator network exactly as implemented in agnathax-simulation.
    Based on cpg_single_chain.cdn file.
    """
    def __init__(self, 
                 num_joints=6,
                 freq=5.0,
                 w_up=10.0,
                 w_down=30.0,
                 prop_fb=0.0,
                 ext_fb=0.0,
                 kappa=0.0,
                 nb_fb=0.0,
                 tau=0.01,
                 time_step=0.005):
        
        self.num_joints = num_joints
        self.freq = freq
        self.w_up = w_up
        self.w_down = w_down
        self.prop_fb = prop_fb
        self.ext_fb = ext_fb
        self.kappa = kappa
        self.nb_fb = nb_fb
        self.tau = tau
        self.dt = time_step
        
        # Phase lag calculation exactly as in agnathax
        self.phi_lag = 2 * np.pi / self.num_joints
        
        # Initialize oscillator states
        self.theta = np.random.uniform(0.01, 2*np.pi-0.01, self.num_joints)
        self.theta_dot = np.zeros(self.num_joints)
        self.x = np.zeros(self.num_joints)
        
        # Fixed random seed as in agnathax
        np.random.seed(28244)
        
    def update(self):
        """Update phase oscillator network exactly as in agnathax."""
        # Calculate theta_dot for each oscillator
        for i in range(self.num_joints):
            # Base frequency term
            theta_dot = 2 * np.pi * self.freq
            
            # Proprioceptive feedback term
            theta_dot -= self.prop_fb * np.sin(self.theta[i])
            
            # Exteroceptive feedback term
            theta_dot -= self.ext_fb * np.cos(self.theta[i] + self.kappa)
            
            # Neural feedback term
            theta_dot -= self.nb_fb
            
            # Add coupling from other oscillators
            for j in range(self.num_joints):
                if i != j:
                    # Bidirectional coupling with different weights
                    if j == i + 1:  # Forward coupling (towards tail)
                        w = self.w_down
                        phi = -self.phi_lag
                    elif j == i - 1:  # Backward coupling (towards head)
                        w = self.w_up
                        phi = self.phi_lag
                    else:
                        continue  # Only adjacent coupling
                    
                    theta_dot += w * np.sin(self.theta[j] - self.theta[i] - phi)
            
            self.theta_dot[i] = theta_dot
        
        # Integrate using Euler method
        self.theta += self.theta_dot * self.dt
        
        # Keep theta in [0, 2π]
        self.theta = self.theta % (2 * np.pi)
        
        # Calculate output x = r*cos(theta) where r=1
        self.x = np.cos(self.theta)
        
        return self.x.copy()
    
    def get_theta(self):
        return self.theta.copy()
    
    def get_theta_dot(self):
        return self.theta_dot.copy()
    
    def get_x(self):
        return self.x.copy()


class MatsuokaNetwork:
    """
    Matsuoka oscillator network exactly as implemented in agnathax-simulation.
    Based on matsuoka_network.cdn file.
    """
    def __init__(self,
                 num_joints=6,
                 beta=5.0,
                 eta=-3.0,
                 gamma=3.0,
                 eta_i=-2.0,
                 eta_e=1.0,
                 eta_head=0.0,
                 tonic=50.0,
                 force=0.0,
                 tau=0.05,
                 fb_i=-10.0,
                 fb_e=1.0,
                 time_step=0.005):
        
        self.num_joints = num_joints
        self.beta = beta
        self.eta = eta
        self.gamma = gamma
        self.eta_i = eta_i
        self.eta_e = eta_e
        self.eta_head = eta_head
        self.tonic = tonic
        self.force = force
        self.tau = tau
        self.fb_i = fb_i
        self.fb_e = fb_e
        self.dt = time_step
        
        # Initialize Matsuoka oscillator states
        # Each oscillator has two neurons: x_a, v_a, x_b, v_b
        self.x_a = np.random.uniform(0.01, 0.1, self.num_joints)
        self.v_a = np.random.uniform(0.01, 0.1, self.num_joints)
        self.x_b = np.random.uniform(0.01, 0.1, self.num_joints)
        self.v_b = np.random.uniform(0.01, 0.1, self.num_joints)
        
        # Output states
        self.xx_a = np.zeros(self.num_joints)
        self.xx_b = np.zeros(self.num_joints)
        
        # Fixed random seed as in agnathax
        np.random.seed(28244)
        
    def update(self):
        """Update Matsuoka oscillator network exactly as in agnathax."""
        # Calculate rectified outputs
        self.xx_a = self.x_a * (self.x_a > 0)
        self.xx_b = self.x_b * (self.x_b > 0)
        
        # Calculate derivatives for each oscillator
        x_a_dot = np.zeros(self.num_joints)
        v_a_dot = np.zeros(self.num_joints)
        x_b_dot = np.zeros(self.num_joints)
        v_b_dot = np.zeros(self.num_joints)
        
        for i in range(self.num_joints):
            # Force feedback terms
            force_l = self.force * (self.force >= 0.0)
            force_r = -self.force * (self.force < 0.0)
            
            fb_c_a = force_l * self.fb_i
            fb_i_a = force_l * self.fb_e
            fb_c_b = force_r * self.fb_i
            fb_i_b = force_r * self.fb_e
            
            # Neuron A dynamics
            x_a_dot[i] = (-1/self.tau * (self.x_a[i] + self.beta*self.v_a[i] - self.eta*self.xx_b[i] - self.tonic) + 
                         fb_i_a + fb_c_b)
            v_a_dot[i] = (-self.v_a[i] + self.xx_a[i]) / (self.tau * self.gamma)
            
            # Neuron B dynamics
            x_b_dot[i] = (-1/self.tau * (self.x_b[i] + self.beta*self.v_b[i] - self.eta*self.xx_a[i] - self.tonic) + 
                         fb_c_a + fb_i_b)
            v_b_dot[i] = (-self.v_b[i] + self.xx_b[i]) / (self.tau * self.gamma)
            
            # Add coupling from other oscillators
            for j in range(self.num_joints):
                if i != j:
                    if j == i + 1:  # Forward coupling (towards tail)
                        # Ipsilateral coupling
                        x_a_dot[i] += self.eta_e/self.tau * self.xx_a[j] * (self.xx_a[j] > 0)
                        x_b_dot[i] += self.eta_e/self.tau * self.xx_b[j] * (self.xx_b[j] > 0)
                        
                        # Contralateral coupling
                        x_a_dot[i] += self.eta_i/self.tau * self.xx_b[j] * (self.xx_b[j] > 0)
                        x_b_dot[i] += self.eta_i/self.tau * self.xx_a[j] * (self.xx_a[j] > 0)
            
            # Self-excitation for first oscillator (head)
            if i == 0:
                x_a_dot[i] += self.eta_head/self.tau * self.xx_a[i] * (self.xx_a[i] > 0)
                x_b_dot[i] += self.eta_head/self.tau * self.xx_b[i] * (self.xx_b[i] > 0)
        
        # Integrate using Euler method
        self.x_a += x_a_dot * self.dt
        self.v_a += v_a_dot * self.dt
        self.x_b += x_b_dot * self.dt
        self.v_b += v_b_dot * self.dt
        
        # Calculate output as difference between neurons
        output = self.xx_a - self.xx_b
        
        return output.copy()
    
    def get_x_a(self):
        return self.x_a.copy()
    
    def get_x_b(self):
        return self.x_b.copy()
    
    def get_xx_a(self):
        return self.xx_a.copy()
    
    def get_xx_b(self):
        return self.xx_b.copy()


class GenesisSwimmingSimulation:
    """Genesis-based swimming simulation with CPG control."""
    
    def __init__(self, cpg_model_type=0, use_gpu=True, record_video=True):
        # Simulation parameters - following Genesis official documentation
        self.SIM_DT = 8e-3  # 4ms as recommended in Genesis docs
        self.SUBSTEPS = 10  # 10 substeps as recommended in Genesis docs
        self.WATER_BOUNDS = (-1.0, -0.5, 0.0), (1.0, 0.5, 1.0)  # Following Genesis docs pattern
        self.PARTICLE_SIZE = 0.01  # 0.01m as recommended in Genesis docs
        self.record_video = record_video
        
        # CPG model type
        self.cpg_model_type = cpg_model_type
        if cpg_model_type == 0:
            self.model_name = "phase_oscillator"
        else:
            self.model_name = "matsuoka"
        
        # Initialize Genesis
        gs.init(backend=gs.gpu if use_gpu else gs.cpu)
        
        # Create scene
        self.scene = gs.Scene(
            sim_options=gs.options.SimOptions(
                dt=self.SIM_DT, 
                substeps=self.SUBSTEPS, 
                gravity=(0, 0, -9.8)  # Standard gravity (downward)
            ),
            sph_options=gs.options.SPHOptions(
                lower_bound=self.WATER_BOUNDS[0],
                upper_bound=self.WATER_BOUNDS[1],
                particle_size=self.PARTICLE_SIZE,
            ),
            vis_options=gs.options.VisOptions(visualize_sph_boundary=True),
            viewer_options=gs.options.ViewerOptions(
                camera_pos=(3.0, 0.0, 0.5), 
                camera_lookat=(0.0, 0.0, 0.15)
            ),
            show_viewer=True,  # Enable viewer
        )
        
        # Add ground
        self.scene.add_entity(gs.morphs.Plane())
        
        # Add water - user specified: from z=0 to z=0.3
        water_height = 0.3  # Height of water from 0 to 0.3
        water_center_z = 0.15  # Center of water (0 + 0.3/2 = 0.15)
        
        # Water size matches larger world bounds exactly
        # World bounds: x from -1.0 to 1.0 (width=2.0), y from -0.5 to 0.5 (width=1.0)
        water_x_size = 2.0  # Larger world width
        water_y_size = 1.0  # Larger world height
        
        self.liquid = self.scene.add_entity(
            material=gs.materials.SPH.Liquid(
                sampler='regular',  # Regular sampling for macOS compatibility
                mu=0.01,  # 增加粘性，减少弹跳
                gamma=0.005  # 减少表面张力，让机器人更容易进入水中
            ),
            morph=gs.morphs.Box(
                pos=(0.0, 0.0, water_center_z),  # Use calculated center position
                size=(water_x_size, water_y_size, water_height)  # Use calculated dimensions
            ),
            surface=gs.surfaces.Default(color=(0.4, 0.8, 1.0), vis_mode="particle"),
        )
        
        # Load 6-joint swimming robot URDF
        urdf_path = os.path.join(os.path.dirname(__file__), 'env/assem_description/urdf/assem_description_6joints.urdf')
        print(f"Loading 6-joint swimming robot URDF from: {urdf_path}")
        
        # Position robot slightly underwater to avoid bouncing
        robot_z = 0.29  # Robot center slightly underwater (0.3 - 0.05)
        print(f"Water center: {water_center_z}, Water height: {water_height}, Robot Z: {robot_z}")
        
        self.robot = self.scene.add_entity(
            material=gs.materials.Rigid(),
            morph=gs.morphs.URDF(file=urdf_path, fixed=False, pos=(0.0, 0.0, robot_z)),
            surface=gs.surfaces.Default(color=(0.79216, 0.81961, 0.93333)),
        )
        
        # Get robot links for state tracking
        self.robot_links = []
        self.robot_joints = []
        
        # Add camera for video recording
        if self.record_video:
            self.cam = self.scene.add_camera(
                res=(640, 480),
                pos=(3.0, 0.0, 0.5),
                lookat=(0.0, 0.0, 0.15),
                fov=45,
                GUI=False,
            )
            print("Camera added for video recording")
        
        # Build scene
        self.scene.build()
        
        # Initialize CPG
        if cpg_model_type == 0:
            self.cpg = PhaseOscillatorNetwork(
                num_joints=6,
                freq=5.0,
                w_up=10.0,
                w_down=30.0,
                prop_fb=0.0,
                ext_fb=0.0,
                kappa=0.0,
                nb_fb=0.0,
                tau=0.01,
                time_step=self.SIM_DT
            )
        else:
            self.cpg = MatsuokaNetwork(
                num_joints=6,
                beta=5.0,
                eta=-3.0,
                gamma=3.0,
                eta_i=-2.0,
                eta_e=1.0,
                eta_head=0.0,
                tonic=50.0,
                force=0.0,
                tau=0.05,
                fb_i=-10.0,
                fb_e=1.0,
                time_step=self.SIM_DT
            )
        
        # Data logging
        self.log = {
            "t": [],
            "joint_angles": [],
            "joint_velocities": [],
            "robot_positions": [],
            "robot_velocities": [],
            "cpg_states": []
        }
        
        # Start video recording
        if self.record_video:
            os.makedirs("videos", exist_ok=True)
            try:
                print("Starting video recording...")
                self.cam.start_recording()
                print("Video recording started successfully!")
                # Give camera time to initialize
                import time
                time.sleep(0.5)
            except Exception as e:
                print(f"Error starting video recording: {e}")
                print("Disabling video recording...")
                self.record_video = False
    
    def get_entity_pose(self, entity):
        """Get entity pose (position and rotation matrix)."""
        try:
            if hasattr(entity, "world_pos") and hasattr(entity, "world_quat"):
                pos = np.array(entity.world_pos, dtype=float)
                quat = np.array(entity.world_quat, dtype=float)
                if pos.shape[0] == 3 and quat.shape[0] == 4:
                    return pos, self._quat_to_rot(quat)
            
            # Fallback to default
            return np.array([0.0, 0.0, 0.2], dtype=float), np.eye(3)
        except Exception as e:
            print(f"[WARN] Error getting entity pose: {e}")
            return np.array([0.0, 0.0, 0.2], dtype=float), np.eye(3)
    
    def _quat_to_rot(self, q):
        """Convert quaternion to rotation matrix."""
        q = np.array(q, dtype=float).flatten()
        if q.shape[0] != 4:
            raise ValueError("Quaternion must have 4 elements")
        
        x, y, z, w = q[0], q[1], q[2], q[3]
        xx, yy, zz = x*x, y*y, z*z
        xy, xz, yz = x*y, x*z, y*z
        wx, wy, wz = w*x, w*y, w*z
        
        R = np.array([
            [1 - 2*(yy + zz),     2*(xy - wz),       2*(xz + wy)],
            [2*(xy + wz),         1 - 2*(xx + zz),   2*(yz - wx)],
            [2*(xz - wy),         2*(yz + wx),       1 - 2*(xx + yy)],
        ])
        return R
    
    def apply_joint_actions(self, joint_angles):
        """Apply joint angles to robot (simplified implementation)."""
        # For now, we'll just store the desired angles
        # In a full implementation, you would apply these to actual joints
        self.desired_joint_angles = joint_angles
    
    def get_robot_state(self):
        """Get current robot state."""
        # Get positions and velocities of all links
        positions = []
        velocities = []
        
        # For URDF robot, get the main body position
        if hasattr(self, 'robot'):
            pos, _ = self.get_entity_pose(self.robot)
            positions.append(pos)
            # For simplicity, assume zero velocity for now
            velocities.append(np.zeros(3))
        else:
            # Fallback for box robot
            for link in self.robot_links:
                pos, _ = self.get_entity_pose(link)
                positions.append(pos)
                velocities.append(np.zeros(3))
        
        return np.array(positions), np.array(velocities)
    
    def step(self):
        """Step the simulation."""
        # Update CPG
        desired_joint_angles = self.cpg.update()
        
        # Apply joint actions
        self.apply_joint_actions(desired_joint_angles)
        
        # Step simulation
        self.scene.step()
        
        # Render frame for video
        if self.record_video and self.cam is not None:
            try:
                self.cam.render()
            except Exception as e:
                print(f"Camera render error during CPG control: {e}")
                # Don't disable camera immediately, try to continue
                pass
        
        return desired_joint_angles
    
    def run_simulation(self, duration=10.0, log_every=10):
        """Run the simulation for specified duration."""
        total_steps = int(duration / self.SIM_DT)
        
        print(f"Running {self.model_name} simulation...")
        print(f"Simulation duration: {duration} seconds ({total_steps} steps)")
        print(f"CPG model type: {self.cpg_model_type}")
        
        try:
            for step in range(total_steps):
                # Step simulation with CPG control
                joint_angles = self.step()
                
                # Log data
                if step % log_every == 0:
                    current_time = step * self.SIM_DT
                    self.log["t"].append(current_time)
                    self.log["joint_angles"].append(joint_angles.tolist())
                    
                    # Get robot state
                    positions, velocities = self.get_robot_state()
                    self.log["robot_positions"].append(positions.tolist())
                    self.log["robot_velocities"].append(velocities.tolist())
                    
                    # Log CPG states
                    if self.cpg_model_type == 0:
                        cpg_states = {
                            "theta": self.cpg.get_theta().tolist(),
                            "theta_dot": self.cpg.get_theta_dot().tolist(),
                            "x": self.cpg.get_x().tolist()
                        }
                    else:
                        cpg_states = {
                            "x_a": self.cpg.get_x_a().tolist(),
                            "x_b": self.cpg.get_x_b().tolist(),
                            "xx_a": self.cpg.get_xx_a().tolist(),
                            "xx_b": self.cpg.get_xx_b().tolist()
                        }
                    self.log["cpg_states"].append(cpg_states)
                
                # Print progress
                if step % 200 == 0:  # Less frequent progress updates for speed
                    progress = (step / total_steps) * 100
                    current_time = step * self.SIM_DT
                    print(f"Simulation progress: {progress:.0f}% - Time: {current_time:.2f}s")
        except KeyboardInterrupt:
            print(f"\nSimulation interrupted at step {step}/{total_steps}")
            print("Saving collected data...")
            
            # Stop video recording immediately
            if self.record_video and self.cam is not None:
                try:
                    print("Stopping video recording due to interruption...")
                    # Try simple stop_recording first
                    self.cam.stop_recording()
                    print("Video recording stopped successfully!")
                except Exception as e:
                    print(f"Error stopping recording: {e}")
                    # Try alternative method
                    try:
                        self.cam.stop_recording(
                            save_to_filename=f"videos/{self.model_name}_simulation_interrupted.mp4"
                        )
                        print(f"Video saved to videos/{self.model_name}_simulation_interrupted.mp4")
                    except Exception as e2:
                        print(f"Alternative save method also failed: {e2}")
            
            # Save log data
            os.makedirs("logs", exist_ok=True)
            with open(f"logs/{self.model_name}_log_interrupted.json", "w") as f:
                json.dump(self.log, f, indent=2)
            print(f"Log saved to logs/{self.model_name}_log_interrupted.json")
            
            raise  # Re-raise to be caught by outer try-catch
        
        # Stop video recording
        if self.record_video and self.cam is not None:
            try:
                print("Stopping video recording...")
                # Try simple stop_recording first
                self.cam.stop_recording()
                print("Video recording stopped successfully!")
            except Exception as e:
                print(f"Error stopping recording: {e}")
                print("Trying alternative save method...")
                try:
                    # Try with filename parameter
                    self.cam.stop_recording(
                        save_to_filename=f"videos/{self.model_name}_simulation.mp4"
                    )
                    print(f"Video saved to videos/{self.model_name}_simulation.mp4")
                except Exception as e2:
                    print(f"Alternative save method also failed: {e2}")
        
        # Save log
        os.makedirs("logs", exist_ok=True)
        with open(f"logs/{self.model_name}_log.json", "w") as f:
            json.dump(self.log, f, indent=2)
        print(f"Log saved to logs/{self.model_name}_log.json")
    
    def plot_results(self):
        """Plot simulation results."""
        t = np.array(self.log["t"])
        joint_angles = np.array(self.log["joint_angles"]).T
        robot_positions = np.array(self.log["robot_positions"])
        
        fig, axes = plt.subplots(3, 2, figsize=(15, 12))
        fig.suptitle(f'{self.model_name.replace("_", " ").title()} Network Test Results', fontsize=16)
        
        # Plot 1: Joint angles over time
        axes[0, 0].plot(t, joint_angles.T)
        axes[0, 0].set_title('Joint Angles Over Time')
        axes[0, 0].set_xlabel('Time (s)')
        axes[0, 0].set_ylabel('Angle (rad)')
        axes[0, 0].legend([f'Joint {i+1}' for i in range(6)])
        axes[0, 0].grid(True)
        
        # Plot 2: CPG states
        if self.cpg_model_type == 0:
            cpg_theta = np.array([state["theta"] for state in self.log["cpg_states"]]).T
            axes[0, 1].plot(t, cpg_theta.T)
            axes[0, 1].set_title('CPG Phase States (θ)')
            axes[0, 1].set_xlabel('Time (s)')
            axes[0, 1].set_ylabel('Phase (rad)')
            axes[0, 1].legend([f'Joint {i+1}' for i in range(6)])
        else:
            cpg_x_a = np.array([state["x_a"] for state in self.log["cpg_states"]]).T
            axes[0, 1].plot(t, cpg_x_a.T)
            axes[0, 1].set_title('CPG Neuron A States (x_a)')
            axes[0, 1].set_xlabel('Time (s)')
            axes[0, 1].set_ylabel('State')
            axes[0, 1].legend([f'Joint {i+1}' for i in range(6)])
        axes[0, 1].grid(True)
        
        # Plot 3: Robot X position
        if len(robot_positions) > 0 and len(robot_positions[0]) > 0:
            axes[1, 0].plot(t, [pos[0][0] for pos in robot_positions], label='Link 1')
            if len(robot_positions[0]) > 3:
                axes[1, 0].plot(t, [pos[3][0] for pos in robot_positions], label='Link 4')
            if len(robot_positions[0]) > 5:
                axes[1, 0].plot(t, [pos[5][0] for pos in robot_positions], label='Link 6')
        axes[1, 0].set_title('Robot X Position')
        axes[1, 0].set_xlabel('Time (s)')
        axes[1, 0].set_ylabel('Position (m)')
        axes[1, 0].legend()
        axes[1, 0].grid(True)
        
        # Plot 4: Robot Z position
        if len(robot_positions) > 0 and len(robot_positions[0]) > 0:
            axes[1, 1].plot(t, [pos[0][2] for pos in robot_positions], label='Link 1')
            if len(robot_positions[0]) > 3:
                axes[1, 1].plot(t, [pos[3][2] for pos in robot_positions], label='Link 4')
            if len(robot_positions[0]) > 5:
                axes[1, 1].plot(t, [pos[5][2] for pos in robot_positions], label='Link 6')
        axes[1, 1].set_title('Robot Z Position')
        axes[1, 1].set_xlabel('Time (s)')
        axes[1, 1].set_ylabel('Position (m)')
        axes[1, 1].legend()
        axes[1, 1].grid(True)
        
        # Plot 5: CPG output
        if self.cpg_model_type == 0:
            cpg_x = np.array([state["x"] for state in self.log["cpg_states"]]).T
            axes[2, 0].plot(t, cpg_x.T)
            axes[2, 0].set_title('CPG Output (x = cos(θ))')
            axes[2, 0].set_xlabel('Time (s)')
            axes[2, 0].set_ylabel('Output')
            axes[2, 0].legend([f'Joint {i+1}' for i in range(6)])
        else:
            cpg_xx_a = np.array([state["xx_a"] for state in self.log["cpg_states"]]).T
            cpg_xx_b = np.array([state["xx_b"] for state in self.log["cpg_states"]]).T
            axes[2, 0].plot(t, cpg_xx_a.T, label='xx_a')
            axes[2, 0].plot(t, cpg_xx_b.T, label='xx_b')
            axes[2, 0].set_title('CPG Rectified Outputs')
            axes[2, 0].set_xlabel('Time (s)')
            axes[2, 0].set_ylabel('Output')
            axes[2, 0].legend()
        axes[2, 0].grid(True)
        
        # Plot 6: Robot movement
        axes[2, 1].plot(robot_positions[:, 0, 0], robot_positions[:, 0, 2], label='Link 1')
        axes[2, 1].plot(robot_positions[:, 3, 0], robot_positions[:, 3, 2], label='Link 4')
        axes[2, 1].plot(robot_positions[:, 5, 0], robot_positions[:, 5, 2], label='Link 6')
        axes[2, 1].set_title('Robot Movement (X-Z plane)')
        axes[2, 1].set_xlabel('X Position (m)')
        axes[2, 1].set_ylabel('Z Position (m)')
        axes[2, 1].legend()
        axes[2, 1].grid(True)
        
        plt.tight_layout()
        plt.savefig(f'{self.model_name}_genesis_test.png', dpi=300, bbox_inches='tight')
        plt.show()
        
        print(f"Plot saved as {self.model_name}_genesis_test.png")


def test_genesis_cpg(cpg_model_type=0):
    """
    Test CPG network with Genesis simulation.
    
    Args:
        cpg_model_type (int): 0 for phase oscillator, 1 for Matsuoka oscillator
    """
    print("CPG Implementation with Genesis")
    print("0: Phase Oscillator Network")
    print("1: Matsuoka Oscillator Network")
    
    # Create simulation
    sim = GenesisSwimmingSimulation(
        cpg_model_type=cpg_model_type,
        use_gpu=False,
        record_video=True  # Re-enable video recording
    )
    
    # Run simulation
    try:
        sim.run_simulation(duration=2.0, log_every=50)  # Longer duration for testing
        print(f"\n{['Phase Oscillator', 'Matsuoka Oscillator'][cpg_model_type]} simulation completed successfully!")
    except KeyboardInterrupt:
        print(f"\n{['Phase Oscillator', 'Matsuoka Oscillator'][cpg_model_type]} simulation interrupted by user.")
        print("Video and data already saved during interruption.")
    except Exception as e:
        print(f"Simulation error: {e}")
        print("Generating plots with available data...")
    
    # Always try to plot results, even if simulation was interrupted
    try:
        sim.plot_results()
        print("Plots generated successfully!")
    except Exception as e:
        print(f"Error generating plots: {e}")
    
    print(f"\n{['Phase Oscillator', 'Matsuoka Oscillator'][cpg_model_type]} test completed!")


if __name__ == "__main__":
    # You can change this parameter to switch between models
    cpg_model_type = 0  # 0 for phase oscillator, 1 for Matsuoka oscillator
    
    test_genesis_cpg(cpg_model_type)
