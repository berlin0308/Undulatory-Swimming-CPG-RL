#!/usr/bin/env python3
"""
Genesis-based swimming simulation with CPG control
"""

import os
import sys
import time
import numpy as np
import matplotlib.pyplot as plt
import json
from datetime import datetime

# Add the current directory to the path to import env modules
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

try:
    import genesis as gs
    print("✓ Genesis imported successfully")
except ImportError as e:
    print(f"✗ Failed to import Genesis: {e}")
    print("Please install Genesis: pip install genesis-world")
    sys.exit(1)

from .cpg import PhaseOscillatorNetwork, MatsuokaNetwork

class GenesisSwimmingSimulation:
    """Genesis-based swimming simulation with CPG control."""
    
    def __init__(self, cpg_model_type=0, use_gpu=True, record_video=True, debug_mode=False):
        # Simulation parameters - adjusted for water stability
        self.SIM_DT = 8e-3  # 8ms time step (smaller for stability)
        self.SUBSTEPS = 10  # 10 substeps (more for stability)
        
        # Adjust particle size based on debug mode
        if debug_mode:
            self.PARTICLE_SIZE = 0.04
        else:
            self.PARTICLE_SIZE = 0.01
        
        # Adjust world bounds based on debug mode
        if debug_mode:
            # Debug mode: larger X,Y dimensions, lower Z height
            self.WATER_BOUNDS = (-2.0, -1.0, 0.0), (2.0, 1.0, 0.4)  # Larger X,Y, lower Z
        else:
            # Normal mode: original dimensions
            self.WATER_BOUNDS = (-1.0, -0.5, 0.0), (1.0, 0.5, 1.0)  # Original dimensions
        
        self.record_video = record_video
        self.debug_mode = debug_mode
        
        # CPG model type
        self.cpg_model_type = cpg_model_type
        if cpg_model_type == 0:
            self.model_name = "phase_oscillator"
        else:
            self.model_name = "matsuoka"
        
        print(f"Initializing Genesis swimming simulation with {self.model_name} CPG...")
        print(f"Debug mode: {debug_mode}")
        print(f"Record video: {record_video}")
        
        # Initialize Genesis first
        gs.init(backend=gs.gpu if use_gpu else gs.cpu)
        print("✓ Genesis initialized")
        
        # Create scene with options (like run_cpg_gs_old.py)
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
                camera_pos=(5.0, 0.0, 0.5) if not debug_mode else (0.0, 0.0, 5.0),  # Camera further away
                camera_lookat=(0.0, 0.0, 0.15) if not debug_mode else (0.0, 0.0, 0.05)  # Debug mode looks at water bottom
            ),
            show_viewer=True,  # Enable viewer
        )
        
        # Add ground
        self.scene.add_entity(gs.morphs.Plane())
        
        # Add water - adjust size based on debug mode
        if debug_mode:
            # Debug mode: larger X,Y dimensions, lower Z height
            water_height = 0.4  # Water height
            water_center_z = 0.2  # Water center (0 + 0.4/2 = 0.2)
            water_x_size = 4.0  # X-axis size (from -2.0 to 2.0)
            water_y_size = 2.0  # Y-axis size (from -1.0 to 1.0)
        else:
            # Normal mode: original dimensions
            water_height = 1.0  # Height of water
            water_center_z = 0.5  # Center of water
            water_x_size = 2.0  # X-axis size
            water_y_size = 1.0  # Y-axis size
        
        self.liquid = self.scene.add_entity(
            material=gs.materials.SPH.Liquid(
                sampler='regular',  # Regular sampling for macOS compatibility
                mu=0.01,  # Viscosity - lower value for more stable water
                gamma=0.02,  # Surface tension - higher value to prevent collapse
            ),
            morph=gs.morphs.Box(
                pos=(0.0, 0.0, water_center_z),  # Use calculated center position
                size=(water_x_size, water_y_size, water_height)  # Use calculated dimensions
            ),
            surface=gs.surfaces.Default(color=(0.4, 0.8, 1.0, 0.05), vis_mode="particle"),  # Almost transparent (Alpha=0.1)
        )
        
        # Load 6-joint swimming robot URDF
        urdf_path = os.path.join(os.path.dirname(os.path.dirname(__file__)), 'assem_description/urdf/assem_description_6joints.urdf')
        print(f"Loading 6-joint swimming robot URDF from: {urdf_path}")
        
        # Position robot - debug mode puts robot at bottom, normal mode slightly underwater
        if debug_mode:
            robot_z = 0.1  # Robot at bottom of water (near ground level)
            print(f"Debug mode: Robot positioned at water bottom")
        else:
            robot_z = 0.29  # Robot center slightly underwater (0.3 - 0.05)
            print(f"Normal mode: Robot positioned slightly underwater")
        
        # Load robot using the working method from run_cpg_gs_old.py
        try:
            # Method 1: Standard URDF loading
            self.robot = self.scene.add_entity(
                material=gs.materials.Rigid(),
                morph=gs.morphs.URDF(file=urdf_path, fixed=False, pos=(0.0, 0.0, robot_z)),
                surface=gs.surfaces.Default(color=(0.79216, 0.81961, 0.93333)),
            )
            print("Loaded robot using standard URDF method")
        except Exception as e:
            print(f"Standard URDF loading failed: {e}")
            try:
                # Method 2: Try with joint control enabled
                self.robot = self.scene.add_entity(
                    material=gs.materials.Rigid(),
                    morph=gs.morphs.URDF(file=urdf_path, fixed=False, pos=(0.0, 0.0, robot_z), enable_joint_control=True),
                    surface=gs.surfaces.Default(color=(0.79216, 0.81961, 0.93333)),
                )
                print("Loaded robot with joint control enabled")
            except Exception as e2:
                print(f"URDF with joint control failed: {e2}")
                try:
                    # Method 3: Try with different material
                    self.robot = self.scene.add_entity(
                        material=gs.materials.Rigid(enable_joint_control=True),
                        morph=gs.morphs.URDF(file=urdf_path, fixed=False, pos=(0.0, 0.0, robot_z)),
                        surface=gs.surfaces.Default(color=(0.79216, 0.81961, 0.93333)),
                    )
                    print("Loaded robot with joint control in material")
                except Exception as e3:
                    print(f"All robot loading methods failed: {e3}")
                    raise
        
        # Try to enable joint control after loading (like run_cpg_gs_old.py)
        try:
            # Method: Enable joint control after robot is loaded
            if hasattr(self.robot, 'enable_joint_control'):
                self.robot.enable_joint_control(True)
                print("Enabled joint control after loading")
            elif hasattr(self.robot, 'set_joint_control_enabled'):
                self.robot.set_joint_control_enabled(True)
                print("Set joint control enabled after loading")
            
            # Try to set joint control parameters
            if hasattr(self.robot, 'set_dofs_kp'):
                # Set high stiffness for all joints
                kp_values = np.array([1000.0] * 7)  # High stiffness
                self.robot.set_dofs_kp(kp_values)
                print("Set high stiffness for all joints")
            
            if hasattr(self.robot, 'set_dofs_kv'):
                # Set high damping for all joints
                kv_values = np.array([100.0] * 7)  # High damping
                self.robot.set_dofs_kv(kv_values)
                print("Set high damping for all joints")
                
        except Exception as enable_error:
            print(f"Failed to enable joint control: {enable_error}")
        
        # Set up joint control according to Genesis documentation
        try:
            # Get joint names and DOF indices according to Genesis documentation
            self.joint_names = []
            self.dofs_idx = []
            
            # Get joint names from robot
            for i in range(self.robot.n_joints):
                joint = self.robot.get_joint(i)
                if hasattr(joint, 'name'):
                    self.joint_names.append(joint.name)
                    # Get local DOF index for each joint
                    if hasattr(joint, 'dof_idx_local'):
                        self.dofs_idx.append(joint.dof_idx_local)
                    else:
                        self.dofs_idx.append(i)  # Fallback to joint index
                else:
                    self.joint_names.append(f"joint_{i}")
                    self.dofs_idx.append(i)
            
            print(f"Joint names: {self.joint_names}")
            print(f"DOF indices: {self.dofs_idx}")
            
        except Exception as e:
            print(f"Failed to set up joint control: {e}")
            # Fallback: use simple indices
            self.dofs_idx = list(range(7))  # Assume 7 joints
            self.joint_names = [f"joint_{i}" for i in range(7)]
        
        # Initialize CPG network
        if cpg_model_type == 0:
            self.cpg = PhaseOscillatorNetwork(num_joints=7, freq=5.0)  # 7 joints, 5Hz frequency
        else:
            self.cpg = MatsuokaNetwork(num_joints=7, tonic=50.0)  # 7 joints, 50Hz tonic input
        
        print(f"✓ CPG network initialized: {self.model_name}")
        
        # Test CPG output
        test_output = self.cpg.step()
        print(f"✓ CPG test output: {test_output[:3]}... (first 3 joints)")
        
        # Add camera for video recording (like run_cpg_gs_old.py)
        if self.record_video:
            # Debug mode camera looks down at water bottom, normal mode keeps original view
            if debug_mode:
                cam_pos = (0.0, 0.0, 2.5)  # Look from above, further away
                cam_lookat = (0.0, 0.0, 0.05)  # Look at water bottom
                print("Debug mode: Camera positioned above for top-down view of robot at water bottom")
            else:
                cam_pos = (5.0, 0.0, 0.5)  # Camera further away
                cam_lookat = (0.0, 0.0, 0.15)  # Original view
            
            self.cam = self.scene.add_camera(
                res=(640, 480),
                pos=cam_pos,
                lookat=cam_lookat,
                fov=45,
                GUI=False,  # Disable GUI for video recording
            )
            print("Camera added for video recording")
        
        # Build scene (like run_cpg_gs_old.py)
        self.scene.build()
        
        # Try to enable joint control if needed
        try:
            # Check if robot needs joint control to be enabled
            if hasattr(self.robot, 'enable_joint_control'):
                self.robot.enable_joint_control(True)
                print("Enabled joint control for robot")
            elif hasattr(self.robot, 'set_joint_control_enabled'):
                self.robot.set_joint_control_enabled(True)
                print("Set joint control enabled for robot")
        except Exception as e:
            print(f"Could not enable joint control: {e}")
        
        # Start video recording (exactly like run_cpg_gs_old.py)
        if self.record_video:
            os.makedirs("videos", exist_ok=True)
            try:
                print("Starting video recording...")
                self.cam.start_recording()
                print("Video recording started successfully!")
                # Give camera time to initialize
                # import time
                # time.sleep(0.5)
            except Exception as e:
                print(f"Error starting video recording: {e}")
                print("Disabling video recording...")
                self.record_video = False
        
        # Initialize data logging
        self.joint_angles_log = []
        self.robot_positions_log = []
        self.cpg_output_log = []
        self.step_count = 0
        
        print("✓ Genesis swimming simulation initialized successfully!")
    
    def apply_joint_actions(self, joint_angles):
        """Apply joint angles to robot using Genesis official method."""
        # Store desired angles
        self.desired_joint_angles = joint_angles
        
        # Debug: Print joint angles occasionally
        if not hasattr(self, 'joint_debug_count'):
            self.joint_debug_count = 0
        self.joint_debug_count += 1
        
        if self.joint_debug_count % 10 == 0:  # Print every 10 steps for debugging
            print(f"Step {self.joint_debug_count}: Joint angles to apply: {joint_angles}")
        
        # Check for nan values in joint angles to prevent instability
        if np.any(np.isnan(joint_angles)) or np.any(np.isinf(joint_angles)):
            if self.joint_debug_count % 10 == 0:
                print(f"  ⚠️  Warning: Invalid joint angles detected (nan/inf), skipping control")
            return
        
        # Use Genesis official set_dofs_position method (hard reset) - This is the working method
        try:
            if hasattr(self.robot, 'set_dofs_position') and hasattr(self, 'dofs_idx'):
                self.robot.set_dofs_position(
                    joint_angles,
                    self.dofs_idx
                )
                if self.joint_debug_count % 10 == 0:
                    print(f"  ✓ Used robot.set_dofs_position (hard reset)")
                return  # Success, exit early
        except Exception as e:
            if self.joint_debug_count % 10 == 0:
                print(f"  ✗ set_dofs_position failed: {e}")
        
        # Fallback: If set_dofs_position fails, try other methods
        if self.joint_debug_count % 10 == 0:
            print(f"  ⚠️  Primary method failed, trying fallback methods...")
        
        # Fallback method 1: Try robot-level joint control
        try:
            if hasattr(self.robot, 'set_joint_positions'):
                self.robot.set_joint_positions(joint_angles)
                if self.joint_debug_count % 10 == 0:
                    print(f"  ✓ Used robot.set_joint_positions fallback")
                return
        except Exception as e:
            if self.joint_debug_count % 10 == 0:
                print(f"  ✗ robot.set_joint_positions fallback failed: {e}")
        
        # Fallback method 2: Try scene-level joint control
        try:
            if hasattr(self.scene, 'set_joint_positions'):
                self.scene.set_joint_positions(self.robot, joint_angles)
                if self.joint_debug_count % 10 == 0:
                    print(f"  ✓ Used scene.set_joint_positions fallback")
                return
        except Exception as e:
            if self.joint_debug_count % 10 == 0:
                print(f"  ✗ scene.set_joint_positions fallback failed: {e}")
        
        # If all methods fail
        if self.joint_debug_count % 10 == 0:
            print(f"  ✗ All joint control methods failed")
    
    def step(self):
        """Step the simulation."""
        import time
        
        # Record start time for FPS calculation
        step_start_time = time.time()
        
        # Step CPG network
        desired_joint_angles = self.cpg.step()
        
        # Check if CPG returned valid output
        if desired_joint_angles is None:
            print(f"Warning: CPG returned None, using zeros")
            desired_joint_angles = np.zeros(7)
        
        # Log CPG output
        self.cpg_output_log.append(desired_joint_angles.copy())
        
        # Debug: Print CPG output occasionally
        if self.step_count % 25 == 0:  # Print every 25 steps
            print(f"Step {self.step_count}: CPG output = {desired_joint_angles}")
        
        # Apply joint actions
        self.apply_joint_actions(desired_joint_angles)
        
        # Verify joint control is working by checking actual joint positions
        if self.step_count % 25 == 0:  # Check every 25 steps for more frequent verification
            try:
                current_qpos = self.robot.get_qpos()
                if current_qpos is not None:
                    # Handle Tensor object
                    if hasattr(current_qpos, 'numpy'):
                        qpos_array = current_qpos.numpy()
                    elif hasattr(current_qpos, 'tolist'):
                        qpos_array = np.array(current_qpos.tolist())
                    else:
                        qpos_array = np.array(list(current_qpos))
                    
                    # Check for nan values in qpos
                    if np.any(np.isnan(qpos_array)) or np.any(np.isinf(qpos_array)):
                        print(f"Step {self.step_count}: ⚠️  CRITICAL: qpos contains nan/inf values!")
                        print(f"  qpos array: {qpos_array}")
                        print(f"  This indicates serious physics instability!")
                        # Try to reset robot to prevent further instability
                        try:
                            print(f"  🔄 Attempting to reset robot position...")
                            self.robot.set_qpos(np.zeros_like(qpos_array))
                        except Exception as reset_error:
                            print(f"  ✗ Robot reset failed: {reset_error}")
                        return
                    
                    print(f"Step {self.step_count}: Joint verification:")
                    print(f"  Desired angles shape: {desired_joint_angles.shape}, values: {desired_joint_angles}")
                    print(f"  Actual qpos shape: {qpos_array.shape}, values: {qpos_array}")
                    
                    # Extract joint angles from qpos (assuming first 7 values are joint angles)
                    if qpos_array.shape[0] >= 7:
                        actual_joint_angles = qpos_array[:7]  # First 7 values are joint angles
                        print(f"  Actual joint angles: {actual_joint_angles}")
                        print(f"  Difference: {np.abs(desired_joint_angles - actual_joint_angles)}")
                    else:
                        print(f"  ⚠️  qpos array too small: {qpos_array.shape[0]} < 7")
                    
                    # Check if joints are actually moving
                    if not hasattr(self, 'prev_joint_angles'):
                        if qpos_array.shape[0] >= 7:
                            self.prev_joint_angles = qpos_array[:7].copy()
                            print(f"  📊 Initial joint positions recorded")
                    else:
                        if qpos_array.shape[0] >= 7:
                            current_joint_angles = qpos_array[:7]
                            movement = np.abs(current_joint_angles - self.prev_joint_angles)
                            max_movement = np.max(movement)
                            print(f"  Max joint movement: {max_movement:.6f}")
                            if max_movement > 0.001:  # Significant movement threshold
                                print(f"  ✅ Joints are moving! Max movement: {max_movement:.6f}")
                            else:
                                print(f"  ⚠️  Joints not moving significantly (max: {max_movement:.6f})")
                            self.prev_joint_angles = current_joint_angles.copy()
            except Exception as verify_error:
                print(f"  ✗ Joint verification failed: {verify_error}")
        
        # Step simulation with joint control
        try:
            # Try to apply joint control before stepping
            if hasattr(self.scene, 'set_joint_targets'):
                self.scene.set_joint_targets(self.robot, desired_joint_angles)
        except Exception as e:
            pass  # Ignore if this method doesn't exist
        
        # Step the simulation
        self.scene.step()
        
        # Render frame for video (exactly like run_cpg_gs_old.py)
        if self.record_video and self.cam is not None:
            try:
                self.cam.render()
            except Exception as e:
                print(f"Camera render error during CPG control: {e}")
                # Don't disable camera immediately, try to continue
                pass
        
        # Record end time for FPS calculation
        step_end_time = time.time()
        step_duration = step_end_time - step_start_time
        current_fps = 1.0 / step_duration if step_duration > 0 else 0
        
        # Calculate average FPS
        if not hasattr(self, 'fps_history'):
            self.fps_history = []
        self.fps_history.append(current_fps)
        if len(self.fps_history) > 100:  # Keep only last 100 FPS measurements
            self.fps_history.pop(0)
        avg_fps = np.mean(self.fps_history)
        
        # Print FPS occasionally
        if self.step_count % 50 == 0:  # Print every 50 steps
            print(f"Step {self.step_count}: FPS = {current_fps:.2f}, Avg FPS = {avg_fps:.2f}")
        
        # Log robot position
        try:
            if hasattr(self, 'robot'):
                pos, _ = self.get_entity_pose(self.robot)
                self.robot_positions_log.append(pos.copy())
                # Debug: Print robot position occasionally
                if self.step_count % 25 == 0:  # Print every 25 steps
                    print(f"Step {self.step_count}: Robot position: x={pos[0]:.3f}, y={pos[1]:.3f}, z={pos[2]:.3f}")
            else:
                self.robot_positions_log.append(np.zeros(3))
        except Exception as e:
            self.robot_positions_log.append(np.zeros(3))
        
        # Log joint angles
        self.joint_angles_log.append(desired_joint_angles.copy())
        
        self.step_count += 1
        
        return desired_joint_angles
    
    def get_entity_pose(self, entity):
        """Get entity pose (position and rotation matrix) - exactly like run_cpg_gs_old.py."""
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
        """Convert quaternion to rotation matrix - exactly like run_cpg_gs_old.py."""
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
                    # Log joint angles
                    self.joint_angles_log.append(joint_angles.copy())
                    
                    # Get robot state
                    try:
                        if hasattr(self, 'robot'):
                            pos, _ = self.get_entity_pose(self.robot)
                            self.robot_positions_log.append(pos.copy())
                        else:
                            self.robot_positions_log.append(np.zeros(3))
                    except Exception as e:
                        self.robot_positions_log.append(np.zeros(3))
                    
                    # Log CPG output
                    self.cpg_output_log.append(joint_angles.copy())
                
                # Print progress
                if step % 200 == 0:  # Less frequent progress updates for speed
                    progress = (step / total_steps) * 100
                    current_time = step * self.SIM_DT
                    print(f"Simulation progress: {progress:.0f}% - Time: {current_time:.2f}s")
        except KeyboardInterrupt:
            print(f"\nSimulation interrupted at step {step}/{total_steps}")
            print("Saving collected data...")
            
            # Stop video recording immediately
            if self.record_video and hasattr(self, 'cam'):
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
            
            raise  # Re-raise to be caught by outer try-catch
        
        # Stop video recording (exactly like run_cpg_gs_old.py)
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
        
        print(f"Simulation completed after {self.step_count} steps")
    
    def plot_results(self):
        """Plot simulation results."""
        try:
            print("Generating plots...")
            
            # Convert logs to numpy arrays
            joint_angles = np.array(self.joint_angles_log)
            robot_positions = np.array(self.robot_positions_log)
            cpg_output = np.array(self.cpg_output_log)
            
            # Create time array
            t = np.arange(len(joint_angles)) * self.SIM_DT
            
            # Create subplots
            fig, axes = plt.subplots(2, 2, figsize=(15, 10))
            fig.suptitle(f'Genesis Swimming Simulation Results - {self.model_name.upper()}', fontsize=16)
            
            # Plot 1: Joint angles over time
            for i in range(min(7, joint_angles.shape[1])):
                axes[0, 0].plot(t, joint_angles[:, i], label=f'Joint {i+1}', alpha=0.7)
            axes[0, 0].set_title('Joint Angles Over Time')
            axes[0, 0].set_xlabel('Time (s)')
            axes[0, 0].set_ylabel('Angle (rad)')
            axes[0, 0].legend()
            axes[0, 0].grid(True)
            
            # Plot 2: Robot X position
            if len(robot_positions) > 0:
                # robot_positions is [time_steps, xyz] (single robot position)
                axes[1, 0].plot(t, robot_positions[:, 0], label='Robot X')
            axes[1, 0].set_title('Robot X Position')
            axes[1, 0].set_xlabel('Time (s)')
            axes[1, 0].set_ylabel('X Position (m)')
            axes[1, 0].legend()
            axes[1, 0].grid(True)
            
            # Plot 3: Robot Z position
            if len(robot_positions) > 0:
                # robot_positions is [time_steps, xyz] (single robot position)
                axes[0, 1].plot(t, robot_positions[:, 2], label='Robot Z')
            axes[0, 1].set_title('Robot Z Position')
            axes[0, 1].set_xlabel('Time (s)')
            axes[0, 1].set_ylabel('Z Position (m)')
            axes[0, 1].legend()
            axes[0, 1].grid(True)
            
            # Plot 4: Robot movement (X-Z plane)
            if len(robot_positions) > 0:
                # robot_positions is [time_steps, xyz] (single robot position)
                axes[1, 1].plot(robot_positions[:, 0], robot_positions[:, 2], label='Robot', alpha=0.7)
            axes[1, 1].set_title('Robot Movement (X-Z Plane)')
            axes[1, 1].set_xlabel('X Position (m)')
            axes[1, 1].set_ylabel('Z Position (m)')
            axes[1, 1].legend()
            axes[1, 1].grid(True)
            axes[1, 1].axis('equal')
            
            plt.tight_layout()
            
            # Save plot
            timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
            plot_filename = f"genesis_swimming_results_{self.model_name}_{timestamp}.png"
            plt.savefig(plot_filename, dpi=300, bbox_inches='tight')
            print(f"✓ Plot saved as: {plot_filename}")
            
            plt.show()
            
        except Exception as e:
            print(f"Error generating plots: {e}")
            import traceback
            traceback.print_exc()
