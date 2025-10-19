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
        
        # Initialize oscillator states with S-shaped pattern
        # Create S-shaped initial configuration for natural swimming
        self.theta = np.zeros(self.num_joints)
        for i in range(self.num_joints):
            # Create S-shaped pattern: alternating positive/negative phases
            if i % 2 == 0:
                self.theta[i] = 0.0  # Even joints start at 0
            else:
                self.theta[i] = np.pi  # Odd joints start at π (180°)
        
        self.theta_dot = np.zeros(self.num_joints)
        self.x = np.cos(self.theta)  # Calculate initial output
        
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
        
        # Initialize Matsuoka oscillator states with S-shaped pattern
        # Each oscillator has two neurons: x_a, v_a, x_b, v_b
        self.x_a = np.zeros(self.num_joints)
        self.v_a = np.zeros(self.num_joints)
        self.x_b = np.zeros(self.num_joints)
        self.v_b = np.zeros(self.num_joints)
        
        # Create S-shaped initial configuration
        for i in range(self.num_joints):
            if i % 2 == 0:
                # Even joints: activate neuron A
                self.x_a[i] = 0.1
                self.x_b[i] = 0.0
            else:
                # Odd joints: activate neuron B
                self.x_a[i] = 0.0
                self.x_b[i] = 0.1
        
        # Output states
        self.xx_a = self.x_a * (self.x_a > 0)
        self.xx_b = self.x_b * (self.x_b > 0)
        
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
        urdf_path = os.path.join(os.path.dirname(__file__), 'assem_description/urdf/assem_description_6joints.urdf')
        print(f"Loading 6-joint swimming robot URDF from: {urdf_path}")
        
        # Position robot - debug mode puts robot at bottom, normal mode slightly underwater
        if debug_mode:
            robot_z = 0.1  # Robot at bottom of water (near ground level)
            print(f"Debug mode: Robot positioned at water bottom")
        else:
            robot_z = 0.29  # Robot center slightly underwater (0.3 - 0.05)
            print(f"Normal mode: Robot positioned slightly underwater")
        
        print(f"Water center: {water_center_z}, Water height: {water_height}, Robot Z: {robot_z}")
        print(f"Water physics: mu={0.01}, gamma={0.02}")
        print(f"Simulation: dt={self.SIM_DT}, substeps={self.SUBSTEPS}, particle_size={self.PARTICLE_SIZE}")
        
        # Try different robot loading approaches
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
        
        # Try to enable joint control after loading
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
        
        # Get robot links for state tracking
        self.robot_links = []
        self.robot_joints = []
        
        # Add camera for video recording
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
                GUI=False,
            )
            print("Camera added for video recording")
        
        # Build scene
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
        
        # Check robot properties after building
        print(f"\n=== Robot Properties Check ===")
        print(f"Robot entity: {self.robot}")
        print(f"Robot has joints: {hasattr(self.robot, 'joints')}")
        if hasattr(self.robot, 'joints'):
            print(f"Number of joints: {len(self.robot.joints) if self.robot.joints else 0}")
            if self.robot.joints:
                for i, joint in enumerate(self.robot.joints):
                    print(f"Joint {i}: {joint}")
                    # Try to get initial joint position and check available methods
                    try:
                        if hasattr(joint, 'get_angle'):
                            initial_angle = joint.get_angle()
                            print(f"  Initial angle: {initial_angle}")
                        else:
                            print(f"  No get_angle method")
                        
                        # Check available methods
                        methods = [method for method in dir(joint) if not method.startswith('_')]
                        joint_methods = [m for m in methods if 'set' in m.lower() or 'angle' in m.lower() or 'position' in m.lower()]
                        print(f"  Available joint methods: {joint_methods}")
                        print(f"  All joint methods: {methods}")  # Show all methods for debugging
                    except Exception as e:
                        print(f"  Error checking joint: {e}")
        print(f"Robot has set_joint_positions: {hasattr(self.robot, 'set_joint_positions')}")
        
        # Check robot entity methods
        robot_methods = [method for method in dir(self.robot) if not method.startswith('_')]
        joint_related_methods = [m for m in robot_methods if 'joint' in m.lower() or 'set' in m.lower() or 'q' in m.lower()]
        print(f"Robot joint-related methods: {joint_related_methods}")
        
        # Check scene methods
        scene_methods = [method for method in dir(self.scene) if not method.startswith('_')]
        scene_joint_methods = [m for m in scene_methods if 'joint' in m.lower() or 'set' in m.lower()]
        print(f"Scene joint-related methods: {scene_joint_methods}")
        print("=== End Robot Properties Check ===\n")
        
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
            
            # Skip control gains setup to prevent instability
            print("⚠️  Skipping control gains setup to prevent nan values")
            
            # # Set control gains according to Genesis documentation - very conservative values
            # # Set positional gains (kp) - very low values for stability
            # kp_values = np.array([10.0] * len(self.dofs_idx))  # Further reduced from 100.0
            # self.robot.set_dofs_kp(
            #     kp=kp_values,
            #     dofs_idx_local=self.dofs_idx
            # )
            # print(f"Set positional gains (kp): {kp_values}")
            # 
            # # Set velocity gains (kv) - very low damping
            # kv_values = np.array([1.0] * len(self.dofs_idx))  # Further reduced from 10.0
            # self.robot.set_dofs_kv(
            #     kv=kv_values,
            #     dofs_idx_local=self.dofs_idx
            # )
            # print(f"Set velocity gains (kv): {kv_values}")
            # 
            # # Set force range for safety - very conservative
            # force_lower = np.array([-1.0] * len(self.dofs_idx))  # Further reduced from -10.0
            # force_upper = np.array([1.0] * len(self.dofs_idx))   # Further reduced from 10.0
            # self.robot.set_dofs_force_range(
            #     lower=force_lower,
            #     upper=force_upper,
            #     dofs_idx_local=self.dofs_idx
            # )
            # print(f"Set force range: [{force_lower[0]:.1f}, {force_upper[0]:.1f}]")
            
        except Exception as e:
            print(f"Failed to set up joint control: {e}")
            # Fallback: use simple indices
            self.dofs_idx = list(range(7))  # Assume 7 joints
            self.joint_names = [f"joint_{i}" for i in range(7)]
        
        # Initialize CPG with original frequency
        if cpg_model_type == 0:
            # Use original frequency setting - match URDF joint count
            self.cpg = PhaseOscillatorNetwork(
                num_joints=7,  # Match URDF joint count
                freq=5.0,  # Higher frequency for more visible movement
                w_up=10.0,
                w_down=30.0,
                prop_fb=0.0,
                ext_fb=0.0,
                kappa=0.0,
                nb_fb=0.0,
                tau=0.01,
                time_step=self.SIM_DT
            )
            print(f"Initialized Phase Oscillator CPG with {self.cpg.num_joints} joints")
            print(f"CPG frequency: {self.cpg.freq} Hz")
            print(f"Initial theta values: {self.cpg.get_theta()}")
        else:
            # Use original Matsuoka network parameters - match URDF joint count
            self.cpg = MatsuokaNetwork(
                num_joints=7,  # Match URDF joint count
                beta=5.0,
                eta=-3.0,
                gamma=3.0,
                eta_i=-2.0,
                eta_e=1.0,
                eta_head=0.0,
                tonic=50.0,  # Original tonic value
                force=0.0,
                tau=0.05,
                fb_i=-10.0,
                fb_e=1.0,
                time_step=self.SIM_DT
            )
            print(f"Initialized Matsuoka CPG with {self.cpg.num_joints} joints")
            print(f"CPG tonic: {self.cpg.tonic}")
            print(f"Initial x_a values: {self.cpg.get_x_a()}")
        
        # Data logging
        self.log = {
            "t": [],
            "joint_angles": [],
            "joint_velocities": [],
            "robot_positions": [],
            "robot_velocities": [],
            "cpg_states": []
        }
        
        # Test if CPG is working properly
        print("\n=== CPG Test ===")
        
        # Show initial S-shaped configuration
        if self.cpg_model_type == 0:
            initial_theta = self.cpg.get_theta()
            initial_output = self.cpg.get_x()
            print(f"Initial S-shaped configuration:")
            print(f"  Theta (phases): {initial_theta}")
            print(f"  Output (cos):   {initial_output}")
            print(f"  Expected S-shape: alternating 0 and π phases")
            print(f"  CPG joints: {len(initial_output)}, URDF joints: {len(self.robot.joints) if hasattr(self, 'robot') and self.robot.joints else 'Unknown'}")
        else:
            initial_x_a = self.cpg.get_x_a()
            initial_x_b = self.cpg.get_x_b()
            initial_output = initial_x_a - initial_x_b
            print(f"Initial S-shaped configuration:")
            print(f"  x_a: {initial_x_a}")
            print(f"  x_b: {initial_x_b}")
            print(f"  Output (x_a - x_b): {initial_output}")
            print(f"  Expected S-shape: alternating positive/negative outputs")
            print(f"  CPG joints: {len(initial_output)}, URDF joints: {len(self.robot.joints) if hasattr(self, 'robot') and self.robot.joints else 'Unknown'}")
        
        
        
        # Start video recording
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
        """Apply joint angles to robot."""
        # Store desired angles
        self.desired_joint_angles = joint_angles
        
        # Debug: Print joint angles occasionally
        if not hasattr(self, 'joint_debug_count'):
            self.joint_debug_count = 0
        self.joint_debug_count += 1
        
        if self.joint_debug_count % 10 == 0:  # Print every 10 steps for more frequent debugging
            print(f"Step {self.joint_debug_count}: Joint angles to apply: {joint_angles}")
        
        # Try Genesis official control methods first (according to documentation)
        # Check for nan values in joint angles to prevent instability
        if np.any(np.isnan(joint_angles)) or np.any(np.isinf(joint_angles)):
            if self.joint_debug_count % 10 == 0:
                print(f"  ⚠️  Warning: Invalid joint angles detected (nan/inf), skipping control")
            return
        
        # Use Genesis official set_dofs_position method (hard reset) - This is the working method
        
        try:
            # Use set_dofs_position (hard reset) - Official Genesis method
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
            # Check if robot has joints
            if hasattr(self.robot, 'joints') and len(self.robot.joints) > 0:
                for i, joint in enumerate(self.robot.joints):
                    if i < len(joint_angles):
                        # Try different Genesis joint control methods
                        try:
                            # Try all possible methods
                            methods_to_try = [
                                ('set_angle', lambda j, a: j.set_angle(a)),
                                ('set_position', lambda j, a: j.set_position(a)),
                                ('set_q', lambda j, a: j.set_q(a)),
                                ('set_joint_angle', lambda j, a: j.set_joint_angle(a)),
                                ('set_joint_position', lambda j, a: j.set_joint_position(a)),
                                ('set_target', lambda j, a: j.set_target(a)),
                                ('set_target_position', lambda j, a: j.set_target_position(a)),
                                ('set_target_angle', lambda j, a: j.set_target_angle(a)),
                            ]
                            
                            success = False
                            for method_name, method_func in methods_to_try:
                                if hasattr(joint, method_name):
                                    try:
                                        method_func(joint, joint_angles[i])
                                        if self.joint_debug_count % 10 == 0:
                                            print(f"  ✓ Set joint {i} to angle {joint_angles[i]:.3f} using {method_name}")
                                        success = True
                                        break
                                    except Exception as method_error:
                                        if self.joint_debug_count % 10 == 0:
                                            print(f"  ✗ {method_name} failed: {method_error}")
                                        continue
                            
                            if not success:
                                if self.joint_debug_count % 10 == 0:
                                    print(f"  ✗ Joint {i} has no working control methods")
                        except Exception as joint_error:
                            if self.joint_debug_count % 50 == 0:
                                print(f"Failed to set joint {i}: {joint_error}")
                            pass
                else:
                    if self.joint_debug_count % 10 == 0:
                        print("  ✗ Robot has no joints or joints list is empty")
                
                # Try robot-level joint control methods
                robot_methods = ['set_joint_positions', 'set_q', 'set_joint_angles', 'set_joint_targets', 'set_targets']
                for method_name in robot_methods:
                    if hasattr(self.robot, method_name):
                        try:
                            method = getattr(self.robot, method_name)
                            method(joint_angles)
                            if self.joint_debug_count % 10 == 0:
                                print(f"  ✓ Used robot.{method_name} method")
                            break
                        except Exception as method_error:
                            if self.joint_debug_count % 10 == 0:
                                print(f"  ✗ robot.{method_name} failed: {method_error}")
                            continue
                
                # Try scene-level joint control
                if hasattr(self.scene, 'set_joint_positions'):
                    try:
                        self.scene.set_joint_positions(self.robot, joint_angles)
                        if self.joint_debug_count % 10 == 0:
                            print(f"  ✓ Used scene.set_joint_positions method")
                    except Exception as scene_error:
                        if self.joint_debug_count % 10 == 0:
                            print(f"  ✗ scene.set_joint_positions failed: {scene_error}")
                
                # Try setting joint targets through scene
                if hasattr(self.scene, 'set_joint_targets'):
                    try:
                        self.scene.set_joint_targets(self.robot, joint_angles)
                        if self.joint_debug_count % 10 == 0:
                            print(f"  ✓ Used scene.set_joint_targets method")
                    except Exception as scene_error:
                        if self.joint_debug_count % 10 == 0:
                            print(f"  ✗ scene.set_joint_targets failed: {scene_error}")
                
                # Try Genesis URDF control method
                try:
                    # Method: Set joint positions directly on the URDF morph
                    if hasattr(self.robot, 'morph') and hasattr(self.robot.morph, 'set_joint_positions'):
                        self.robot.morph.set_joint_positions(joint_angles)
                        if self.joint_debug_count % 10 == 0:
                            print(f"  ✓ Used robot.morph.set_joint_positions method")
                except Exception as morph_error:
                    if self.joint_debug_count % 10 == 0:
                        print(f"  ✗ robot.morph.set_joint_positions failed: {morph_error}")
                
                # Try setting joint positions as a dictionary
                try:
                    joint_dict = {f'joint{i+1}': joint_angles[i] for i in range(len(joint_angles))}
                    if hasattr(self.robot, 'set_joint_positions'):
                        self.robot.set_joint_positions(joint_dict)
                        if self.joint_debug_count % 10 == 0:
                            print(f"  ✓ Used robot.set_joint_positions with dict")
                except Exception as dict_error:
                    if self.joint_debug_count % 10 == 0:
                        print(f"  ✗ robot.set_joint_positions with dict failed: {dict_error}")
                
                # Try Genesis actuation method (for soft robots or muscle-driven robots)
                try:
                    if hasattr(self.robot, 'set_actuation'):
                        # Convert joint angles to actuation values (normalize to 0-1 range)
                        actuation = (np.array(joint_angles) + 1.0) / 2.0  # Convert from [-1,1] to [0,1]
                        self.robot.set_actuation(actuation)
                        if self.joint_debug_count % 10 == 0:
                            print(f"  ✓ Used robot.set_actuation method with values: {actuation}")
                except Exception as actuation_error:
                    if self.joint_debug_count % 10 == 0:
                        print(f"  ✗ robot.set_actuation failed: {actuation_error}")
                
                # Try setting joint positions using Genesis scene API
                try:
                    # Method: Use scene to set joint positions for the robot
                    if hasattr(self.scene, 'set_entity_joint_positions'):
                        self.scene.set_entity_joint_positions(self.robot, joint_angles)
                        if self.joint_debug_count % 10 == 0:
                            print(f"  ✓ Used scene.set_entity_joint_positions method")
                except Exception as entity_error:
                    if self.joint_debug_count % 10 == 0:
                        print(f"  ✗ scene.set_entity_joint_positions failed: {entity_error}")
                
                # Try direct joint manipulation through Genesis physics
                try:
                    # Method: Access joint physics directly
                    for i, joint in enumerate(self.robot.joints):
                        if i < len(joint_angles):
                            # Try to set joint state directly
                            if hasattr(joint, 'state'):
                                joint.state.position = joint_angles[i]
                                if self.joint_debug_count % 10 == 0 and i == 0:
                                    print(f"  ✓ Set joint {i} state.position to {joint_angles[i]:.3f}")
                            elif hasattr(joint, 'q'):
                                joint.q = joint_angles[i]
                                if self.joint_debug_count % 10 == 0 and i == 0:
                                    print(f"  ✓ Set joint {i} q to {joint_angles[i]:.3f}")
                except Exception as physics_error:
                    if self.joint_debug_count % 10 == 0:
                        print(f"  ✗ Direct joint physics manipulation failed: {physics_error}")
                
                # Try using Genesis controller system
                try:
                    # Method: Create or use a joint controller
                    if not hasattr(self, 'joint_controller'):
                        # Try to create a joint controller
                        if hasattr(gs, 'controllers') and hasattr(gs.controllers, 'JointController'):
                            self.joint_controller = gs.controllers.JointController(self.robot)
                            print("Created Genesis joint controller")
                        elif hasattr(self.scene, 'add_controller'):
                            # Try to add a joint controller to the scene
                            controller = self.scene.add_controller('joint', self.robot)
                            if controller:
                                self.joint_controller = controller
                                print("Added joint controller to scene")
                    
                    # Use the controller if available
                    if hasattr(self, 'joint_controller'):
                        if hasattr(self.joint_controller, 'set_targets'):
                            self.joint_controller.set_targets(joint_angles)
                            if self.joint_debug_count % 10 == 0:
                                print(f"  ✓ Used joint_controller.set_targets method")
                        elif hasattr(self.joint_controller, 'set_joint_positions'):
                            self.joint_controller.set_joint_positions(joint_angles)
                            if self.joint_debug_count % 10 == 0:
                                print(f"  ✓ Used joint_controller.set_joint_positions method")
                except Exception as controller_error:
                    if self.joint_debug_count % 10 == 0:
                        print(f"  ✗ Joint controller method failed: {controller_error}")
                
                # Try Genesis PD controller approach (like gym)
                try:
                    # Method: Use PD controller similar to gym environment
                    if not hasattr(self, 'pd_controller'):
                        # Try to create a PD controller for joints
                        if hasattr(gs, 'controllers') and hasattr(gs.controllers, 'PDController'):
                            self.pd_controller = gs.controllers.PDController(self.robot)
                            print("Created Genesis PD controller")
                        elif hasattr(self.scene, 'add_controller'):
                            # Try to add a PD controller
                            pd_controller = self.scene.add_controller('pd', self.robot)
                            if pd_controller:
                                self.pd_controller = pd_controller
                                print("Added PD controller to scene")
                    
                    # Use PD controller if available
                    if hasattr(self, 'pd_controller'):
                        if hasattr(self.pd_controller, 'set_targets'):
                            self.pd_controller.set_targets(joint_angles)
                            if self.joint_debug_count % 10 == 0:
                                print(f"  ✓ Used pd_controller.set_targets method")
                        elif hasattr(self.pd_controller, 'set_joint_targets'):
                            self.pd_controller.set_joint_targets(joint_angles)
                            if self.joint_debug_count % 10 == 0:
                                print(f"  ✓ Used pd_controller.set_joint_targets method")
                except Exception as pd_error:
                    if self.joint_debug_count % 10 == 0:
                        print(f"  ✗ PD controller method failed: {pd_error}")
                
                # Try direct joint state modification (force approach)
                try:
                    # Method: Force set joint states directly
                    for i, joint in enumerate(self.robot.joints):
                        if i < len(joint_angles):
                            # Try to directly modify joint physics state
                            if hasattr(joint, 'physics_state'):
                                joint.physics_state.position = joint_angles[i]
                                if self.joint_debug_count % 10 == 0 and i == 0:
                                    print(f"  ✓ Set joint {i} physics_state.position to {joint_angles[i]:.3f}")
                            elif hasattr(joint, 'target_position'):
                                joint.target_position = joint_angles[i]
                                if self.joint_debug_count % 10 == 0 and i == 0:
                                    print(f"  ✓ Set joint {i} target_position to {joint_angles[i]:.3f}")
                            elif hasattr(joint, 'desired_position'):
                                joint.desired_position = joint_angles[i]
                                if self.joint_debug_count % 10 == 0 and i == 0:
                                    print(f"  ✓ Set joint {i} desired_position to {joint_angles[i]:.3f}")
                except Exception as force_error:
                    if self.joint_debug_count % 10 == 0:
                        print(f"  ✗ Force joint state modification failed: {force_error}")
                
                # Try Genesis DOF (Degrees of Freedom) system
                try:
                    # Method: Use DOF system for joint control
                    if hasattr(self.robot, 'set_dofs_position'):
                        # Set joint positions using DOF system
                        self.robot.set_dofs_position(joint_angles)
                        if self.joint_debug_count % 10 == 0:
                            print(f"  ✓ Used robot.set_dofs_position method")
                    elif hasattr(self.robot, 'set_dofs_motion_ang'):
                        # Set motion angles using DOF system
                        self.robot.set_dofs_motion_ang(joint_angles)
                        if self.joint_debug_count % 10 == 0:
                            print(f"  ✓ Used robot.set_dofs_motion_ang method")
                    
                    # Try setting DOF parameters for each joint
                    for i, joint in enumerate(self.robot.joints):
                        if i < len(joint_angles):
                            # Try to set DOF motion angle directly
                            if hasattr(joint, 'dofs_motion_ang'):
                                joint.dofs_motion_ang = joint_angles[i]
                                if self.joint_debug_count % 10 == 0 and i == 0:
                                    print(f"  ✓ Set joint {i} dofs_motion_ang to {joint_angles[i]:.3f}")
                            elif hasattr(joint, 'dofs_kp'):
                                # Set stiffness and try to control
                                joint.dofs_kp = 100.0  # High stiffness
                                joint.dofs_kv = 10.0   # Damping
                                if self.joint_debug_count % 10 == 0 and i == 0:
                                    print(f"  ✓ Set joint {i} DOF parameters (kp=100, kv=10)")
                except Exception as dof_error:
                    if self.joint_debug_count % 10 == 0:
                        print(f"  ✗ DOF system control failed: {dof_error}")
                
                # Try setting joint solver parameters
                try:
                    # Method: Use joint solver parameters
                    for i, joint in enumerate(self.robot.joints):
                        if i < len(joint_angles):
                            # Try to set solver parameters for joint control
                            if hasattr(joint, 'set_sol_params'):
                                # Set solver parameters with target position
                                joint.set_sol_params(
                                    kp=100.0,  # Position gain
                                    kv=10.0,   # Velocity gain
                                    target_pos=joint_angles[i]  # Target position
                                )
                                if self.joint_debug_count % 10 == 0 and i == 0:
                                    print(f"  ✓ Set joint {i} solver params with target_pos={joint_angles[i]:.3f}")
                except Exception as solver_error:
                    if self.joint_debug_count % 10 == 0:
                        print(f"  ✗ Joint solver parameters failed: {solver_error}")
                
                # Try Genesis qpos system (most promising based on debug output)
                try:
                    # Method: Use qpos system for joint control
                    if hasattr(self.robot, 'set_qpos'):
                        # Get current qpos and modify joint positions
                        current_qpos = self.robot.get_qpos()
                        if current_qpos is not None and len(current_qpos) >= len(joint_angles):
                            # Handle Tensor object (convert to numpy array if needed)
                            if hasattr(current_qpos, 'numpy'):
                                # Convert tensor to numpy array
                                new_qpos = current_qpos.numpy().copy()
                            elif hasattr(current_qpos, 'copy'):
                                new_qpos = current_qpos.copy()
                            else:
                                # Convert to list and back to array
                                new_qpos = np.array(current_qpos.tolist() if hasattr(current_qpos, 'tolist') else list(current_qpos))
                            
                            # Set joint positions in qpos (assuming first 7 elements are joint positions)
                            new_qpos[:len(joint_angles)] = joint_angles
                            self.robot.set_qpos(new_qpos)
                            if self.joint_debug_count % 10 == 0:
                                print(f"  ✓ Used robot.set_qpos method with joint angles")
                        else:
                            # Try setting qpos directly with joint angles
                            self.robot.set_qpos(joint_angles)
                            if self.joint_debug_count % 10 == 0:
                                print(f"  ✓ Used robot.set_qpos method directly")
                except Exception as qpos_error:
                    if self.joint_debug_count % 10 == 0:
                        print(f"  ✗ QPOS system control failed: {qpos_error}")
                
                # Try direct joint qpos manipulation
                try:
                    # Method: Direct joint qpos manipulation
                    for i, joint in enumerate(self.robot.joints):
                        if i < len(joint_angles):
                            # Try to set joint qpos directly
                            if hasattr(joint, 'q_start') and hasattr(joint, 'q_end'):
                                # Get joint qpos range
                                q_start = joint.q_start
                                q_end = joint.q_end
                                if q_start is not None and q_end is not None:
                                    # Set joint position in robot's qpos
                                    if hasattr(self.robot, 'set_qpos'):
                                        current_qpos = self.robot.get_qpos()
                                        if current_qpos is not None and q_start < len(current_qpos):
                                            # Handle Tensor object
                                            if hasattr(current_qpos, 'numpy'):
                                                new_qpos = current_qpos.numpy().copy()
                                            elif hasattr(current_qpos, 'copy'):
                                                new_qpos = current_qpos.copy()
                                            else:
                                                new_qpos = np.array(current_qpos.tolist() if hasattr(current_qpos, 'tolist') else list(current_qpos))
                                            
                                            new_qpos[q_start] = joint_angles[i]
                                            self.robot.set_qpos(new_qpos)
                                            if self.joint_debug_count % 10 == 0 and i == 0:
                                                print(f"  ✓ Set joint {i} qpos[{q_start}] to {joint_angles[i]:.3f}")
                except Exception as joint_qpos_error:
                    if self.joint_debug_count % 10 == 0:
                        print(f"  ✗ Joint qpos manipulation failed: {joint_qpos_error}")
                
                # Try simplified qpos approach - set all joints at once
                try:
                    # Method: Simplified qpos approach
                    if hasattr(self.robot, 'set_qpos'):
                        # Get current qpos to understand the structure
                        current_qpos = self.robot.get_qpos()
                        if current_qpos is not None:
                            # Handle Tensor object
                            if hasattr(current_qpos, 'numpy'):
                                qpos_array = current_qpos.numpy().copy()
                            elif hasattr(current_qpos, 'copy'):
                                qpos_array = current_qpos.copy()
                            else:
                                qpos_array = np.array(current_qpos.tolist() if hasattr(current_qpos, 'tolist') else list(current_qpos))
                            
                            # Set joint angles in the correct positions
                            # Assuming joint angles are at the beginning of qpos
                            for i, angle in enumerate(joint_angles):
                                if i < len(qpos_array):
                                    qpos_array[i] = angle
                            
                            self.robot.set_qpos(qpos_array)
                            if self.joint_debug_count % 10 == 0:
                                print(f"  ✓ Set all joints using corrected qpos approach")
                        else:
                            # Fallback: try direct joint angles
                            qpos_array = np.array(joint_angles, dtype=np.float32)
                            self.robot.set_qpos(qpos_array)
                            if self.joint_debug_count % 10 == 0:
                                print(f"  ✓ Set all joints using direct qpos approach")
                except Exception as simple_qpos_error:
                    if self.joint_debug_count % 10 == 0:
                        print(f"  ✗ Simplified qpos approach failed: {simple_qpos_error}")
                
                # Try DOF position setting with correct dimensions
                try:
                    # Method: Use DOF position setting with proper dimensions
                    if hasattr(self.robot, 'set_dofs_position'):
                        # Get the number of DOFs
                        total_dofs = 0
                        for joint in self.robot.joints:
                            if hasattr(joint, 'n_dofs'):
                                total_dofs += joint.n_dofs
                        
                        if total_dofs > 0:
                            # Create DOF array with correct dimensions
                            dof_array = np.zeros(total_dofs, dtype=np.float32)
                            dof_idx = 0
                            
                            # Set joint angles in DOF array
                            for i, angle in enumerate(joint_angles):
                                if i < len(self.robot.joints):
                                    joint = self.robot.joints[i]
                                    if hasattr(joint, 'n_dofs') and joint.n_dofs > 0:
                                        dof_array[dof_idx] = angle
                                        dof_idx += joint.n_dofs
                            
                            self.robot.set_dofs_position(dof_array)
                            if self.joint_debug_count % 10 == 0:
                                print(f"  ✓ Set all joints using DOF position (total_dofs={total_dofs})")
                except Exception as dof_pos_error:
                    if self.joint_debug_count % 10 == 0:
                        print(f"  ✗ DOF position approach failed: {dof_pos_error}")
                
                # Try individual joint DOF setting
                try:
                    # Method: Set each joint's DOF individually
                    for i, joint in enumerate(self.robot.joints):
                        if i < len(joint_angles):
                            # Try to set joint DOF directly
                            if hasattr(joint, 'dof_start') and hasattr(joint, 'dof_end'):
                                dof_start = joint.dof_start
                                dof_end = joint.dof_end
                                if dof_start is not None and dof_end is not None:
                                    # Set this joint's DOF
                                    if hasattr(self.robot, 'set_dofs_position'):
                                        # Get current DOF state
                                        current_dofs = self.robot.get_qpos()  # This might be DOF state
                                        if current_dofs is not None:
                                            # Handle Tensor
                                            if hasattr(current_dofs, 'numpy'):
                                                dof_array = current_dofs.numpy().copy()
                                            elif hasattr(current_dofs, 'copy'):
                                                dof_array = current_dofs.copy()
                                            else:
                                                dof_array = np.array(current_dofs.tolist() if hasattr(current_dofs, 'tolist') else list(current_dofs))
                                            
                                            # Set this joint's DOF
                                            if dof_start < len(dof_array):
                                                dof_array[dof_start] = joint_angles[i]
                                                self.robot.set_dofs_position(dof_array)
                                                if self.joint_debug_count % 10 == 0 and i == 0:
                                                    print(f"  ✓ Set joint {i} DOF[{dof_start}] to {joint_angles[i]:.3f}")
                except Exception as individual_dof_error:
                    if self.joint_debug_count % 10 == 0:
                        print(f"  ✗ Individual joint DOF setting failed: {individual_dof_error}")
                
                # Try force-based joint control
                try:
                    # Method: Use force control to move joints
                    if hasattr(self.robot, 'set_dofs_force_range'):
                        # Set force range for joints (need upper and lower bounds)
                        force_lower = np.array([-100.0] * len(joint_angles))  # Lower force bound
                        force_upper = np.array([100.0] * len(joint_angles))   # Upper force bound
                        self.robot.set_dofs_force_range(force_lower, force_upper)
                    
                    if hasattr(self.robot, 'set_dofs_velocity'):
                        # Try to set joint velocities to move towards target
                        target_velocities = np.array(joint_angles) * 10.0  # Scale for velocity
                        self.robot.set_dofs_velocity(target_velocities)
                        if self.joint_debug_count % 10 == 0:
                            print(f"  ✓ Set joint velocities for force control")
                    
                    # Try setting high stiffness and damping for better control
                    if hasattr(self.robot, 'set_dofs_kp'):
                        kp_values = np.array([1000.0] * len(joint_angles))  # High stiffness
                        self.robot.set_dofs_kp(kp_values)
                    
                    if hasattr(self.robot, 'set_dofs_kv'):
                        kv_values = np.array([100.0] * len(joint_angles))  # High damping
                        self.robot.set_dofs_kv(kv_values)
                        if self.joint_debug_count % 10 == 0:
                            print(f"  ✓ Set high stiffness/damping for better control")
                except Exception as force_control_error:
                    if self.joint_debug_count % 10 == 0:
                        print(f"  ✗ Force-based joint control failed: {force_control_error}")
                
                # Try Genesis actuator system
                try:
                    # Method: Use Genesis actuator system
                    if hasattr(self.robot, 'actuators'):
                        # Try to control actuators directly
                        for i, actuator in enumerate(self.robot.actuators):
                            if i < len(joint_angles):
                                if hasattr(actuator, 'set_target'):
                                    actuator.set_target(joint_angles[i])
                                elif hasattr(actuator, 'set_position'):
                                    actuator.set_position(joint_angles[i])
                                elif hasattr(actuator, 'set_angle'):
                                    actuator.set_angle(joint_angles[i])
                        if self.joint_debug_count % 10 == 0:
                            print(f"  ✓ Used Genesis actuator system")
                except Exception as actuator_error:
                    if self.joint_debug_count % 10 == 0:
                        print(f"  ✗ Genesis actuator system failed: {actuator_error}")
                
                # Try direct physics manipulation
                try:
                    # Method: Direct physics state manipulation
                    for i, joint in enumerate(self.robot.joints):
                        if i < len(joint_angles):
                            # Try to directly manipulate joint physics
                            if hasattr(joint, 'state'):
                                if hasattr(joint.state, 'position'):
                                    joint.state.position = joint_angles[i]
                                elif hasattr(joint.state, 'q'):
                                    joint.state.q = joint_angles[i]
                                elif hasattr(joint.state, 'angle'):
                                    joint.state.angle = joint_angles[i]
                            elif hasattr(joint, 'physics_state'):
                                if hasattr(joint.physics_state, 'position'):
                                    joint.physics_state.position = joint_angles[i]
                            if self.joint_debug_count % 10 == 0 and i == 0:
                                print(f"  ✓ Direct physics manipulation for joint {i}")
                except Exception as physics_error:
                    if self.joint_debug_count % 10 == 0:
                        print(f"  ✗ Direct physics manipulation failed: {physics_error}")
                
                # Try scene-level joint control
                try:
                    # Method: Use scene to control joints
                    if hasattr(self.scene, 'set_entity_joint_positions'):
                        self.scene.set_entity_joint_positions(self.robot, joint_angles)
                        if self.joint_debug_count % 10 == 0:
                            print(f"  ✓ Used scene.set_entity_joint_positions")
                    elif hasattr(self.scene, 'set_joint_positions'):
                        self.scene.set_joint_positions(self.robot, joint_angles)
                        if self.joint_debug_count % 10 == 0:
                            print(f"  ✓ Used scene.set_joint_positions")
                except Exception as scene_error:
                    if self.joint_debug_count % 10 == 0:
                        print(f"  ✗ Scene-level joint control failed: {scene_error}")
                        
                except Exception as e:
                    # If joint control fails, at least record angles
                    if not hasattr(self, 'joint_control_errors'):
                        self.joint_control_errors = 0
                    self.joint_control_errors += 1
                    if self.joint_control_errors % 100 == 0:
                        print(f"Joint control error (attempt {self.joint_control_errors}): {e}")
                        print("Robot may not be moving due to joint control issues.")
    
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
        import time
        
        # Record start time for FPS calculation
        step_start_time = time.time()
        
        # Update CPG
        desired_joint_angles = self.cpg.update()
        
        # Print CPG output for debugging
        if not hasattr(self, 'step_count'):
            self.step_count = 0
        self.step_count += 1
        
        # Adjust debug output frequency based on debug mode
        if self.debug_mode:
            debug_frequency = 20  # More frequent debugging in debug mode
        else:
            debug_frequency = 100  # Normal mode
        
        if self.step_count % debug_frequency == 0:
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
            elif hasattr(self.scene, 'apply_joint_actions'):
                self.scene.apply_joint_actions(self.robot, desired_joint_angles)
            elif hasattr(self.scene, 'step_with_action'):
                # Try gym-like step with action
                self.scene.step_with_action(self.robot, desired_joint_angles)
            elif hasattr(self.robot, 'step'):
                # Try robot-level step method
                self.robot.step(desired_joint_angles)
        except Exception as e:
            pass  # Continue with normal step
        
        self.scene.step()
        
        # Render frame for video
        if self.record_video and self.cam is not None:
            try:
                self.cam.render()
            except Exception as e:
                print(f"Camera render error during CPG control: {e}")
                # Don't disable camera immediately, try to continue
                pass
        
        # Calculate FPS
        step_end_time = time.time()
        step_duration = step_end_time - step_start_time
        
        if not hasattr(self, 'fps_history'):
            self.fps_history = []
        
        if step_duration > 0:
            current_fps = 1.0 / step_duration
            self.fps_history.append(current_fps)
            
            # Keep recent 100 FPS records
            if len(self.fps_history) > 100:
                self.fps_history = self.fps_history[-100:]
            
            # Adjust FPS output frequency based on debug mode
            if self.debug_mode:
                fps_frequency = 20  # More frequent FPS output in debug mode
            else:
                fps_frequency = 100  # Normal mode
            
            if self.step_count % fps_frequency == 0:
                avg_fps = sum(self.fps_history) / len(self.fps_history)
                print(f"Current FPS: {current_fps:.2f}, Average FPS: {avg_fps:.2f}")
        
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
        if len(robot_positions) > 0:
            # Check if robot_positions has the expected structure
            if len(robot_positions.shape) >= 2 and robot_positions.shape[1] >= 3:
                # robot_positions is [time_steps, links, xyz]
                axes[1, 0].plot(t, robot_positions[:, 0, 0], label='Link 1')
                if robot_positions.shape[1] > 3:
                    axes[1, 0].plot(t, robot_positions[:, 3, 0], label='Link 4')
                if robot_positions.shape[1] > 5:
                    axes[1, 0].plot(t, robot_positions[:, 5, 0], label='Link 6')
            else:
                # Fallback: assume robot_positions is [time_steps, xyz] (single link)
                axes[1, 0].plot(t, robot_positions[:, 0], label='Robot X')
        axes[1, 0].set_title('Robot X Position')
        axes[1, 0].set_xlabel('Time (s)')
        axes[1, 0].set_ylabel('Position (m)')
        axes[1, 0].legend()
        axes[1, 0].grid(True)
        
        # Plot 4: Robot Z position
        if len(robot_positions) > 0:
            # Check if robot_positions has the expected structure
            if len(robot_positions.shape) >= 2 and robot_positions.shape[1] >= 3:
                # robot_positions is [time_steps, links, xyz]
                axes[1, 1].plot(t, robot_positions[:, 0, 2], label='Link 1')
                if robot_positions.shape[1] > 3:
                    axes[1, 1].plot(t, robot_positions[:, 3, 2], label='Link 4')
                if robot_positions.shape[1] > 5:
                    axes[1, 1].plot(t, robot_positions[:, 5, 2], label='Link 6')
            else:
                # Fallback: assume robot_positions is [time_steps, xyz] (single link)
                axes[1, 1].plot(t, robot_positions[:, 2], label='Robot Z')
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
        if len(robot_positions) > 0:
            # Check if robot_positions has the expected structure
            if len(robot_positions.shape) >= 2 and robot_positions.shape[1] >= 3:
                # robot_positions is [time_steps, links, xyz]
                axes[2, 1].plot(robot_positions[:, 0, 0], robot_positions[:, 0, 2], label='Link 1')
                if robot_positions.shape[1] > 3:
                    axes[2, 1].plot(robot_positions[:, 3, 0], robot_positions[:, 3, 2], label='Link 4')
                if robot_positions.shape[1] > 5:
                    axes[2, 1].plot(robot_positions[:, 5, 0], robot_positions[:, 5, 2], label='Link 6')
            else:
                # Fallback: assume robot_positions is [time_steps, xyz] (single link)
                axes[2, 1].plot(robot_positions[:, 0], robot_positions[:, 2], label='Robot')
        axes[2, 1].set_title('Robot Movement (X-Z plane)')
        axes[2, 1].set_xlabel('X Position (m)')
        axes[2, 1].set_ylabel('Z Position (m)')
        axes[2, 1].legend()
        axes[2, 1].grid(True)
        
        plt.tight_layout()
        plt.savefig(f'{self.model_name}_genesis_test.png', dpi=300, bbox_inches='tight')
        plt.show()
        
        print(f"Plot saved as {self.model_name}_genesis_test.png")


def test_genesis_cpg(cpg_model_type=0, debug_mode=False):
    """
    Test CPG network with Genesis simulation.
    
    Args:
        cpg_model_type (int): 0 for phase oscillator, 1 for Matsuoka oscillator
        debug_mode (bool): Enable debug mode with larger particles and top-down view
    """
    print("CPG Implementation with Genesis")
    print("0: Phase Oscillator Network")
    print("1: Matsuoka Oscillator Network")
    print(f"Debug mode: {'ON' if debug_mode else 'OFF'}")
    # Create simulation with debug mode
    sim = GenesisSwimmingSimulation(
        cpg_model_type=cpg_model_type,
        use_gpu=False,
        record_video=True,  # Re-enable video recording
        debug_mode=debug_mode  # Enable debug mode
    )
    
    # Run simulation
    try:
        sim.run_simulation(duration=1.0, log_every=25)  # Longer duration to observe movement
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
    # You can change these parameters to switch between models and modes
    cpg_model_type = 0  # 0 for phase oscillator, 1 for Matsuoka oscillator
    debug_mode = True   # True for debug mode (larger particles, top-down view), False for normal mode
    test_genesis_cpg(cpg_model_type, debug_mode)
