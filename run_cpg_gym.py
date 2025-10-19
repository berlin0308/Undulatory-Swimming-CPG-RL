#!/usr/bin/env python3
"""
CPG implementation for swimming robot.
Implements both phase oscillator and Matsuoka oscillator models.
"""

import numpy as np
import matplotlib.pyplot as plt
import sys
import os

# Add env directory to path
sys.path.append(os.path.join(os.path.dirname(__file__), 'env'))

from swimming_gym_env import SwimmingGymEnv

class PhaseOscillatorNetwork:
    """
    Phase oscillator network exactly as implemented in agnathax-simulation.
    Based on cpg_single_chain.cdn file.
    """
    def __init__(self, 
                 num_joints=10,
                 freq=5.0,
                 w_up=10.0,
                 w_down=30.0,
                 prop_fb=0.0,
                 ext_fb=0.0,
                 kappa=0.0,
                 nb_fb=0.0,
                 tau=0.05,
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
                 num_joints=10,
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


def test_cpg_network(cpg_model_type=0):
    """
    Test CPG network with selectable oscillator type.
    
    Args:
        cpg_model_type (int): 0 for phase oscillator, 1 for Matsuoka oscillator
    """
    if cpg_model_type == 0:
        print("Testing Phase Oscillator Network...")
        model_name = "phase_oscillator"
    else:
        print("Testing Matsuoka Oscillator Network...")
        model_name = "matsuoka"
    
    # Simulation parameters exactly as in agnathax
    TIME_STEP = 0.005  # 5ms as in agnathax
    SECONDS_OF_SIMULATION = 10  # 10s as in agnathax
    TEST_STEPS = int(SECONDS_OF_SIMULATION / TIME_STEP)
    t = np.arange(TEST_STEPS) * TIME_STEP
    
    # Initialize swimming environment
    env = SwimmingGymEnv(
        render=True,
        on_rack=False,
        isRLGymInterface=False,
        time_step=TIME_STEP,
        action_repeat=1,
        motor_control_mode="PD",
        add_noise=False,
        record_video=True
    )
    
    # Initialize CPG network based on model type
    if cpg_model_type == 0:
        # Phase oscillator network
        cpg = PhaseOscillatorNetwork(
            num_joints=6,   # 6 joints to match robot
            freq=1.0,       # 1 Hz
            w_up=10.0,      # W_UP = 10
            w_down=30.0,    # W_DOWN = 30
            prop_fb=0.0,    # PROP_FB = 0
            ext_fb=0.0,     # EXT_FB = 0
            kappa=0.0,      # KAPPA = 0
            nb_fb=0.0,      # NB_FB = 0
            tau=0.01,       # TAU = 0.01
            time_step=TIME_STEP
        )
    else:
        # Matsuoka oscillator network
        cpg = MatsuokaNetwork(
            num_joints=6,    # 6 joints to match robot
            beta=5.0,         # Beta = 5.0
            eta=-3.0,         # Eta = -3.0
            gamma=3.0,        # Gamma = 3.0
            eta_i=-2.0,       # Eta_i = -2
            eta_e=1.0,        # Eta_e = 1
            eta_head=0.0,     # Eta_head = 0
            tonic=50.0,       # Tonic = 50
            force=0.0,        # Force = 0.0
            tau=0.05,         # Tau = 0.05
            fb_i=-10.0,       # FB_I = -10
            fb_e=1.0,         # FB_E = 1
            time_step=TIME_STEP
        )
    
    # Storage arrays
    joint_angles = np.zeros((6, TEST_STEPS))
    joint_velocities = np.zeros((6, TEST_STEPS))
    
    if cpg_model_type == 0:
        # Phase oscillator states
        cpg_theta = np.zeros((6, TEST_STEPS))
        cpg_theta_dot = np.zeros((6, TEST_STEPS))
        cpg_x = np.zeros((6, TEST_STEPS))
    else:
        # Matsuoka oscillator states
        cpg_x_a = np.zeros((6, TEST_STEPS))
        cpg_x_b = np.zeros((6, TEST_STEPS))
        cpg_xx_a = np.zeros((6, TEST_STEPS))
        cpg_xx_b = np.zeros((6, TEST_STEPS))
    
    # Robot states storage
    robot_positions = np.zeros((TEST_STEPS, 3))
    robot_velocities = np.zeros((TEST_STEPS, 3))
    
    print(f"Running simulation for {SECONDS_OF_SIMULATION} seconds...")
    if cpg_model_type == 0:
        print(f"CPG frequency: {cpg.freq:.2f} Hz")
        print(f"Number of joints: {cpg.num_joints}")
        print(f"Phase lag: {cpg.phi_lag:.3f} rad")
    else:
        print(f"Number of joints: {cpg.num_joints}")
        print(f"Beta: {cpg.beta}, Eta: {cpg.eta}, Gamma: {cpg.gamma}")
        print(f"Tonic: {cpg.tonic}, Tau: {cpg.tau}")
    print(f"🎥 Video recording enabled - {model_name}_video.mp4 will be generated")
    
    # Run simulation
    for i in range(TEST_STEPS):
        # Update CPG
        desired_joint_angles = cpg.update()
        
        # Get CPG states based on model type
        if cpg_model_type == 0:
            cpg_theta[:, i] = cpg.get_theta()
            cpg_theta_dot[:, i] = cpg.get_theta_dot()
            cpg_x[:, i] = cpg.get_x()
        else:
            cpg_x_a[:, i] = cpg.get_x_a()
            cpg_x_b[:, i] = cpg.get_x_b()
            cpg_xx_a[:, i] = cpg.get_xx_a()
            cpg_xx_b[:, i] = cpg.get_xx_b()
        
        # Get current robot state
        current_joint_angles = env.get_joint_angles()
        current_joint_velocities = env.get_joint_velocities()
        
        # Save current states
        joint_angles[:, i] = current_joint_angles
        joint_velocities[:, i] = current_joint_velocities
        
        # Get robot pose and velocity
        base_pos = env.get_base_position()
        base_vel = env.get_base_velocity()
        
        robot_positions[i, :] = base_pos
        robot_velocities[i, :] = base_vel
        
        # Apply PD control to achieve desired joint angles
        action = desired_joint_angles
        
        # Step the environment
        env.step(action)
        
        # Print progress
        if i % 1000 == 0:
            print(f"Step {i}/{TEST_STEPS} - Time: {i*TIME_STEP:.2f}s - Forward velocity: {base_vel[0]:.3f} m/s")
    
    print("Simulation completed!")
    print(f"🎥 Video saved as: {model_name}_video.mp4")
    
    # Close environment
    env.close()
    
    # Create plots
    fig, axes = plt.subplots(3, 2, figsize=(15, 12))
    if cpg_model_type == 0:
        fig.suptitle('Phase Oscillator Network Test Results', fontsize=16)
    else:
        fig.suptitle('Matsuoka Oscillator Network Test Results', fontsize=16)
    
    # Plot 1: Joint angles over time
    axes[0, 0].plot(t, joint_angles.T)
    axes[0, 0].set_title('Joint Angles Over Time')
    axes[0, 0].set_xlabel('Time (s)')
    axes[0, 0].set_ylabel('Angle (rad)')
    axes[0, 0].legend([f'Joint {i+1}' for i in range(6)])
    axes[0, 0].grid(True)
    
    if cpg_model_type == 0:
        # Plot 2: CPG phase states
        axes[0, 1].plot(t, cpg_theta.T)
        axes[0, 1].set_title('CPG Phase States (θ)')
        axes[0, 1].set_xlabel('Time (s)')
        axes[0, 1].set_ylabel('Phase (rad)')
        axes[0, 1].legend([f'Joint {i+1}' for i in range(6)])
        axes[0, 1].grid(True)
        
        # Plot 3: CPG phase derivatives
        axes[1, 0].plot(t, cpg_theta_dot.T)
        axes[1, 0].set_title('CPG Phase Derivatives (θ_dot)')
        axes[1, 0].set_xlabel('Time (s)')
        axes[1, 0].set_ylabel('Phase Rate (rad/s)')
        axes[1, 0].legend([f'Joint {i+1}' for i in range(6)])
        axes[1, 0].grid(True)
        
        # Plot 4: CPG output x
        axes[1, 1].plot(t, cpg_x.T)
        axes[1, 1].set_title('CPG Output (x = cos(θ))')
        axes[1, 1].set_xlabel('Time (s)')
        axes[1, 1].set_ylabel('Output')
        axes[1, 1].legend([f'Joint {i+1}' for i in range(6)])
        axes[1, 1].grid(True)
    else:
        # Plot 2: CPG neuron A states
        axes[0, 1].plot(t, cpg_x_a.T)
        axes[0, 1].set_title('CPG Neuron A States (x_a)')
        axes[0, 1].set_xlabel('Time (s)')
        axes[0, 1].set_ylabel('State')
        axes[0, 1].legend([f'Joint {i+1}' for i in range(6)])
        axes[0, 1].grid(True)
        
        # Plot 3: CPG neuron B states
        axes[1, 0].plot(t, cpg_x_b.T)
        axes[1, 0].set_title('CPG Neuron B States (x_b)')
        axes[1, 0].set_xlabel('Time (s)')
        axes[1, 0].set_ylabel('State')
        axes[1, 0].legend([f'Joint {i+1}' for i in range(6)])
        axes[1, 0].grid(True)
        
        # Plot 4: CPG rectified outputs
        axes[1, 1].plot(t, cpg_xx_a.T, label='xx_a')
        axes[1, 1].plot(t, cpg_xx_b.T, label='xx_b')
        axes[1, 1].set_title('CPG Rectified Outputs')
        axes[1, 1].set_xlabel('Time (s)')
        axes[1, 1].set_ylabel('Output')
        axes[1, 1].legend()
        axes[1, 1].grid(True)
    
    # Plot 5: Robot position
    axes[2, 0].plot(t, robot_positions[:, 0], label='X position')
    axes[2, 0].plot(t, robot_positions[:, 1], label='Y position')
    axes[2, 0].plot(t, robot_positions[:, 2], label='Z position')
    axes[2, 0].set_title('Robot Position')
    axes[2, 0].set_xlabel('Time (s)')
    axes[2, 0].set_ylabel('Position (m)')
    axes[2, 0].legend()
    axes[2, 0].grid(True)
    
    # Plot 6: Robot velocity
    axes[2, 1].plot(t, robot_velocities[:, 0], label='X velocity')
    axes[2, 1].plot(t, robot_velocities[:, 1], label='Y velocity')
    axes[2, 1].plot(t, robot_velocities[:, 2], label='Z velocity')
    axes[2, 1].set_title('Robot Velocity')
    axes[2, 1].set_xlabel('Time (s)')
    axes[2, 1].set_ylabel('Velocity (m/s)')
    axes[2, 1].legend()
    axes[2, 1].grid(True)
    
    plt.tight_layout()
    plt.savefig(f'{model_name}_test.png', dpi=300, bbox_inches='tight')
    plt.show()
    
    # Analyze results
    print("\n=== Analysis ===")
    
    if cpg_model_type == 0:
        # Check phase differences between adjacent joints
        avg_phase_diffs = []
        for i in range(5):
            phase_diff = np.mean(cpg_theta[i+1, 1000:] - cpg_theta[i, 1000:])
            avg_phase_diffs.append(phase_diff)
            print(f"Average phase difference between joint {i+2} and {i+1}: {phase_diff:.3f} rad")
        
        # Check frequency
        from scipy.signal import find_peaks
        peaks, _ = find_peaks(cpg_x[0, 1000:], height=0.1)
        if len(peaks) > 1:
            peak_times = t[1000 + peaks]
            periods = np.diff(peak_times)
            avg_period = np.mean(periods)
            estimated_freq = 1.0 / avg_period
            print(f"Estimated frequency from peaks: {estimated_freq:.2f} Hz")
        
        # Check expected vs measured phase difference
        expected_phase_diff = 2 * np.pi / cpg.num_joints
        print(f"Expected phase difference per joint: {expected_phase_diff:.3f} rad")
        print(f"Average measured phase difference: {np.mean(avg_phase_diffs):.3f} rad")
    else:
        # Matsuoka oscillator analysis
        print(f"Final neuron A states: {cpg_x_a[:, -1000:].mean(axis=1)}")
        print(f"Final neuron B states: {cpg_x_b[:, -1000:].mean(axis=1)}")
        print(f"Final rectified A outputs: {cpg_xx_a[:, -1000:].mean(axis=1)}")
        print(f"Final rectified B outputs: {cpg_xx_b[:, -1000:].mean(axis=1)}")
    
    print(f"\n{model_name.replace('_', ' ').title()} test completed successfully!")
    print(f"Check '{model_name}_test.png' for visualization.")


def test_old_matsuoka():
    """Test the agnathax Matsuoka oscillator network."""
    print("Testing Agnathax Matsuoka Oscillator Network...")
    
    # Simulation parameters exactly as in agnathax
    TIME_STEP = 0.005  # 5ms as in agnathax
    SECONDS_OF_SIMULATION = 10  # 10s as in agnathax
    TEST_STEPS = int(SECONDS_OF_SIMULATION / TIME_STEP)
    t = np.arange(TEST_STEPS) * TIME_STEP
    
    # Initialize swimming environment
    env = SwimmingGymEnv(
        render=True,
        on_rack=False,
        isRLGymInterface=False,
        time_step=TIME_STEP,
        action_repeat=1,
        motor_control_mode="PD",
        add_noise=False,
        record_video=True
    )
    
    # Initialize agnathax Matsuoka oscillator network
    cpg = AgnathaxMatsuokaNetwork(
        num_joints=10,    # 10 joints as in agnathax
        beta=5.0,         # Beta = 5.0
        eta=-3.0,         # Eta = -3.0
        gamma=3.0,        # Gamma = 3.0
        eta_i=-2.0,       # Eta_i = -2
        eta_e=1.0,        # Eta_e = 1
        eta_head=0.0,     # Eta_head = 0
        tonic=50.0,       # Tonic = 50
        force=0.0,        # Force = 0.0
        tau=0.05,         # Tau = 0.05
        fb_i=-10.0,       # FB_I = -10
        fb_e=1.0,         # FB_E = 1
        time_step=TIME_STEP
    )
    
    # Storage arrays
    joint_angles = np.zeros((10, TEST_STEPS))
    joint_velocities = np.zeros((10, TEST_STEPS))
    cpg_x_a = np.zeros((10, TEST_STEPS))
    cpg_x_b = np.zeros((10, TEST_STEPS))
    cpg_xx_a = np.zeros((10, TEST_STEPS))
    cpg_xx_b = np.zeros((10, TEST_STEPS))
    
    # Robot states storage
    robot_positions = np.zeros((TEST_STEPS, 3))
    robot_velocities = np.zeros((TEST_STEPS, 3))
    
    print(f"Running simulation for {SECONDS_OF_SIMULATION} seconds...")
    print(f"Number of joints: {cpg.num_joints}")
    print(f"Beta: {cpg.beta}, Eta: {cpg.eta}, Gamma: {cpg.gamma}")
    print(f"Tonic: {cpg.tonic}, Tau: {cpg.tau}")
    print("🎥 Video recording enabled - agnathax_matsuoka_video.mp4 will be generated")
    
    # Run simulation
    for i in range(TEST_STEPS):
        # Update CPG
        desired_joint_angles = cpg.update()
        
        # Get CPG states
        cpg_x_a[:, i] = cpg.get_x_a()
        cpg_x_b[:, i] = cpg.get_x_b()
        cpg_xx_a[:, i] = cpg.get_xx_a()
        cpg_xx_b[:, i] = cpg.get_xx_b()
        
        # Get current robot state
        current_joint_angles = env.get_joint_angles()
        current_joint_velocities = env.get_joint_velocities()
        
        # Save current states
        joint_angles[:, i] = current_joint_angles
        joint_velocities[:, i] = current_joint_velocities
        
        # Get robot pose and velocity
        base_pos = env.get_base_position()
        base_vel = env.get_base_velocity()
        
        robot_positions[i, :] = base_pos
        robot_velocities[i, :] = base_vel
        
        # Apply PD control to achieve desired joint angles
        action = desired_joint_angles
        
        # Step the environment
        env.step(action)
        
        # Print progress
        if i % 1000 == 0:
            print(f"Step {i}/{TEST_STEPS} - Time: {i*TIME_STEP:.2f}s - Forward velocity: {base_vel[0]:.3f} m/s")
    
    print("Simulation completed!")
    print("🎥 Video saved as: agnathax_matsuoka_video.mp4")
    
    # Close environment
    env.close()
    
    # Create plots
    fig, axes = plt.subplots(3, 2, figsize=(15, 12))
    fig.suptitle('Agnathax Matsuoka Oscillator Network Test Results', fontsize=16)
    
    # Plot 1: Joint angles over time
    axes[0, 0].plot(t, joint_angles.T)
    axes[0, 0].set_title('Joint Angles Over Time')
    axes[0, 0].set_xlabel('Time (s)')
    axes[0, 0].set_ylabel('Angle (rad)')
    axes[0, 0].legend([f'Joint {i+1}' for i in range(10)])
    axes[0, 0].grid(True)
    
    # Plot 2: CPG neuron A states
    axes[0, 1].plot(t, cpg_x_a.T)
    axes[0, 1].set_title('CPG Neuron A States (x_a)')
    axes[0, 1].set_xlabel('Time (s)')
    axes[0, 1].set_ylabel('State')
    axes[0, 1].legend([f'Joint {i+1}' for i in range(10)])
    axes[0, 1].grid(True)
    
    # Plot 3: CPG neuron B states
    axes[1, 0].plot(t, cpg_x_b.T)
    axes[1, 0].set_title('CPG Neuron B States (x_b)')
    axes[1, 0].set_xlabel('Time (s)')
    axes[1, 0].set_ylabel('State')
    axes[1, 0].legend([f'Joint {i+1}' for i in range(10)])
    axes[1, 0].grid(True)
    
    # Plot 4: CPG rectified outputs
    axes[1, 1].plot(t, cpg_xx_a.T, label='xx_a')
    axes[1, 1].plot(t, cpg_xx_b.T, label='xx_b')
    axes[1, 1].set_title('CPG Rectified Outputs')
    axes[1, 1].set_xlabel('Time (s)')
    axes[1, 1].set_ylabel('Output')
    axes[1, 1].legend()
    axes[1, 1].grid(True)
    
    # Plot 5: Robot position
    axes[2, 0].plot(t, robot_positions[:, 0], label='X position')
    axes[2, 0].plot(t, robot_positions[:, 1], label='Y position')
    axes[2, 0].plot(t, robot_positions[:, 2], label='Z position')
    axes[2, 0].set_title('Robot Position')
    axes[2, 0].set_xlabel('Time (s)')
    axes[2, 0].set_ylabel('Position (m)')
    axes[2, 0].legend()
    axes[2, 0].grid(True)
    
    # Plot 6: Robot velocity
    axes[2, 1].plot(t, robot_velocities[:, 0], label='X velocity')
    axes[2, 1].plot(t, robot_velocities[:, 1], label='Y velocity')
    axes[2, 1].plot(t, robot_velocities[:, 2], label='Z velocity')
    axes[2, 1].set_title('Robot Velocity')
    axes[2, 1].set_xlabel('Time (s)')
    axes[2, 1].set_ylabel('Velocity (m/s)')
    axes[2, 1].legend()
    axes[2, 1].grid(True)
    
    plt.tight_layout()
    plt.savefig('agnathax_matsuoka_test.png', dpi=300, bbox_inches='tight')
    plt.show()
    
    print("\nAgnathax Matsuoka Oscillator test completed successfully!")
    print("Check 'agnathax_matsuoka_test.png' for visualization.")


if __name__ == "__main__":
    print("CPG Implementation")
    print("0: Phase Oscillator Network")
    print("1: Matsuoka Oscillator Network")
    
    # You can change this parameter to switch between models
    cpg_model_type = 0  # 0 for phase oscillator, 1 for Matsuoka oscillator
    
    print(f"Running {['Phase Oscillator', 'Matsuoka Oscillator'][cpg_model_type]} Network test...")
    test_cpg_network(cpg_model_type)
