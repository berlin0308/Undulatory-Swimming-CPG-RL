#!/usr/bin/env python3
"""
CPG (Central Pattern Generator) implementations for swimming robot.
Implements both phase oscillator and Matsuoka oscillator models.
Based on agnathax-simulation implementation.
"""

import numpy as np

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
                 alpha=1.0,
                 time_step=0.005
                 ):
        
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
        self.alpha = alpha
        
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
        self.x = self.alpha * np.cos(self.theta)  # Calculate initial output
        
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
        
        # Calculate output x = alpha * cos(theta)
        self.x = self.alpha * np.cos(self.theta)
        
        return self.x.copy()
    
    def step(self):
        """Alias for update() method for compatibility."""
        return self.update()
    
    def get_theta(self):
        """Get current phase angles."""
        return self.theta.copy()
    
    def get_theta_dot(self):
        """Get current phase velocities."""
        return self.theta_dot.copy()
    
    def get_x(self):
        """Get current oscillator outputs."""
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
    
    def step(self):
        """Alias for update() method for compatibility."""
        return self.update()
    
    def get_x_a(self):
        """Get current neuron A states."""
        return self.x_a.copy()
    
    def get_x_b(self):
        """Get current neuron B states."""
        return self.x_b.copy()
    
    def get_xx_a(self):
        """Get current rectified neuron A outputs."""
        return self.xx_a.copy()
    
    def get_xx_b(self):
        """Get current rectified neuron B outputs."""
        return self.xx_b.copy()
