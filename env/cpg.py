#!/usr/bin/env python3
"""
CPG (Central Pattern Generator) models for swimming robot control.

Provides two oscillator models:
- PhaseOscillatorNetwork: Simple phase-based oscillator
- MatsuokaNetwork: Biologically-inspired Matsuoka oscillator
"""

import numpy as np


class PhaseOscillatorNetwork:
    """Phase oscillator-based CPG network for rhythmic joint control."""
    
    def __init__(self, num_joints: int, alpha: float = 1.5, freq: float = 1.0,
                 couple: bool = True, couple_w_forward: float = 1.0, couple_w_backward: float = 1.0,
                 total_phase_offset: float = 2.0 * np.pi, dt: float = 0.01):
        """
        Initialize phase oscillator network.

        Args:
            num_joints: Number of joints to control
            alpha: Coupling strength between oscillators
            freq: Base frequency in Hz
            couple: Enable oscillator coupling (default: True)
            couple_w_forward: Forward coupling weight (i -> i+1)
            couple_w_backward: Backward coupling weight (i -> i-1)
            total_phase_offset: Total phase span across all joints (default: 2π)
            dt: Time step for integration (should match environment dt)
        """
        self.num_joints = num_joints
        self.alpha = alpha
        self.freq = freq
        self.couple = couple
        self.couple_w_forward = couple_w_forward
        self.couple_w_backward = couple_w_backward
        self.total_phase_offset = total_phase_offset
        self.dt = dt  # Use provided dt instead of hardcoded value

        # State variables
        self.theta = np.zeros(num_joints, dtype=np.float32)
        self.omega = 2.0 * np.pi * freq  # Angular frequency

        # Initialize with slight phase offsets for wave propagation
        for i in range(num_joints):
            self.theta[i] = i * (self.total_phase_offset / (num_joints + 1))
    
    def step(self, freq_mod: float = 0.0, amp_mod: float = 0.0, phase_shift: float = 0.0,
             w_forward: float = None, w_backward: float = None, total_phase_offset: float = None) -> np.ndarray:
        """
        Update oscillator phases and return joint angle commands.

        Args:
            freq_mod: Frequency modulation (multiplier, base=1.0)
            amp_mod: Amplitude modulation (additive, base=1.0)
            phase_shift: Global phase shift (radians)
            w_forward: Forward coupling weight (overrides default if provided)
            w_backward: Backward coupling weight (overrides default if provided)
            total_phase_offset: Total phase span (overrides default if provided)

        Returns:
            Array of joint angles (sinusoidal output)
        """
        # Apply frequency modulation (clamp to reasonable range)
        freq_multiplier = np.clip(1.0 + freq_mod, 0.5, 2.0)
        omega_modulated = self.omega * freq_multiplier

        # Use provided weights or fall back to defaults
        wf = w_forward if w_forward is not None else self.couple_w_forward
        wb = w_backward if w_backward is not None else self.couple_w_backward

        # Update total_phase_offset if provided (allows dynamic modulation)
        if total_phase_offset is not None:
            self.total_phase_offset = total_phase_offset

        # Compute phase updates
        if self.couple:
            # Phase coupling: Kuramoto-style nearest-neighbor coupling
            # dθ_i/dt = ω + Σ K_ij × sin(θ_j - θ_i)
            dtheta = np.zeros(self.num_joints, dtype=np.float32)

            for i in range(self.num_joints):
                coupling_term = 0.0

                # Forward coupling (i -> i+1)
                if i < self.num_joints - 1:
                    phase_diff = self.theta[i+1] - self.theta[i]
                    coupling_term += self.alpha * wf * np.sin(phase_diff)

                # Backward coupling (i-1 -> i)
                if i > 0:
                    phase_diff = self.theta[i-1] - self.theta[i]
                    coupling_term += self.alpha * wb * np.sin(phase_diff)

                dtheta[i] = omega_modulated + coupling_term

            self.theta += dtheta * self.dt
        else:
            # No coupling: independent oscillators
            self.theta += omega_modulated * self.dt

        self.theta = self.theta % (2 * np.pi)

        # Convert phases to joint angles with amplitude modulation and phase shift
        # Amplitude: base=1.0, clamp to [0.2, 1.5]
        amplitude = np.clip(1.0 + amp_mod, 0.2, 1.5)
        joint_angles = amplitude * np.sin(self.theta + phase_shift)

        return joint_angles.astype(np.float32)
    
    def reset(self):
        """Reset oscillator states with fixed wave pattern using arange."""
        # 使用 arange 產生固定的相位分佈
        self.theta = np.arange(self.num_joints, dtype=np.float32) * (self.total_phase_offset / (self.num_joints + 1))


class MatsuokaNetwork:
    """Matsuoka oscillator-based CPG network."""
    
    def __init__(self, num_joints: int, tau: float = 0.1, beta: float = 2.5):
        """
        Initialize Matsuoka oscillator network.
        
        Args:
            num_joints: Number of joints to control
            tau: Time constant
            beta: Mutual inhibition strength
        """
        self.num_joints = num_joints
        self.tau = tau
        self.beta = beta
        
        # State variables (flexor and extensor neurons)
        self.x_a = np.zeros(num_joints, dtype=np.float32)
        self.x_b = np.zeros(num_joints, dtype=np.float32)
        self.v_a = np.zeros(num_joints, dtype=np.float32)
        self.v_b = np.zeros(num_joints, dtype=np.float32)
        
        self.dt = 0.01  # Time step

        # Initialize with fixed wave pattern using arange (避免隨機性)
        phase = np.arange(num_joints, dtype=np.float32) * (2 * np.pi / num_joints)
        self.x_a[:] = 0.1 * np.sin(phase)
        self.x_b[:] = 0.1 * np.cos(phase)
    
    def step(self) -> np.ndarray:
        """
        Update Matsuoka oscillator states and return joint angles.
        
        Returns:
            Array of joint angles (difference between flexor and extensor)
        """
        # Tonic input
        tonic = 1.0
        
        # Adaptation dynamics
        dv_a = (-self.v_a + self.x_a) / (2.0 * self.tau)
        dv_b = (-self.v_b + self.x_b) / (2.0 * self.tau)
        
        # Neuron dynamics with mutual inhibition
        dx_a = (-self.x_a - self.beta * self.x_b - self.v_a + tonic) / self.tau
        dx_b = (-self.x_b - self.beta * self.x_a - self.v_b + tonic) / self.tau
        
        # Update states
        self.x_a += dx_a * self.dt
        self.x_b += dx_b * self.dt
        self.v_a += dv_a * self.dt
        self.v_b += dv_b * self.dt
        
        # Output is difference between flexor and extensor
        joint_angles = self.x_a - self.x_b
        
        return joint_angles.astype(np.float32)
    
    def reset(self):
        """Reset oscillator states with fixed wave pattern using arange."""
        # 使用 arange 產生固定的波形初始化，避免隨機性
        phase = np.arange(self.num_joints, dtype=np.float32) * (2 * np.pi / self.num_joints)
        self.x_a = 0.1 * np.sin(phase).astype(np.float32)
        self.x_b = 0.1 * np.cos(phase).astype(np.float32)
        self.v_a = np.zeros(self.num_joints, dtype=np.float32)
        self.v_b = np.zeros(self.num_joints, dtype=np.float32)