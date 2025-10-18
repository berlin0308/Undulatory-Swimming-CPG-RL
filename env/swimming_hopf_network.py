# SPDX-FileCopyrightText: Copyright (c) 2022 Guillaume Bellegarda. All rights reserved.
# SPDX-License-Identifier: BSD-3-Clause
# 
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
# 1. Redistributions of source code must retain the above copyright notice, this
# list of conditions and the following disclaimer.
#
# 2. Redistributions in binary form must reproduce the above copyright notice,
# this list of conditions and the following disclaimer in the documentation
# and/or other materials provided with the distribution.
#
# 3. Neither the name of the copyright holder nor the names of its
# contributors may be used to endorse or promote products derived from
# this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
# DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
# FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
# DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
# SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
# OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
# OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
#
# Copyright (c) 2022 EPFL, Guillaume Bellegarda

"""
Swimming CPG network based on Hopf oscillators for eel-like swimming motion.
Adapted from quadruped CPG for swimming locomotion with traveling wave pattern.
"""
import numpy as np

# for RL 
MU_LOW = 1
MU_UPP = 2


class SwimmingHopfNetwork():
    """ CPG network based on hopf polar equations for swimming locomotion.
    
    Joint Order is joint1, joint2, joint3 (from head to tail)
    Creates traveling wave pattern for swimming motion
    """
    def __init__(self,
                  mu=1**2,                 # intrinsic amplitude, converges to sqrt(mu)
                  omega_swim=2*2*np.pi,    # swimming frequency
                  alpha=50,                # amplitude convergence factor
                  coupling_strength=1,     # coefficient to multiply coupling matrix
                  couple=True,             # whether oscillators should be coupled
                  time_step=0.001,         # time step 
                  wave_length=2.0,         # wavelength of traveling wave
                  wave_amplitude=0.3,      # amplitude of joint angles
                  num_joints=3,            # number of joints
                  use_RL=False             # whether to learn parameters with RL  
                  ):
    
        ###############
        # initialize CPG data structures: amplitude is row 0, and phase is row 1
        self.X = np.zeros((2, num_joints))
        self.X_dot = np.zeros((2, num_joints))

        # save parameters 
        self._mu = mu
        self._omega_swim = omega_swim
        self._alpha = alpha
        self._couple = couple
        self._coupling_strength = coupling_strength
        self._dt = time_step
        self._wave_length = wave_length
        self._wave_amplitude = wave_amplitude
        self._num_joints = num_joints

        # set coupling matrix for traveling wave pattern
        self._set_swimming_coupling()

        # set oscillator initial conditions  
        self.X[0,:] = np.random.rand(num_joints) * .1
        self.X[1,:] = self._initial_phases

        # for RL
        self.use_RL = use_RL
        self._omega_rl = np.zeros(num_joints)
        self._mu_rl = np.zeros(num_joints) 
        if use_RL:
            self.X[0,:] = MU_LOW # mapping MU_LOW=1 to MU_UPP=2


    def _set_swimming_coupling(self):
        """ Set coupling matrix for swimming traveling wave pattern. """
        # Create traveling wave pattern: each joint is phase-shifted from the previous
        # Phase difference between adjacent joints for traveling wave
        phase_diff = 2 * np.pi / self._wave_length
        
        # Initialize coupling matrix
        self.PHI = np.zeros((self._num_joints, self._num_joints))
        
        # Set initial phases for traveling wave
        self._initial_phases = np.zeros(self._num_joints)
        for i in range(self._num_joints):
            self._initial_phases[i] = i * phase_diff
        
        # Set coupling between adjacent joints
        for i in range(self._num_joints):
            for j in range(self._num_joints):
                if i != j:
                    # Adjacent joints have stronger coupling
                    if abs(i - j) == 1:
                        self.PHI[i, j] = phase_diff
                    else:
                        # Non-adjacent joints have weaker coupling
                        self.PHI[i, j] = (i - j) * phase_diff * 0.5


    def update(self):
        """ Update oscillator states and return joint angles. """
        
        # update parameters, integrate
        if not self.use_RL:
            self._integrate_hopf_equations()
        else:
            self._integrate_hopf_equations_rl()
        
        # map CPG variables to joint angles for swimming motion
        # Create traveling wave pattern
        joint_angles = np.zeros(self._num_joints)
        
        for i in range(self._num_joints):
            # Use amplitude and phase to create sinusoidal joint motion
            # Amplitude modulates the strength of the wave
            amplitude = self.X[0, i] * self._wave_amplitude
            
            # Phase creates the traveling wave pattern
            phase = self.X[1, i]
            
            # Joint angle is sinusoidal with traveling wave pattern
            joint_angles[i] = amplitude * np.sin(phase)
        
        return joint_angles

      
        
    def _integrate_hopf_equations(self):
        """ Hopf polar equations and integration for swimming motion. """
        # bookkeeping - save copies of current CPG states 
        X = self.X.copy()
        X_dot_prev = self.X_dot.copy() 
        X_dot = np.zeros((2, self._num_joints))

        # loop through each joint's oscillator
        for i in range(self._num_joints):
            # get r_i, theta_i from X
            r, theta = X[0,i], X[1,i]
            
            # compute r_dot (amplitude dynamics)
            r_dot = self._alpha * (self._mu - r**2) * r
            
            # phase dynamics - constant frequency for swimming
            theta_dot = self._omega_swim

            # loop through other oscillators to add coupling
            if self._couple:
                for j in range(self._num_joints):
                    if i != j:
                        r_j, theta_j = X[0, j], X[1, j]
                        # Coupling creates traveling wave pattern
                        theta_dot += self._coupling_strength * r_j * np.sin(theta_j - theta - self.PHI[i, j])

            # set X_dot[:,i]
            X_dot[:,i] = [r_dot, theta_dot]

        # integrate (Euler)
        self.X = X + (X_dot_prev + X_dot) * self._dt / 2
        self.X_dot = X_dot
        # mod phase variables to keep between 0 and 2pi
        self.X[1,:] = self.X[1,:] % (2*np.pi)


    ###################### Helper functions for accessing CPG States
    def get_r(self):
        """ Get CPG amplitudes (r) """
        return self.X[0,:]

    def get_theta(self):
        """ Get CPG phases (theta) """
        return self.X[1,:]

    def get_dr(self):
        """ Get CPG amplitude derivatives (r_dot) """
        return self.X_dot[0,:]

    def get_dtheta(self):
        """ Get CPG phase derivatives (theta_dot) """
        return self.X_dot[1,:]

    ###################### Functions for setting parameters for RL
    def set_omega_rl(self, omegas):
        """ Set intrinsic frequencies. """
        self._omega_rl = omegas 

    def set_mu_rl(self, mus):
        """ Set intrinsic amplitude setpoints. """
        self._mu_rl = mus

    def _integrate_hopf_equations_rl(self):
        """ Hopf polar equations and integration, using quantities set by RL """
        # bookkeeping - save copies of current CPG states 
        X = self.X.copy()
        X_dot_prev = self.X_dot.copy() 
        X_dot = np.zeros((2, self._num_joints))

        # loop through each joint's oscillator, find current velocities
        for i in range(self._num_joints):
            # get r_i, theta_i from X
            r, theta = X[:,i]
            # amplitude (use mu from RL, i.e. self._mu_rl[i])
            r_dot = self._alpha * (self._mu_rl[i] - r**2) * r
            # phase (use omega from RL, i.e. self._omega_rl[i])
            theta_dot = self._omega_rl[i]

            X_dot[:,i] = [r_dot, theta_dot]

        # integrate 
        self.X = X + (X_dot_prev + X_dot) * self._dt / 2
        self.X_dot = X_dot
        self.X[1,:] = self.X[1,:] % (2*np.pi)
