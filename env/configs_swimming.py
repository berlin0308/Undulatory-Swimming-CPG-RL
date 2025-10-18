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

"""Configuration parameters for swimming robot."""

import numpy as np


class SwimmingConfig:
  """Configuration for swimming eel-like robot."""
  
  def __init__(self):
    # Robot dimensions (from URDF)
    self.BASE_LENGTH = 0.2796  # length of base link
    self.LINK_LENGTH = 0.1398  # length of each link
    self.TOTAL_LENGTH = self.BASE_LENGTH + 2 * self.LINK_LENGTH  # total robot length
    
    # Joint parameters
    self.NUM_JOINTS = 6  # number of revolute joints (6-joint swimming robot)
    self.JOINT_LIMITS = np.array([[-np.pi, np.pi]] * self.NUM_JOINTS)  # joint limits
    
    # Motor control parameters
    self.MOTOR_KP = np.array([100.0] * self.NUM_JOINTS)  # position gains
    self.MOTOR_KD = np.array([10.0] * self.NUM_JOINTS)   # velocity gains
    self.MOTOR_TORQUE_LIMITS = np.array([5.0] * self.NUM_JOINTS)  # torque limits
    
    # Swimming parameters
    self.SWIMMING_FREQUENCY = 2.0  # Hz, swimming frequency
    self.WAVE_AMPLITUDE = 0.3      # rad, amplitude of joint angles
    self.WAVE_LENGTH = 2.0         # wavelength of traveling wave
    
    # Physics parameters
    self.MASS = 0.75  # kg, total robot mass
    self.WATER_DENSITY = 1000.0  # kg/m^3
    self.DRAG_COEFFICIENT = 1.0  # dimensionless drag coefficient
    
    # Control parameters
    self.TIME_STEP = 0.001  # simulation time step
    self.ACTION_REPEAT = 1  # number of simulation steps per action
    
    # Observation parameters
    self.OBSERVATION_NOISE_STD = 0.01  # standard deviation of observation noise
    
    # Reward parameters
    self.FORWARD_VELOCITY_WEIGHT = 1.0  # weight for forward velocity reward
    self.ENERGY_PENALTY_WEIGHT = 0.1    # weight for energy consumption penalty
