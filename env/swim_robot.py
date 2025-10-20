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

"""Swimming robot class for eel-like locomotion."""

import numpy as np
import pybullet as p


class SwimmingRobot:
  """Swimming robot class for eel-like locomotion."""
  
  def __init__(self, pybullet_client, robot_id, config):
    """Initialize swimming robot.
    
    Args:
      pybullet_client: PyBullet client instance
      robot_id: Robot ID in PyBullet simulation
      config: Robot configuration
    """
    self._pybullet_client = pybullet_client
    self._robot_id = robot_id
    self._config = config
    
    # Get joint information
    self._joint_ids = []
    self._joint_names = []
    self._joint_lower_limits = []
    self._joint_upper_limits = []
    
    for i in range(self._pybullet_client.getNumJoints(self._robot_id)):
      joint_info = self._pybullet_client.getJointInfo(self._robot_id, i)
      if joint_info[2] == self._pybullet_client.JOINT_REVOLUTE:
        self._joint_ids.append(i)
        self._joint_names.append(joint_info[1].decode('utf-8'))
        self._joint_lower_limits.append(joint_info[8])
        self._joint_upper_limits.append(joint_info[9])
    
    self._num_joints = len(self._joint_ids)
    
    # Initialize joint states
    self._joint_angles = np.zeros(self._num_joints)
    self._joint_velocities = np.zeros(self._num_joints)
    
  def get_joint_angles(self):
    """Get current joint angles."""
    joint_states = self._pybullet_client.getJointStates(self._robot_id, self._joint_ids)
    self._joint_angles = np.array([state[0] for state in joint_states])
    return self._joint_angles.copy()
  
  def get_joint_velocities(self):
    """Get current joint velocities."""
    joint_states = self._pybullet_client.getJointStates(self._robot_id, self._joint_ids)
    self._joint_velocities = np.array([state[1] for state in joint_states])
    return self._joint_velocities.copy()
  
  def get_base_position(self):
    """Get robot base position."""
    base_pos, _ = self._pybullet_client.getBasePositionAndOrientation(self._robot_id)
    return np.array(base_pos)
  
  def get_base_orientation(self):
    """Get robot base orientation."""
    _, base_orn = self._pybullet_client.getBasePositionAndOrientation(self._robot_id)
    return np.array(base_orn)
  
  def get_base_velocity(self):
    """Get robot base velocity."""
    base_vel, _ = self._pybullet_client.getBaseVelocity(self._robot_id)
    return np.array(base_vel)
  
  def get_base_angular_velocity(self):
    """Get robot base angular velocity."""
    _, base_ang_vel = self._pybullet_client.getBaseVelocity(self._robot_id)
    return np.array(base_ang_vel)
  
  def set_joint_angles(self, joint_angles):
    """Set joint angles."""
    if len(joint_angles) != self._num_joints:
      raise ValueError(f"Expected {self._num_joints} joint angles, got {len(joint_angles)}")
    
    for i, joint_id in enumerate(self._joint_ids):
      self._pybullet_client.resetJointState(self._robot_id, joint_id, joint_angles[i], 0)
  
  def apply_joint_torques(self, torques):
    """Apply joint torques."""
    if len(torques) != self._num_joints:
      raise ValueError(f"Expected {self._num_joints} torques, got {len(torques)}")
    
    self._pybullet_client.setJointMotorControlArray(
        self._robot_id,
        self._joint_ids,
        self._pybullet_client.TORQUE_CONTROL,
        forces=torques)
  
  def apply_joint_positions(self, positions, kp=None, kd=None):
    """Apply joint position control."""
    if len(positions) != self._num_joints:
      raise ValueError(f"Expected {self._num_joints} positions, got {len(positions)}")
    
    if kp is None:
      kp = self._config.MOTOR_KP
    if kd is None:
      kd = self._config.MOTOR_KD
    
    self._pybullet_client.setJointMotorControlArray(
        self._robot_id,
        self._joint_ids,
        self._pybullet_client.POSITION_CONTROL,
        targetPositions=positions,
        forces=self._config.MOTOR_TORQUE_LIMITS,
        positionGains=kp,
        velocityGains=kd)
  
  def get_joint_torques(self):
    """Get current joint torques."""
    joint_states = self._pybullet_client.getJointStates(self._robot_id, self._joint_ids)
    return np.array([state[3] for state in joint_states])
  
  def get_joint_info(self):
    """Get joint information."""
    return {
        'ids': self._joint_ids,
        'names': self._joint_names,
        'lower_limits': self._joint_lower_limits,
        'upper_limits': self._joint_upper_limits,
        'num_joints': self._num_joints
    }
  
  def compute_swimming_velocity(self):
    """Compute swimming velocity from joint motion."""
    # Simple model: swimming velocity proportional to joint velocity magnitude
    joint_vel = self.get_joint_velocities()
    swimming_velocity = np.linalg.norm(joint_vel) * 0.1  # scaling factor
    return swimming_velocity
  
  def compute_energy_consumption(self):
    """Compute energy consumption from joint torques and velocities."""
    torques = self.get_joint_torques()
    velocities = self.get_joint_velocities()
    power = np.sum(np.abs(torques * velocities))
    return power
