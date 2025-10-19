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

"""This file implements the gym environment for a swimming eel-like robot. """
import os, inspect
# so we can import files
currentdir = os.path.dirname(os.path.abspath(inspect.getfile(inspect.currentframe())))
os.sys.path.insert(0, currentdir)

import time, datetime
import numpy as np
# gymnasium
import gymnasium as gym
from gymnasium import spaces
from gymnasium.utils import seeding
# pybullet
import pybullet
import pybullet_utils.bullet_client as bc
import pybullet_data
import random
random.seed(10)
# swimming robot and configs
import swimming_robot
import configs_swimming


class SwimmingGymEnv(gym.Env):
  """
  Swimming environment for eel-like robot with CPG control.
  """
  metadata = {"render.modes": ["human", "rgb_array"], "video.frames_per_second": 50}

  def __init__(self,
               urdf_root=pybullet_data.getDataPath(),
               time_step=0.01,
               action_repeat=1,
               isRLGymInterface=False,
               render=False,
               on_rack=False,
               motor_control_mode="PD",
               add_noise=False,
               record_video=False,
               **kwargs):
    """Initialize the swimming environment.

    Args:
      urdf_root: The path to the URDF data directory.
      time_step: The time step of the simulation.
      action_repeat: The number of simulation steps that an action is repeated.
      isRLGymInterface: If the gym interface is used for reinforcement learning.
      render: Whether to render the simulation.
      on_rack: Whether to place the robot on rack for debugging.
      motor_control_mode: Motor control mode (PD, TORQUE).
      add_noise: Whether to add noise to the observations.
      record_video: Whether to record video.
    """
    self._urdf_root = urdf_root
    self._time_step = time_step
    self._action_repeat = action_repeat
    self._isRLGymInterface = isRLGymInterface
    self._motor_control_mode = motor_control_mode
    self._add_noise = add_noise
    self._on_rack = on_rack
    self._render = render
    self._record_video = record_video

    # Load robot configuration
    self._robot_config = configs_swimming.SwimmingConfig()
    
    # Initialize PyBullet
    if self._render:
      self._pybullet_client = bc.BulletClient(connection_mode=pybullet.GUI)
    else:
      self._pybullet_client = bc.BulletClient()
    
    # Set up simulation
    self._pybullet_client.setAdditionalSearchPath(pybullet_data.getDataPath())
    self._pybullet_client.setGravity(0, 0, -9.81)
    
    # Add water plane for buoyancy
    self._add_water_plane()
    
    # Load robot
    self._load_robot()
    
    # Set up action and observation spaces
    self._set_action_space()
    self._set_observation_space()
    
    # Initialize CPG
    self._cpg = None
    
    # Video recording
    if self._record_video:
      self._video_log_id = self._pybullet_client.startStateLogging(
          self._pybullet_client.STATE_LOGGING_VIDEO_MP4, "swimming_6joints_video.mp4")

  def _add_water_plane(self):
    """Add water plane for buoyancy effects."""
    # Create a large water plane
    water_plane = self._pybullet_client.createCollisionShape(
        self._pybullet_client.GEOM_PLANE)
    water_body = self._pybullet_client.createMultiBody(
        baseMass=0,
        baseCollisionShapeIndex=water_plane,
        basePosition=[0, 0, -0.5],  # Water surface at z=-0.5 (deeper water)
        baseOrientation=[0, 0, 0, 1])
    
    # Set water properties
    self._pybullet_client.changeDynamics(
        water_body, -1,
        lateralFriction=0.1,
        restitution=0.0)

  def _load_robot(self):
    """Load the swimming robot."""
    # Force load 6-joint version - fix path to go up one directory
    urdf_path = os.path.join(os.path.dirname(currentdir), "assem_description/urdf/assem_description_6joints.urdf")
    print(f"Loading 6-joint swimming robot URDF from: {urdf_path}")
    
    if self._on_rack:
      # Place robot on rack for debugging
      self.robot = self._pybullet_client.loadURDF(
          urdf_path,
          [0, 0, 1.0],  # position
          [0, 0, 0, 1],  # orientation
          useFixedBase=True)
    else:
      # Place robot submerged in water
      self.robot = self._pybullet_client.loadURDF(
          urdf_path,
          [0, 0, -0.2],  # position submerged in water (swimming depth)
          [0, 0, 0, 1],  # orientation
          useFixedBase=False)
    
    # Get number of joints
    self._num_joints = self._pybullet_client.getNumJoints(self.robot)
    print(f"Total joints in robot: {self._num_joints}")
    
    # Set joint limits and control parameters
    self._setup_joints()

  def _setup_joints(self):
    """Setup joint parameters and control."""
    self._joint_ids = []
    self._joint_names = []
    self._joint_lower_limits = []
    self._joint_upper_limits = []
    
    for i in range(self._num_joints):
      joint_info = self._pybullet_client.getJointInfo(self.robot, i)
      print(f"Joint {i}: {joint_info[1].decode('utf-8')}, type: {joint_info[2]}")
      if joint_info[2] == self._pybullet_client.JOINT_REVOLUTE:
        self._joint_ids.append(i)
        self._joint_names.append(joint_info[1].decode('utf-8'))
        self._joint_lower_limits.append(joint_info[8])
        self._joint_upper_limits.append(joint_info[9])
    
    print(f"Found {len(self._joint_ids)} revolute joints: {self._joint_names}")
    
    # Set joint control parameters
    self._pybullet_client.setJointMotorControlArray(
        self.robot,
        self._joint_ids,
        self._pybullet_client.POSITION_CONTROL,
        forces=[0] * len(self._joint_ids))

  def _set_action_space(self):
    """Set action space based on motor control mode."""
    if self._motor_control_mode == "PD":
      # PD control: joint angles
      self.action_space = spaces.Box(
          low=np.array([-np.pi] * len(self._joint_ids)),
          high=np.array([np.pi] * len(self._joint_ids)),
          dtype=np.float32)
    elif self._motor_control_mode == "TORQUE":
      # Torque control: joint torques
      self.action_space = spaces.Box(
          low=np.array([-10.0] * len(self._joint_ids)),
          high=np.array([10.0] * len(self._joint_ids)),
          dtype=np.float32)
    else:
      raise ValueError("Unknown motor control mode: {}".format(self._motor_control_mode))

  def _set_observation_space(self):
    """Set observation space."""
    # Observation includes joint angles, joint velocities, and base pose
    obs_dim = (len(self._joint_ids) * 2 +  # joint angles and velocities
               7)  # base position (3) + orientation (4)
    
    self.observation_space = spaces.Box(
        low=-np.inf,
        high=np.inf,
        shape=(obs_dim,),
        dtype=np.float32)

  def reset(self, seed=None, options=None):
    """Reset the environment."""
    super().reset(seed=seed)
    
    # Reset robot to initial position
    if self._on_rack:
      self._pybullet_client.resetBasePositionAndOrientation(
          self.robot, [0, 0, 1.0], [0, 0, 0, 1])
    else:
      self._pybullet_client.resetBasePositionAndOrientation(
          self.robot, [0, 0, 0.5], [0, 0, 0, 1])
    
    # Reset joint states
    for i, joint_id in enumerate(self._joint_ids):
      self._pybullet_client.resetJointState(self.robot, joint_id, 0, 0)
    
    # Reset CPG if exists
    if self._cpg is not None:
      self._cpg = None
    
    # Step simulation to stabilize
    for _ in range(10):
      self._pybullet_client.stepSimulation()
    
    observation = self._get_observation()
    info = {}
    
    return observation, info

  def step(self, action):
    """Step the simulation."""
    # Apply action
    if self._motor_control_mode == "PD":
      self._pybullet_client.setJointMotorControlArray(
          self.robot,
          self._joint_ids,
          self._pybullet_client.POSITION_CONTROL,
          targetPositions=action,
          forces=[10.0] * len(self._joint_ids))
    elif self._motor_control_mode == "TORQUE":
      self._pybullet_client.setJointMotorControlArray(
          self.robot,
          self._joint_ids,
          self._pybullet_client.TORQUE_CONTROL,
          forces=action)
    
    # Step simulation
    for _ in range(self._action_repeat):
      # Apply buoyancy force
      self._apply_buoyancy()
      self._pybullet_client.stepSimulation()
      if self._render:
        time.sleep(self._time_step)
    
    observation = self._get_observation()
    reward = self._get_reward()
    terminated = False
    truncated = False
    info = {}
    
    return observation, reward, terminated, truncated, info

  def _get_observation(self):
    """Get current observation."""
    # Check if robot still exists
    try:
      if not self._pybullet_client.getNumBodies():
        # Robot was deleted, return zero observation
        joint_angles = [0.0] * len(self._joint_ids)
        joint_velocities = [0.0] * len(self._joint_ids)
        base_pos = [0.0, 0.0, 0.0]
        base_orn = [0.0, 0.0, 0.0, 1.0]
      else:
        # Get joint states
        joint_states = self._pybullet_client.getJointStates(self.robot, self._joint_ids)
        joint_angles = [state[0] for state in joint_states]
        joint_velocities = [state[1] for state in joint_states]
        
        # Get base pose
        try:
          base_pos, base_orn = self._pybullet_client.getBasePositionAndOrientation(self.robot)
        except:
          base_pos = [0.0, 0.0, 0.0]
          base_orn = [0.0, 0.0, 0.0, 1.0]
    except:
      # Fallback if anything goes wrong
      joint_angles = [0.0] * len(self._joint_ids)
      joint_velocities = [0.0] * len(self._joint_ids)
      base_pos = [0.0, 0.0, 0.0]
      base_orn = [0.0, 0.0, 0.0, 1.0]
    
    # Combine observations
    observation = np.concatenate([
        joint_angles,
        joint_velocities,
        base_pos,
        base_orn
    ])
    
    if self._add_noise:
      observation += np.random.normal(0, 0.01, observation.shape)
    
    return observation.astype(np.float32)

  def _get_reward(self):
    """Calculate reward for swimming performance."""
    # Simple reward: forward velocity
    base_vel, _ = self._pybullet_client.getBaseVelocity(self.robot)
    forward_velocity = base_vel[0]  # x-direction velocity
    
    # Reward forward swimming
    reward = forward_velocity
    
    return reward

  def render(self, mode='human'):
    """Render the environment."""
    if mode == 'rgb_array':
      # Return RGB array for video recording
      width, height, rgb_array, _, _ = self._pybullet_client.getCameraImage(
          width=640, height=480,
          viewMatrix=self._pybullet_client.computeViewMatrixFromYawPitchRoll(
              cameraTargetPosition=[0, 0, 0],
              distance=2.0,
              yaw=0,
              pitch=-20,
              roll=0),
          projectionMatrix=self._pybullet_client.computeProjectionMatrixFOV(
              fov=60,
              aspect=640/480,
              nearVal=0.1,
              farVal=100.0))
      return rgb_array
    return None

  def close(self):
    """Close the environment."""
    if self._record_video:
      self._pybullet_client.stopStateLogging(self._video_log_id)
    self._pybullet_client.disconnect()

  # Helper methods for CPG control
  def get_joint_angles(self):
    """Get current joint angles."""
    try:
      joint_states = self._pybullet_client.getJointStates(self.robot, self._joint_ids)
      return np.array([state[0] for state in joint_states])
    except:
      return np.zeros(len(self._joint_ids))

  def get_joint_velocities(self):
    """Get current joint velocities."""
    try:
      joint_states = self._pybullet_client.getJointStates(self.robot, self._joint_ids)
      return np.array([state[1] for state in joint_states])
    except:
      return np.zeros(len(self._joint_ids))

  def get_base_position(self):
    """Get robot base position."""
    try:
      base_pos, _ = self._pybullet_client.getBasePositionAndOrientation(self.robot)
      return np.array(base_pos)
    except:
      return np.array([0.0, 0.0, 0.0])

  def get_base_orientation(self):
    """Get robot base orientation."""
    try:
      _, base_orn = self._pybullet_client.getBasePositionAndOrientation(self.robot)
      return np.array(base_orn)
    except:
      return np.array([0.0, 0.0, 0.0, 1.0])

  def get_base_velocity(self):
    """Get robot base velocity."""
    try:
      base_vel, _ = self._pybullet_client.getBaseVelocity(self.robot)
      return np.array(base_vel)
    except:
      return np.array([0.0, 0.0, 0.0])

  def get_base_angular_velocity(self):
    """Get robot base angular velocity."""
    try:
      _, base_ang_vel = self._pybullet_client.getBaseVelocity(self.robot)
      return np.array(base_ang_vel)
    except:
      return np.array([0.0, 0.0, 0.0])

  def _apply_buoyancy(self):
    """Apply buoyancy force to keep robot floating."""
    # Get robot position
    base_pos, _ = self._pybullet_client.getBasePositionAndOrientation(self.robot)
    
    # Apply upward force if robot is below water surface
    if base_pos[2] < 0.1:  # If below water surface
      buoyancy_force = [0, 0, 5.0]  # Upward force
      self._pybullet_client.applyExternalForce(
          self.robot, -1, buoyancy_force, base_pos, 
          self._pybullet_client.WORLD_FRAME)
