import gymnasium as gym
import numpy as np
import pybullet as p
import pybullet_data
import math
from gymnasium import spaces

class RoboticArmEnv(gym.Env):
    def __init__(self):
        super(RoboticArmEnv, self).__init__()
        
        # Connect to PyBullet
        self.client = p.connect(p.GUI)
        p.setAdditionalSearchPath(pybullet_data.getDataPath())
        
        # Set up the environment
        p.setGravity(0, 0, -9.81)
        p.setTimeStep(1/240)
        
        # Load the plane and robot
        self.plane = p.loadURDF("plane.urdf")
        self.robot = p.loadURDF("urdf/robot.urdf", [0, 0, 0.1])
        
        # Get robot info
        self.num_joints = p.getNumJoints(self.robot)
        self.joint_indices = [i for i in range(self.num_joints) if p.getJointInfo(self.robot, i)[2] == p.JOINT_REVOLUTE]
        
        # Define action and observation spaces
        self.action_space = spaces.Box(
            low=-1.0,
            high=1.0,
            shape=(len(self.joint_indices),),
            dtype=np.float32
        )
        
        self.observation_space = spaces.Box(
            low=-np.inf,
            high=np.inf,
            shape=(len(self.joint_indices) * 2 + 3,),  # joint angles + joint velocities + target position
            dtype=np.float32
        )
        
        # Target position
        self.target_pos = np.array([0.5, 0.5, 0.5])
        self.target_visual = p.createVisualShape(
            p.GEOM_SPHERE,
            radius=0.05,
            rgbaColor=[1, 0, 0, 1]
        )
        self.target_body = p.createMultiBody(
            baseMass=0,
            baseVisualShapeIndex=self.target_visual,
            basePosition=self.target_pos
        )
        
    def get_observation(self):
        # Get joint states
        joint_states = p.getJointStates(self.robot, self.joint_indices)
        joint_angles = [state[0] for state in joint_states]
        joint_velocities = [state[1] for state in joint_states]
        
        # Get end effector position
        end_effector_state = p.getLinkState(self.robot, self.joint_indices[-1])
        end_effector_pos = end_effector_state[0]
        
        # Calculate distance to target
        distance_to_target = np.linalg.norm(np.array(end_effector_pos) - self.target_pos)
        
        # Combine observations
        obs = np.concatenate([
            joint_angles,
            joint_velocities,
            self.target_pos
        ])
        
        return obs, distance_to_target
    
    def reset(self, seed=None):
        super().reset(seed=seed)
        
        # Reset robot to initial position
        for i in self.joint_indices:
            p.resetJointState(self.robot, i, 0)
        
        # Generate new target position
        self.target_pos = np.random.uniform(-0.5, 0.5, size=3)
        self.target_pos[2] = np.random.uniform(0.1, 0.5)  # Keep z above ground
        p.resetBasePositionAndOrientation(self.target_body, self.target_pos, [0, 0, 0, 1])
        
        obs, _ = self.get_observation()
        return obs, {}
    
    def step(self, action):
        # Apply action to robot joints
        for i, joint_idx in enumerate(self.joint_indices):
            p.setJointMotorControl2(
                self.robot,
                joint_idx,
                p.POSITION_CONTROL,
                targetPosition=action[i] * math.pi,
                force=100
            )
        
        # Step simulation
        p.stepSimulation()
        
        # Get new observation
        obs, distance_to_target = self.get_observation()
        
        # Calculate reward
        reward = -distance_to_target  # Negative distance as reward
        
        # Check if episode is done
        done = distance_to_target < 0.05  # Success threshold
        
        return obs, reward, done, False, {}
    
    def render(self, mode='human'):
        pass  # PyBullet handles rendering
    
    def close(self):
        p.disconnect(self.client) 