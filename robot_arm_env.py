import gym
import pybullet as p
import numpy as np
import time

class RobotArmEnv(gym.Env):
    def __init__(self):
        super(RobotArmEnv, self).__init__()
        self.physicsClient = p.connect(p.DIRECT)
        self.robot_id = p.loadURDF("robot_arm.urdf")
        self.target_pos = np.array([0.5, 0.0, 0.2])
        self.action_space = gym.spaces.Box(low=-1, high=1, shape=(6,), dtype=np.float32)
        self.observation_space = gym.spaces.Box(low=-np.inf, high=np.inf, shape=(6,), dtype=np.float32)
        
    def reset(self):
        p.resetSimulation()
        self.robot_id = p.loadURDF("robot_arm.urdf")
        self.current_pos = np.zeros(6)
        return self.current_pos
        
    def step(self, action):
        for i in range(len(action)):
            p.setJointMotorControl2(self.robot_id, i, p.POSITION_CONTROL, action[i])
        p.stepSimulation()
        joint_state = p.getJointStates(self.robot_id, range(6))
        joint_positions = np.array([state[0] for state in joint_state])
        distance = np.linalg.norm(joint_positions[:3] - self.target_pos)
        reward = -distance
        done = distance < 0.05
        return joint_positions, reward, done, {}

    def render(self, mode="human"):
        pass
