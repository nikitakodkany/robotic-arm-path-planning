import gym
import pybullet as p
import numpy as np
import time

class RobotArmEnv(gym.Env):
    def __init__(self):
        super(RobotArmEnv, self).__init__()
        self.physicsClient = p.connect(p.DIRECT)
        p.setGravity(0, 0, -9.81)
        self.robot_id = p.loadURDF("robot_arm.urdf")
        
        # Get end effector link index
        self.end_effector_idx = -1
        for i in range(p.getNumJoints(self.robot_id)):
            if p.getJointInfo(self.robot_id, i)[12].decode('utf-8') == 'end_effector':
                self.end_effector_idx = i
                break
        
        # Fixed target position within reachable workspace
        self.target_pos = np.array([0.3, 0.0, 0.4])
        
        # 2 joints, each with position control
        self.action_space = gym.spaces.Box(low=-1, high=1, shape=(2,), dtype=np.float32)
        
        # Observation: 2 joint positions + 2 joint velocities + 3 target position + 3 end effector position
        self.observation_space = gym.spaces.Box(low=-np.inf, high=np.inf, shape=(10,), dtype=np.float32)
        
        # Previous distance for reward shaping
        self.prev_distance = None
        
    def reset(self):
        p.resetSimulation()
        p.setGravity(0, 0, -9.81)
        self.robot_id = p.loadURDF("robot_arm.urdf")
        
        # Get end effector link index again after reset
        self.end_effector_idx = -1
        for i in range(p.getNumJoints(self.robot_id)):
            if p.getJointInfo(self.robot_id, i)[12].decode('utf-8') == 'end_effector':
                self.end_effector_idx = i
                break
        
        # Reset joint positions to a reasonable starting configuration
        init_joint1 = np.random.uniform(-np.pi/4, np.pi/4)  # Smaller range for initial position
        init_joint2 = np.random.uniform(0, np.pi/2)  # Start with arm slightly raised
        p.resetJointState(self.robot_id, 0, init_joint1)
        p.resetJointState(self.robot_id, 1, init_joint2)
        
        # Get current end effector position
        end_effector_state = p.getLinkState(self.robot_id, self.end_effector_idx)
        end_effector_pos = np.array(end_effector_state[0])
        
        # Get joint states
        joint_states = [p.getJointState(self.robot_id, i) for i in range(2)]
        joint_positions = np.array([state[0] for state in joint_states])
        joint_velocities = np.array([state[1] for state in joint_states])
        
        # Initialize previous distance
        self.prev_distance = np.linalg.norm(end_effector_pos - self.target_pos)
        
        # Combine observations
        observation = np.concatenate([
            joint_positions,
            joint_velocities,
            self.target_pos,
            end_effector_pos
        ])
        
        return observation
        
    def step(self, action):
        # Scale actions from [-1, 1] to joint limits
        scaled_actions = np.clip(action * np.pi/4, -np.pi/2, np.pi/2)  # Reduced joint limits
        
        # Apply actions to both joints
        for i in range(2):
            p.setJointMotorControl2(
                self.robot_id,
                i,
                p.POSITION_CONTROL,
                targetPosition=scaled_actions[i],
                force=50.0,  # Reduced force
                maxVelocity=1.0  # Reduced velocity
            )
        
        # Simulate a few steps for stability
        for _ in range(5):
            p.stepSimulation()
        
        # Get end effector position
        end_effector_state = p.getLinkState(self.robot_id, self.end_effector_idx)
        end_effector_pos = np.array(end_effector_state[0])
        
        # Calculate distance to target
        distance = np.linalg.norm(end_effector_pos - self.target_pos)
        
        # Get joint states for observation
        joint_states = [p.getJointState(self.robot_id, i) for i in range(2)]
        joint_positions = np.array([state[0] for state in joint_states])
        joint_velocities = np.array([state[1] for state in joint_states])
        
        # Combine observations
        observation = np.concatenate([
            joint_positions,
            joint_velocities,
            self.target_pos,
            end_effector_pos
        ])
        
        # Calculate reward
        reward = 0
        
        # Distance reward (negative)
        reward -= distance * 0.1  # Scaled down the penalty
        
        # Progress reward (for getting closer to target)
        if self.prev_distance is not None:
            progress_reward = self.prev_distance - distance
            reward += progress_reward * 10.0  # Encourage moving towards target
        
        # Success reward
        if distance < 0.05:
            reward += 100.0  # Large bonus for reaching target
            done = True
        else:
            done = False
        
        # Penalty for joint limits
        joint_limit_penalty = 0
        for pos in joint_positions:
            if abs(pos) > np.pi/2:  # Reduced joint limit threshold
                joint_limit_penalty += (abs(pos) - np.pi/2) * 0.1
        reward -= joint_limit_penalty
        
        # Update previous distance
        self.prev_distance = distance
        
        # Early termination for instability
        if not self._is_state_valid():
            reward -= 50.0  # Penalty for invalid state
            done = True
        
        # Add info dict for debugging
        info = {
            'distance': distance,
            'end_effector_pos': end_effector_pos,
            'joint_positions': joint_positions
        }
        
        return observation, reward, done, info

    def _is_state_valid(self):
        # Check if the robot is in a valid state
        for i in range(2):
            state = p.getJointState(self.robot_id, i)
            if abs(state[0]) > np.pi:  # Joint limits
                return False
            if abs(state[1]) > 2.0:  # Velocity limits
                return False
        return True

    def render(self, mode="human"):
        pass
