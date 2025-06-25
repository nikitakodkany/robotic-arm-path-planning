from stable_baselines3 import PPO
import sys
import os

# Add the src directory to the Python path
sys.path.append(os.path.join(os.path.dirname(__file__), '..'))

from environments.robot_arm_env import RobotArmEnv

env = RobotArmEnv()
model = PPO("MlpPolicy", env, verbose=1)
model.learn(total_timesteps=100000)
model.save("../../models/robot_arm_ppo")
