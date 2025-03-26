from stable_baselines3 import PPO
from robot_arm_env import RobotArmEnv

env = RobotArmEnv()
model = PPO("MlpPolicy", env, verbose=1)
model.learn(total_timesteps=100000)
model.save("robot_arm_ppo")
