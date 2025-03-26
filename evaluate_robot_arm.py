from stable_baselines3 import PPO
from robot_arm_env import RobotArmEnv

model = PPO.load("robot_arm_ppo")
env = RobotArmEnv()
obs = env.reset()

for _ in range(1000):
    action, _states = model.predict(obs)
    obs, rewards, done, info = env.step(action)
    print(f"Step: {_}, Reward: {rewards}, Done: {done}")
    if done:
        print("Target reached!")
        break
