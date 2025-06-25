from stable_baselines3 import PPO
import pybullet as p
import time
import numpy as np
import sys
import os

# Add the src directory to the Python path
sys.path.append(os.path.join(os.path.dirname(__file__), '..'))

from environments.robot_arm_env import RobotArmEnv

def main():
    # Create environment with GUI
    env = RobotArmEnv()
    env.physicsClient = p.connect(p.GUI)  # Use GUI mode for visualization
    p.configureDebugVisualizer(p.COV_ENABLE_GUI, 0)
    p.resetDebugVisualizerCamera(cameraDistance=1.0, cameraYaw=0, cameraPitch=-30, cameraTargetPosition=[0,0,0])
    
    # Load the trained model
    model = PPO.load("../../models/robot_arm_ppo")
    
    # Run a few episodes
    for episode in range(5):
        obs = env.reset()
        total_reward = 0
        steps = 0
        done = False
        
        while not done and steps < 1000:
            action, _ = model.predict(obs, deterministic=True)
            obs, reward, done, _ = env.step(action)
            total_reward += reward
            steps += 1
            time.sleep(0.01)  # Slow down visualization
            
        print(f"Episode {episode + 1}: Total Reward = {total_reward:.2f}, Steps = {steps}")
    
    p.disconnect()

if __name__ == "__main__":
    main()
