from stable_baselines3 import PPO
import pybullet as p
import time
import numpy as np
import sys
import os

# Add the src directory to the Python path
sys.path.append(os.path.join(os.path.dirname(__file__), '..'))

from environments.robot_arm_env import RobotArmEnv

def evaluate_model(model_path, num_episodes=5, render=True):
    # Create environment with GUI
    env = RobotArmEnv()
    env.physicsClient = p.connect(p.GUI)  # Use GUI mode for visualization
    p.configureDebugVisualizer(p.COV_ENABLE_GUI, 0)
    p.resetDebugVisualizerCamera(cameraDistance=1.0, cameraYaw=0, cameraPitch=-30, cameraTargetPosition=[0,0,0])
    
    # Load the trained model
    model = PPO.load(model_path)
    
    # Run episodes
    for episode in range(num_episodes):
        obs = env.reset()
        total_reward = 0
        steps = 0
        done = False
        
        while not done and steps < 1000:
            action, _ = model.predict(obs, deterministic=True)
            obs, reward, done, info = env.step(action)
            total_reward += reward
            steps += 1
            time.sleep(0.01)  # Slow down visualization
            
        print(f"Episode {episode + 1}:")
        print(f"  Total Reward: {total_reward:.2f}")
        print(f"  Steps: {steps}")
        print(f"  Final Distance: {info['distance']:.3f}")
        print(f"  End Effector Position: {info['end_effector_pos']}")
        print(f"  Joint Positions: {info['joint_positions']}")
        print()
    
    p.disconnect()

if __name__ == "__main__":
    # Path to the trained model
    model_path = "../../models/robot_arm_ppo"
    
    # Evaluate the model
    evaluate_model(model_path, num_episodes=5, render=True) 