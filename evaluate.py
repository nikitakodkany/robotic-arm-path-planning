#!/usr/bin/env python3
"""
Main evaluation script for the robotic arm path planning project.
This script can be run from the project root directory.
"""

import sys
import os

# Add the src directory to the Python path
sys.path.append(os.path.join(os.path.dirname(__file__), 'src'))

from evaluation.evaluate_model import evaluate_model

if __name__ == "__main__":
    print("Starting robotic arm evaluation...")
    print("Loading model from models/robot_arm_ppo")
    print("=" * 50)
    
    # Path to the trained model
    model_path = "models/robot_arm_ppo"
    
    # Evaluate the model
    evaluate_model(model_path, num_episodes=5, render=True)
    
    print("Evaluation completed!") 