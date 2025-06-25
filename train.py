#!/usr/bin/env python3
"""
Main training script for the robotic arm path planning project.
This script can be run from the project root directory.
"""

import sys
import os

# Add the src directory to the Python path
sys.path.append(os.path.join(os.path.dirname(__file__), 'src'))

from training.train_robot_arm import *

if __name__ == "__main__":
    print("Starting robotic arm training...")
    print("Training will save the model to models/robot_arm_ppo")
    print("=" * 50)
    
    # The training will be executed by the imported code
    print("Training completed!") 