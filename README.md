# Robotic Arm Path Planning

This project implements a reinforcement learning-based path planning system for a robotic arm using PyBullet and Stable-Baselines3. The goal is to train a robotic arm to reach target positions in 3D space.

## Project Structure

```
robotic-arm-path-planning/
├── robot_arm_env.py      # Gym environment for the robotic arm
├── train_robot_arm.py    # Training script using PPO
├── evaluate_model.py     # Script to evaluate trained models
├── robot_arm.urdf        # URDF file defining the robotic arm
├── requirements.txt      # Python dependencies
└── README.md            # Project documentation
```

## Requirements

- Python 3.10+
- PyBullet
- Stable-Baselines3
- PyTorch
- NumPy
- Matplotlib

## Installation

1. Clone the repository:
```bash
git clone https://github.com/yourusername/robotic-arm-path-planning.git
cd robotic-arm-path-planning
```

2. Install the required packages:
```bash
pip install -r requirements.txt
```

## Environment Description

The robotic arm environment consists of:
- A 2-DOF robotic arm with revolute joints
- A fixed target position in 3D space
- Continuous action space for joint control
- Rich observation space including:
  - Joint positions and velocities
  - Target position
  - End-effector position

### Action Space
- 2 continuous actions (one per joint)
- Actions are scaled to ±π/2 radians
- Position control with force and velocity limits

### Observation Space
- 10-dimensional vector containing:
  - 2 joint positions
  - 2 joint velocities
  - 3 target position coordinates
  - 3 end-effector position coordinates

### Reward Function
The reward function includes:
- Negative distance penalty
- Progress reward for moving closer to target
- Success reward for reaching target
- Joint limit penalties
- Early termination for invalid states

## Training

To train the model:
```bash
python train_robot_arm.py
```

The training script uses PPO with the following hyperparameters:
- Learning rate: 3e-4
- Number of steps per update: 1024
- Batch size: 64
- Number of epochs: 10
- Discount factor (gamma): 0.99
- GAE lambda: 0.95
- Clip range: 0.2
- Network architecture: [128, 128] for both policy and value networks

## Evaluation

To evaluate a trained model:
```bash
python evaluate_model.py
```

The evaluation script:
- Loads the trained model
- Runs 5 episodes with visualization
- Prints detailed information for each episode:
  - Total reward
  - Number of steps
  - Final distance to target
  - End-effector position
  - Joint positions

## Results

The model's performance is evaluated based on:
- Success rate in reaching the target
- Average distance to target
- Smoothness of motion
- Joint limit violations

## Future Improvements

1. Add more complex tasks:
   - Multiple target positions
   - Obstacle avoidance
   - Pick-and-place operations

2. Enhance the environment:
   - Add more DOF to the arm
   - Include dynamic obstacles
   - Add force/torque sensing

3. Improve the learning:
   - Implement curriculum learning
   - Add demonstration data
   - Try different RL algorithms

## License

This project is licensed under the MIT License - see the LICENSE file for details.