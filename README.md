# Robotic Arm Path Planning with Reinforcement Learning

A simulation-based project for robotic arm path planning and object manipulation using Reinforcement Learning (PPO) and inverse kinematics in C++. Simulation is built in PyBullet and the RL environment is OpenAI Gym-compatible.

## Structure

- `robot_arm_env.py`: Gym environment for the robot arm
- `train_robot_arm.py`: PPO model training
- `evaluate_robot_arm.py`: Model evaluation
- `robot_arm_ik.cpp`: Inverse Kinematics in C++
- `call_cpp_from_python.py`: Calls the C++ path planner from Python
- `robot_arm.urdf`: Robot description (add or modify URDF)