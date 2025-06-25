# Project Structure

This document describes the organized directory structure of the Robotic Arm Path Planning project.

## 📁 Directory Layout

```
robotic-arm-path-planning/
├── src/                          # Source code directory
│   ├── environments/             # Robot environment implementations
│   │   ├── __init__.py
│   │   ├── robot_arm_env.py      # Main robot arm environment
│   │   └── robotic_arm_env.py    # Alternative environment
│   ├── training/                 # Training scripts and utilities
│   │   ├── __init__.py
│   │   └── train_robot_arm.py    # PPO training script
│   ├── evaluation/               # Evaluation and testing scripts
│   │   ├── __init__.py
│   │   ├── evaluate_model.py     # Model evaluation with GUI
│   │   └── evaluate_robot_arm.py # Alternative evaluation
│   ├── utils/                    # Utility functions and tools
│   │   ├── __init__.py
│   │   ├── call_cpp_from_python.py # C++ integration bridge
│   │   └── robot_arm_ik.cpp      # C++ inverse kinematics
│   └── __init__.py
├── data/                         # Data files and resources
│   └── urdf/                     # Robot model definitions
│       ├── robot_arm.urdf        # Main robot arm model
│       └── robot_alt.urdf        # Alternative robot model
├── models/                       # Trained models and weights
│   └── robot_arm_ppo.zip         # Trained PPO model
├── docs/                         # Documentation
│   ├── features.md               # Project features and capabilities
│   └── project_structure.md      # This file
├── train.py                      # Main training script (root level)
├── evaluate.py                   # Main evaluation script (root level)
├── requirements.txt              # Python dependencies
└── README.md                     # Project overview and setup
```

## 🎯 Purpose of Each Directory

### **src/** - Source Code
- **environments/**: Contains all robot environment implementations
  - `robot_arm_env.py`: Main 2-DOF robotic arm environment
  - `robotic_arm_env.py`: Alternative environment implementation
- **training/**: Training-related scripts and utilities
  - `train_robot_arm.py`: PPO training implementation
- **evaluation/**: Model evaluation and testing scripts
  - `evaluate_model.py`: GUI-based model evaluation
  - `evaluate_robot_arm.py`: Alternative evaluation approach
- **utils/**: Utility functions and cross-language integration
  - `call_cpp_from_python.py`: Python-C++ bridge
  - `robot_arm_ik.cpp`: High-performance inverse kinematics

### **data/** - Data and Resources
- **urdf/**: Robot model definitions in URDF format
  - `robot_arm.urdf`: Main 2-DOF robotic arm model
  - `robot_alt.urdf`: Alternative robot configuration

### **models/** - Trained Models
- Contains saved model weights and checkpoints
- `robot_arm_ppo.zip`: Trained PPO model for the robotic arm

### **docs/** - Documentation
- `features.md`: Comprehensive project features and capabilities
- `project_structure.md`: This directory structure guide

## 🚀 Usage

### **Training**
```bash
# From project root
python train.py

# Or from src directory
python src/training/train_robot_arm.py
```

### **Evaluation**
```bash
# From project root
python evaluate.py

# Or from src directory
python src/evaluation/evaluate_model.py
```

### **Development**
```bash
# Add src to Python path for development
export PYTHONPATH="${PYTHONPATH}:$(pwd)/src"

# Or use the provided scripts that handle path management
```

## 🔧 Import Structure

The project uses relative imports within the `src/` directory:

```python
# Example: importing from training script
import sys
import os
sys.path.append(os.path.join(os.path.dirname(__file__), '..'))
from environments.robot_arm_env import RobotArmEnv
```

## 📝 Benefits of This Structure

1. **Modularity**: Clear separation of concerns
2. **Scalability**: Easy to add new environments, training methods, or utilities
3. **Maintainability**: Organized code structure
4. **Reusability**: Components can be easily imported and reused
5. **Documentation**: Clear documentation structure
6. **Data Management**: Separated data, models, and source code

## 🔄 Migration Notes

- All import paths have been updated to reflect the new structure
- URDF file paths are now relative to the data directory
- Model save/load paths have been updated
- Root-level scripts (`train.py`, `evaluate.py`) provide easy access 