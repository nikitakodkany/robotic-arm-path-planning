#include <pybullet/pybullet.h>
#include <iostream>
#include <vector>
#include <cmath>

int main() {
    pybullet::connect(pybullet::SHARED_MEMORY);
    int robot_id = pybullet::loadURDF("robot_arm.urdf");
    std::vector<float> target_pos = {0.5, 0.2, 0.3};

    for (int i = 0; i < 1000; i++) {
        std::vector<float> joint_angles = {0.0, 0.5, -0.5, 0.2, 0.1, -0.2};
        for (int j = 0; j < joint_angles.size(); j++) {
            pybullet::setJointMotorControl2(robot_id, j, pybullet::POSITION_CONTROL, joint_angles[j]);
        }
        pybullet::stepSimulation();
        std::vector<float> end_effector_pos = pybullet::getLinkState(robot_id, 6).worldPosition;
        float distance = std::sqrt(std::pow(end_effector_pos[0] - target_pos[0], 2) +
                                   std::pow(end_effector_pos[1] - target_pos[1], 2) +
                                   std::pow(end_effector_pos[2] - target_pos[2], 2));
        std::cout << "Step: " << i << ", Distance to target: " << distance << std::endl;
        if (distance < 0.05) {
            std::cout << "Target reached!" << std::endl;
            break;
        }
    }

    return 0;
}
