# icir_tutorial_pinocchio
CNU icir lab tutorial package for pinocchio library and mujoco simulation

All source codes are opimized for ROS-Noetic.

## 1. Prerequisites
```bash
git clone https://github.com/jspark861102/icir_gen3_robot_description.git
git clone https://github.com/jspark861102/kimm_mujoco_ros.git -b melodic
git clone https://github.com/jspark861102/icir_tutorial_pinocchio.git
```

## 2. Run
```bash
roslaunch icir_tutorial_pinocchio icir_tutorial_pinocchio_simulation.launch 

from terminal, press 'h' to move the robot to the home pose
from terminal, press 'a' to move the robot to another pose
```