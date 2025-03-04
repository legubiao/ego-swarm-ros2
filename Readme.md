# Ego Planner Swarm ROS2

This repository was forked from the ros2_version brach
of [Ego Planner Swarm](https://github.com/ZJU-FAST-Lab/ego-planner-swarm). I refactored the code to make it follow
modern cmake.

Todo List:

- [ ] Refactor all the code to make it follow modern cmake
- [ ] Add Gazebo Harmonic PX4 simulation

## 1. Quick Start

* clone the repository
    ```bash
    mkdir -p ~/ego_ws/src
    cd ~/ego_ws/src
    git clone https://github.com/legubiao/ego-swarm-ros2
  ```
* rosdep
    ```bash
    cd ~/ego_ws
    rosdep install --from-paths src --ignore-src -r -y
    ```
* Compile the package
    ```bash
    colcon build
    ```

## 2. Run in the uav simulation

### 2.1 Launch Rviz

```bash
source ~/ego_ws/install/setup.bash
ros2 launch ego_planner rviz.launch.py 
```

### 3.2 Launch Ego Swarm

Choose from one of the followings:

* Single UAV
  ```bash
  source ~/ego_ws/install/setup.bash
  ros2 launch ego_planner single_run_in_sim.launch.py 
  ```
* swarm
  ```bash
  source ~/ego_ws/install/setup.bash
  ros2 launch ego_planner swarm.launch.py 
  ```
* large swarm
  ```bash
  source ~/ego_ws/install/setup.bash
  ros2 launch ego_planner swarm_large.launch.py  
  ```

* 附加参数，可以选择地图生成模式以及是否考虑动力学
    * use_mockamap:地图生成方式，默认为False，False时使用Random Forest, True时使用mockamap
    * use_dynamic:是否考虑动力学，默认为False, False时不考虑, True时考虑

```
ros2 launch ego_planner single_run_in_sim.launch.py use_mockamap:=True use_dynamic:=False
```