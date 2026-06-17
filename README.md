# 机器人导航工作空间 (robot_navigation_ws)

本仓库包含基于 ROS（Melodic/Noetic）的移动机器人导航功能包，集成了多种 SLAM 算法（Gmapping、Cartographer、Hector、Karto）和路径规划插件（DWA、TEB、Simple Local Planner），适用于室内移动机器人自主导航。

## 功能包列表
- `robot_description`：机器人 URDF/Xacro 模型描述
- `robot_navigation`：核心导航 launch 文件、参数配置、地图及脚本
- `robot_simulation`：Gazebo 仿真模型与场景
- `simple_local_planner`：自定义本地规划器插件
- `smooth_waypoint_planner`：平滑路径规划插件

## 环境依赖
- Ubuntu 18.04 / 20.04
- ROS Melodic / Noetic
- Gazebo（可选，用于仿真）
- 依赖包：`ros-<distro>-navigation`、`ros-<distro>-gmapping`、`ros-<distro>-cartographer` 等

## 编译与运行
```bash
# 1. 克隆仓库（如果你在本地已存在则跳过）
cd ~/workspace
git clone https://github.com/maker-lbw/ros_program.git rk_ws

# 2. 编译
cd ~/workspace/rk_ws
rm -rf build devel   # 首次编译可不执行
catkin_make

# 3. 设置环境变量
source devel/setup.bash

# 4. 启动导航（示例）
roslaunch robot_navigation rk_navigation.launch