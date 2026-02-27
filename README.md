<p align="center">
  <img src="assets/demo.gif" width="720"/>
</p>

<h1 align="center">Elastic-Tracker ROS 2</h1>

<p align="center">
  <b>基于 Elastic-Tracker 的无人机自主目标跟踪系统</b><br>
  ROS 2 Humble · PX4 · LiDAR SLAM · YOLO 视觉检测
</p>

<p align="center">
  <img src="https://img.shields.io/badge/ROS_2-Humble-blue?logo=ros" alt="ROS 2"/>
  <img src="https://img.shields.io/badge/PX4-Autopilot-orange?logo=drone" alt="PX4"/>
  <img src="https://img.shields.io/badge/C%2B%2B-17-brightgreen?logo=cplusplus" alt="C++17"/>
</p>

---

## Overview

本项目将 [Elastic-Tracker](https://github.com/ZJU-FAST-Lab/Elastic-Tracker)（ZJU FAST Lab, ICRA 2022）从 ROS 1 移植到 **ROS 2 Humble**，并集成 PX4 飞控、LiDAR SLAM 里程计与 YOLO 实时目标检测，实现无人机对地面行人的自主跟踪。

### 核心特性

- **弹性轨迹规划** — 安全飞行走廊 + 可见性约束，保证跟踪过程中目标始终可见
- **实时避障** — 基于深度相机 / LiDAR 构建局部栅格地图，A* + 轨迹优化联合规划
- **目标状态估计** — EKF 融合 YOLO 检测与里程计，预测目标运动
- **PX4 集成** — 通过 micro-XRCE-DDS 直接对接 PX4 飞控，几何姿态控制

---

## 系统架构

```
┌─────────────────┐     /Odometry (ENU)
│  small_point_lio │────────────────────────────────────┐
│  (LiDAR SLAM)   │                                    │
└─────────────────┘                                    ▼
                                              ┌──────────────┐
┌─────────────────┐    /yolo/detections       │              │
│   YOLO 检测     │─────────────────────────► │  target_ekf  │
└─────────────────┘                           │              │
                                              └──────┬───────┘
                                                     │ /target_ekf_odom
                                                     ▼
┌─────────────────┐   /gridmap_inflate    ┌──────────────────┐
│    mapping      │──────────────────────►│    planning      │
│  (局部建图)      │                       │  (轨迹规划)       │
└─────────────────┘                       └──────┬───────────┘
                                                  │ trajectory
                                                  ▼
                                          ┌──────────────┐
                                          │  traj_server  │
                                          └──────┬───────┘
                                                  │ position_cmd
                                                  ▼
                                          ┌──────────────┐      ┌─────────┐
                                          │   px4_ctrl    │─────►│   PX4   │
                                          │ (几何控制器)    │      │  飞控   │
                                          └──────────────┘      └─────────┘
```

---

## 项目结构

```
tracker/
├── assets/                     # 演示素材
├── src/
│   ├── planning/               # 轨迹规划（核心）
│   │   ├── planning/           #   规划节点 + 轨迹服务
│   │   ├── traj_opt/           #   轨迹优化求解器
│   │   └── decomp_ros_*/       #   凸分解可视化
│   ├── mapping/                # 局部栅格地图
│   ├── target_ekf/             # 目标 EKF 状态估计
│   ├── px4_ctrl/               # PX4 几何姿态控制器
│   ├── px4_msgs/               # PX4 消息定义
│   ├── quadrotor_msgs/         # 自定义消息
│   ├── small_point_lio/        # LiDAR-惯性里程计
│   └── livox_ros_driver2/      # Livox 雷达驱动
└── README.md
```

---

## 环境依赖

| 依赖 | 版本 |
|------|------|
| Ubuntu | 22.04 |
| ROS 2 | Humble |
| PX4-Autopilot | v1.15+ |
| Eigen | 3.4+ |
| PCL | 1.12+ |
| OpenCV | 4.x |

---

## 编译

```bash
# 1. 安装 ROS 2 Humble
source /opt/ros/humble/setup.bash

# 2. 克隆项目
git clone <repo_url> tracker
cd tracker

# 3. 编译
colcon build
source install/setup.bash
```

---

## 启动

```bash
# 启动跟踪系统（默认使用 nav_msgs 里程计）
ros2 launch planning px4_sim_tracking.launch.py

# 使用 PX4 原生里程计
ros2 launch planning px4_sim_tracking.launch.py odom_source:=px4_msgs
```

---

## 参考

- **原版论文**: [Elastic Tracker: A Spatio-temporal Trajectory Planner for Flexible Aerial Tracking](https://arxiv.org/abs/2109.07111), Jialin Ji, Neng Pan, Chao Xu, Fei Gao, ICRA 2022
- **原版仓库**: [ZJU-FAST-Lab/Elastic-Tracker](https://github.com/ZJU-FAST-Lab/Elastic-Tracker)
- **视频**: [YouTube](https://www.youtube.com/watch?v=G5taHOpAZj8) | [Bilibili](https://www.bilibili.com/video/BV1o44y1b7wC)

---

## License

本项目基于 Elastic-Tracker 开发，遵循其原始许可协议。
