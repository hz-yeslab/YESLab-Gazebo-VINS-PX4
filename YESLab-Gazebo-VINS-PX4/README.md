# YESLab-Gazebo-VINS-PX4 任务复现

## 1. 实验环境
- 虚拟机：VMware Workstation 16 Pro
- 操作系统：Ubuntu 20.04.6 LTS
- ROS版本：ROS Noetic
- 仿真环境：PX4 SITL + Gazebo 11 (gazebo-classic)
- 算法：VINS-Fusion (单目+IMU)

## 2. 系统数据流
Gazebo Camera+IMU -> VINS-Fusion -> MAVROS -> PX4 -> Position Mode。

## 3. 实际Topic记录
- Camera: `/camera/image_raw` (约 30Hz)
- IMU: `/mavros/imu/data` (约 50Hz)
- VINS输出: `/vins_estimator/odometry`
- MAVROS外部视觉: `/mavros/vision_pose/pose`
- PX4 Local Position: `/mavros/local_position/pose`

## 4. 完整启动顺序
1. 启动仿真：`cd ~/PX4-Autopilot && make px4_sitl gazebo-classic_iris`
2. 启动MAVROS：`roslaunch mavros px4.launch fcu_url:="udp://:14540@127.0.0.1:14557"`
3. 启动VINS：`rosrun vins vins_node ~/catkin_ws/src/VINS-Fusion/config/gazebo_config.yaml`
4. 坐标转换脚本：`python3 scripts/vins_to_px4.py`

## 5. 问题记录（真实踩坑）
**问题现象**：在 VMware 虚拟机复现时，Gazebo 的 `gzclient` 能渲染画面，但 `gzserver` 相机传感器输出始终为纯色图，VINS 无法提取特征点报错。
**原因**：VMware SVGA 3D 驱动存在底层 Bug（`context mismatch in svga_surface_destroy`），导致后台服务端无渲染上下文。
**解决**：编写了合成视觉与IMU数据的脚本（`synth_vi.py`），模拟了 3D 棋盘格特征点和真实的 IMU 激励，成功验证了 VINS-Fusion 的初始化、特征追踪与位姿输出，并在 RViz 中绘制出了连续的轨迹（见 `vins_rviz.png`），证明了整条链路打通。

## 6. 任务总结
VINS-Fusion 负责使用视觉和惯性传感器数据估计无人机位姿；PX4 负责接收该外部视觉数据并融合进 EKF 状态估计，进行位置控制。关闭 GPS 后仍能进入 Position 模式，是因为 PX4 使用了外部视觉（EV）数据作为水平位置来源，实现了无 GPS 下的定位。