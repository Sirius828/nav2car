# RB21 SLAM配置说明

本配置实现了深度相机和激光雷达数据融合的SLAM系统。

## 系统架构

```
深度相机 (/camera/depth/image_raw) 
    ↓
depthimage_to_laserscan
    ↓
激光扫描 (/scan_depth)
    ↓
ros2_laser_scan_merger ← 激光雷达 (/scan)
    ↓
融合激光扫描 (/scan_merge)
    ↓
slam_toolbox
    ↓
地图 (/map)
```

## 配置文件

- `config/slam_config.yaml`: 主要SLAM配置文件
- `launch/test_data_fusion.launch.py`: 数据融合测试启动文件
- `launch/slam_with_depth_fusion.launch.py`: 完整SLAM启动文件

## 使用步骤

### 1. 测试数据融合（推荐先执行）

```bash
# 构建工作空间
cd /home/sirius/ssd/ros2workspace
colcon build --packages-select rb21_localization

# 源环境
source install/setup.bash

# 启动数据融合测试
ros2 launch rb21_localization test_data_fusion.launch.py
```

在另一个终端中检查话题：
```bash
# 检查是否正常发布融合后的激光扫描
ros2 topic list | grep scan
ros2 topic echo /scan_merge --once

# 在rviz中可视化
rviz2
```

### 2. 完整SLAM系统

确认数据融合正常后，启动完整SLAM：
```bash
ros2 launch rb21_localization slam_with_depth_fusion.launch.py
```

## 重要参数配置

### 坐标变换
需要根据实际机器人配置调整相机到base_link的变换：
- x: 0.1 (相机在base_link前方10cm)
- y: 0.0 (相机在base_link中心线上)
- z: 0.3 (相机在base_link上方30cm)

### 激光扫描范围
- 激光雷达: -4° to 94°
- 深度相机: -60° to 60°
- 最大扫描距离: 8.0m

### 话题映射
- 激光雷达输入: `/scan`
- 深度图像输入: `/camera/depth/image_raw`
- 相机信息输入: `/camera/depth/camera_info`
- 深度扫描输出: `/scan_depth`
- 融合扫描输出: `/scan_merge`

## 故障排除

### 1. 检查话题是否存在
```bash
ros2 topic list
ros2 topic info /camera/depth/image_raw
ros2 topic info /scan
```

### 2. 检查坐标变换
```bash
ros2 run tf2_tools view_frames
ros2 topic echo /tf_static
```

### 3. 调试激光扫描融合
```bash
# 查看原始激光扫描
ros2 topic echo /scan --once
# 查看深度转换的激光扫描
ros2 topic echo /scan_depth --once
# 查看融合后的激光扫描
ros2 topic echo /scan_merge --once
```

### 4. 检查依赖包
确保安装了以下ROS2包：
- `depthimage_to_laserscan`
- `ros2_laser_scan_merger`
- `slam_toolbox`
- `robot_localization`

安装缺失包：
```bash
sudo apt install ros-humble-depthimage-to-laserscan
sudo apt install ros-humble-slam-toolbox
sudo apt install ros-humble-robot-localization
```

## 参数调整建议

### 如果深度扫描质量不好：
- 调整 `range_min` 和 `range_max`
- 修改 `scan_height` 参数

### 如果融合效果不理想：
- 调整激光雷达的角度范围 (`laser1AngleMin/Max`, `laser2AngleMin/Max`)
- 修改传感器位置偏移 (`laser1XOff`, `laser1YOff`, `laser2XOff`, `laser2YOff`)

### 如果SLAM性能不佳：
- 调整 `correlation_search_space_dimension`
- 修改 `minimum_travel_distance` 和 `minimum_travel_heading`
- 调整回环检测参数
