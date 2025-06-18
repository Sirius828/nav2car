# Gimbal Controller Package

云台控制器包，用于根据目标像素位置控制云台角度，实现目标跟踪功能。

## 系统架构

```
相机图像 → YOLO检测 → 云台控制器 → 底层通信 → STM32电机
    ↓           ↓          ↓          ↓        ↓
image_rect → target_pos → gimbal_cmd → serial → 云台转动
```

## 功能特性

- ✅ **像素到角度转换**：基于相机FOV自动计算
- ✅ **PID控制**：平滑精确的角度控制
- ✅ **死区处理**：避免小幅抖动
- ✅ **安全限位**：防止云台过度转动
- ✅ **自动FOV计算**：从camera_info自动获取视场角
- ✅ **参数可调**：所有控制参数可在launch文件中配置

## 话题接口

### 订阅话题
- `/target_position_pixel` (geometry_msgs/Point): YOLO检测的目标像素坐标
  - `x`: 目标中心x坐标
  - `y`: 目标中心y坐标  
  - `z`: 检测置信度
- `/camera/camera_info` (sensor_msgs/CameraInfo): 相机内参信息

### 发布话题
- `/gimbal/angle_cmd` (geometry_msgs/Vector3): 云台角度控制指令
  - `x`: yaw角度 (度)
  - `y`: pitch角度 (度)
  - `z`: 控制模式 (1.0=绝对角度)

## 参数说明

### 图像参数
- `image_width`: 图像宽度 (默认: 640)
- `image_height`: 图像高度 (默认: 480)
- `camera_horizontal_fov`: 水平视场角(度) (默认: 60.0)
- `camera_vertical_fov`: 垂直视场角(度) (默认: 45.0)
- `use_camera_info`: 是否自动从camera_info计算FOV (默认: True)

### PID控制参数
- `kp_yaw/kp_pitch`: 比例系数 (默认: 0.5)
- `ki_yaw/ki_pitch`: 积分系数 (默认: 0.0)
- `kd_yaw/kd_pitch`: 微分系数 (默认: 0.1)

### 安全参数
- `dead_zone_pixels`: 死区像素数 (默认: 20)
- `max_angle_step`: 最大单步角度(度) (默认: 3.0)
- `yaw_limit_min/max`: yaw轴角度限制 (默认: ±90°)
- `pitch_limit_min/max`: pitch轴角度限制 (默认: -30°~+30°)

## 使用方法

### 1. 编译包
```bash
cd /home/sirius/ssd/ros2workspace
colcon build --packages-select gimbal_controller
source install/setup.bash
```

### 2. 启动完整系统
```bash
# 方法1：分步启动
# 启动相机和云台控制器
ros2 launch gimbal_controller tracking_system.launch.py

# 启动YOLO检测器（需要虚拟环境）
/home/sirius/ssd/ros2workspace/src/yolo_detector/scripts/start_yolo_detector.sh
```

```bash
# 方法2：单独启动各组件
# 启动相机系统
ros2 launch camera_driver camera.launch.py

# 启动云台控制器
ros2 launch gimbal_controller gimbal_controller.launch.py

# 启动YOLO检测器
/home/sirius/ssd/ros2workspace/src/yolo_detector/scripts/start_yolo_detector.sh
```

### 3. 监控系统状态
```bash
# 查看目标检测结果
ros2 topic echo /target_position_pixel

# 查看云台控制指令
ros2 topic echo /gimbal/angle_cmd

# 查看系统话题
ros2 topic list
```

## 调试和参数调优

### 死区调整
如果云台抖动，增大 `dead_zone_pixels`：
```bash
ros2 param set /gimbal_controller dead_zone_pixels 30
```

### PID调优
- **响应过慢**：增大kp值
- **有震荡**：减小kp值，增大kd值
- **有稳态误差**：增大ki值

### 角度限制
根据云台硬件规格调整角度限制：
```yaml
'yaw_limit_min': -180.0,
'yaw_limit_max': 180.0,
'pitch_limit_min': -45.0,
'pitch_limit_max': 45.0,
```

## 下一步扩展

### 底层通信包 (未来)
创建 `robot_base_driver` 包来统一管理：
- 云台角度指令
- 导航速度指令
- 其他传感器数据
- STM32串口通信

### 数据格式示例
```yaml
# /gimbal/angle_cmd 输出示例
x: 15.5    # yaw角度：向右15.5度
y: -5.2    # pitch角度：向下5.2度  
z: 1.0     # 绝对角度模式
```

## 故障排除

1. **没有角度输出**：检查是否收到target_position_pixel话题
2. **角度计算错误**：确认camera_info话题正常，FOV参数正确
3. **控制不稳定**：调整PID参数和死区设置
4. **角度超限**：检查limit参数设置

## 系统架构图

```
┌─────────────┐    ┌──────────────┐    ┌─────────────────┐
│   Camera    │───▶│ YOLO Detector│───▶│ Gimbal Controller│
│   Driver    │    │              │    │                 │
└─────────────┘    └──────────────┘    └─────────────────┘
                                               │
                                               ▼
                   ┌─────────────────┐    ┌─────────────────┐
                   │ Robot Base      │◀───│ /gimbal/angle_cmd│
                   │ Driver (未来)   │    │     话题        │
                   └─────────────────┘    └─────────────────┘
                          │
                          ▼
                   ┌─────────────────┐
                   │     STM32       │
                   │   电机控制      │
                   └─────────────────┘
```
