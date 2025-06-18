"""
Ferrari跟踪系统集成Launch文件
==========================

此文件整合了camera_driver、yolo_detector和gimbal_controller三个包，
实现完整的Ferrari目标跟踪系统。



参数说明:
--------
- yolo_model: YOLO模型文件 (ferrari.engine, yolo11n.pt等)
- yolo_confidence: 检测置信度 (0.0-1.0, 推荐0.7-0.9)
- yolo_target_class: 目标类别 (ferrrari, person, car等)
- camera_device: 相机设备 (/dev/video0, /dev/video1等)
- camera_width/height: 相机分辨率 (640x480, 1280x720等)
- gimbal_kp: 云台响应灵敏度 (0.1-1.0, 越大越敏感)
- gimbal_dead_zone: 云台死区 (10-50像素, 越小越敏感)
"""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess, LogInfo, DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
import os

def generate_launch_description():
    
    # ========== 可配置参数声明 ==========
    
    # === YOLO检测参数 ===
    yolo_model_arg = DeclareLaunchArgument(
        'yolo_model',
        default_value='ferrari.engine',
        description='YOLO模型文件名'
    )
    
    yolo_confidence_arg = DeclareLaunchArgument(
        'yolo_confidence',
        default_value='0.85',
        description='检测置信度阈值 (0.7=宽松, 0.85=平衡, 0.9=严格)'
    )
    
    yolo_target_class_arg = DeclareLaunchArgument(
        'yolo_target_class',
        default_value='ferrrari',
        description='目标类别名称 (注意: 模型中实际为"ferrrari")'
    )
    
    yolo_device_arg = DeclareLaunchArgument(
        'yolo_device',
        default_value='0',
        description='推理设备 (0=第一块GPU, 1=第二块GPU, cpu=CPU推理)'
    )
    
    # === 相机参数 ===
    camera_device_arg = DeclareLaunchArgument(
        'camera_device',
        default_value='/dev/video0',
        description='相机设备路径 (/dev/video0, /dev/video1等)'
    )
    
    camera_width_arg = DeclareLaunchArgument(
        'camera_width',
        default_value='640',
        description='图像宽度 (640=标清, 1280=高清, 1920=全高清)'
    )
    
    camera_height_arg = DeclareLaunchArgument(
        'camera_height',
        default_value='480',
        description='图像高度 (480=标清, 720=高清, 1080=全高清)'
    )
    
    camera_fps_arg = DeclareLaunchArgument(
        'camera_fps',
        default_value='0',
        description='相机帧率 (0=自动, 15/30/60等)'
    )
    
    # === 相机图像质量参数 ===
    camera_brightness_arg = DeclareLaunchArgument(
        'camera_brightness',
        default_value='0',
        description='亮度调节 (-64到64, 0=默认)'
    )
    
    camera_contrast_arg = DeclareLaunchArgument(
        'camera_contrast',
        default_value='32',
        description='对比度 (0到64, 32=默认)'
    )
    
    camera_exposure_arg = DeclareLaunchArgument(
        'camera_exposure',
        default_value='25',
        description='曝光时间 (毫秒, 10=明亮环境, 30=一般环境, 50=暗环境)'
    )
    
    # === 云台控制参数 ===
    gimbal_kp_arg = DeclareLaunchArgument(
        'gimbal_kp',
        default_value='0.4',
        description='云台响应灵敏度 (0.2=保守, 0.4=平衡, 0.7=敏感)'
    )
    
    gimbal_dead_zone_arg = DeclareLaunchArgument(
        'gimbal_dead_zone',
        default_value='20',
        description='云台死区像素 (30=稳定, 20=平衡, 10=敏感)'
    )
    
    gimbal_max_step_arg = DeclareLaunchArgument(
        'gimbal_max_step',
        default_value='3.0',
        description='最大单步角度 (度, 2.0=平稳, 3.0=平衡, 5.0=快速)'
    )
    
    # === 高级参数 ===
    enable_debug_arg = DeclareLaunchArgument(
        'enable_debug',
        default_value='false',
        description='启用调试输出 (true/false)'
    )
    
    # ========== 获取参数值 ==========
    yolo_model = LaunchConfiguration('yolo_model')
    yolo_confidence = LaunchConfiguration('yolo_confidence')
    yolo_target_class = LaunchConfiguration('yolo_target_class')
    yolo_device = LaunchConfiguration('yolo_device')
    
    camera_device = LaunchConfiguration('camera_device')
    camera_width = LaunchConfiguration('camera_width')
    camera_height = LaunchConfiguration('camera_height')
    camera_fps = LaunchConfiguration('camera_fps')
    camera_brightness = LaunchConfiguration('camera_brightness')
    camera_contrast = LaunchConfiguration('camera_contrast')
    camera_exposure = LaunchConfiguration('camera_exposure')
    
    gimbal_kp = LaunchConfiguration('gimbal_kp')
    gimbal_dead_zone = LaunchConfiguration('gimbal_dead_zone')
    gimbal_max_step = LaunchConfiguration('gimbal_max_step')
    enable_debug = LaunchConfiguration('enable_debug')
    
    return LaunchDescription([
        # ========== 声明所有参数 ==========
        # YOLO参数
        yolo_model_arg,
        yolo_confidence_arg,
        yolo_target_class_arg,
        yolo_device_arg,
        
        # 相机参数
        camera_device_arg,
        camera_width_arg,
        camera_height_arg,
        camera_fps_arg,
        camera_brightness_arg,
        camera_contrast_arg,
        camera_exposure_arg,
        
        # 云台参数
        gimbal_kp_arg,
        gimbal_dead_zone_arg,
        gimbal_max_step_arg,
        
        # 高级参数
        enable_debug_arg,
        
        # ========== 系统启动信息 ==========
        LogInfo(msg="=========================================="),
        LogInfo(msg="🏎️  Ferrari跟踪系统启动中..."),
        LogInfo(msg="=========================================="),
        LogInfo(msg=[
            "📋 当前配置: ",
            "模型=", yolo_model, " | ",
            "类别=", yolo_target_class, " | ",
            "置信度=", yolo_confidence
        ]),
        LogInfo(msg=[
            "🎥 相机设置: ",
            "设备=", camera_device, " | ",
            "分辨率=", camera_width, "x", camera_height, " | ",
            "帧率=", camera_fps
        ]),
        LogInfo(msg=[
            "🎯 云台设置: ",
            "灵敏度=", gimbal_kp, " | ",
            "死区=", gimbal_dead_zone, "px | ",
            "最大步长=", gimbal_max_step, "°"
        ]),
        LogInfo(msg=""),
        
        # ========== 1. 相机驱动节点 ==========
        LogInfo(msg="🎥 启动相机驱动和图像校正..."),
        Node(
            package='camera_driver',
            executable='camera_driver',
            name='camera_driver',
            parameters=[{
                'device': camera_device,
                'width': camera_width,
                'height': camera_height,
                'fps': 0,  # 使用自动协商帧率，和原始launch文件一致
                'calib_file': 'file:///home/sirius/ssd/ros2workspace/src/camera_driver/calib/usb_cam.yaml',
                
                # V4L2 控件参数 - 使用和原始launch文件相同的配置
                'brightness': 0,
                'contrast': 32,
                'saturation': 60,
                'hue': 0,
                'gamma': 100,
                'gain': 0,
                'power_line_frequency': 1,
                'sharpness': 2,
                'backlight_compensation': 1,
                
                # 曝光控制 - 和原始launch文件保持一致
                'exposure_auto': 0,
                'exposure_time_absolute': camera_exposure
            }],
            remappings=[
                ('camera/image_raw', '/camera/image_raw'),
                ('camera/camera_info', '/camera/camera_info'),
            ],
            output='log'  # 减少终端输出
        ),
        
        # ========== 2. 相机图像校正节点 ==========
        Node(
            package='camera_driver',
            executable='camera_rectify',
            name='camera_rectify',
            remappings=[
                ('/camera/image_raw', '/camera/image_raw'),
                ('/camera/camera_info', '/camera/camera_info'),
                ('/camera/image_rect', '/camera/image_rect'),
            ],
            output='log'
        ),
        
        # ========== 3. 云台控制器节点 ==========
        LogInfo(msg="🎯 启动云台控制器..."),
        Node(
            package='gimbal_controller',
            executable='gimbal_controller',
            name='gimbal_controller',
            parameters=[{
                # 图像参数
                'image_width': camera_width,
                'image_height': camera_height,
                'camera_horizontal_fov': 53.35,  # 真实水平视场角(度)
                'camera_vertical_fov': 41.30,    # 真实垂直视场角(度)
                'use_camera_info': True,          # 自动从camera_info计算FOV
                
                # PID参数 - 使用可配置参数
                'kp_yaw': gimbal_kp,       # 比例系数
                'ki_yaw': 0.0,             # yaw轴积分系数
                'kd_yaw': 0.1,             # yaw轴微分系数
                'kp_pitch': gimbal_kp,     # 比例系数
                'ki_pitch': 0.0,           # pitch轴积分系数
                'kd_pitch': 0.1,           # pitch轴微分系数
                
                # 控制参数 - 使用可配置参数
                'dead_zone_pixels': gimbal_dead_zone,    # 死区像素数
                'max_angle_step': gimbal_max_step,       # 最大单步角度
                'yaw_limit_min': -180.0,     # yaw轴角度范围
                'yaw_limit_max': 180.0,      
                'pitch_limit_min': -45.0,    # pitch轴角度范围
                'pitch_limit_max': 45.0,     
            }],
            remappings=[
                ('/target_position_pixel', '/target_position_pixel'),
                ('/camera/camera_info', '/camera/camera_info'),
                ('/gimbal/angle_cmd', '/gimbal/angle_cmd'),
            ],
            output='screen'  # 显示角度输出
        ),
        
        # ========== 4. YOLO检测器节点 ==========
        LogInfo(msg="🤖 启动YOLO检测器..."),
        ExecuteProcess(
            cmd=[
                '/home/sirius/ssd/ferrari/bin/python3',  # 直接使用虚拟环境的Python
                '/home/sirius/ssd/ros2workspace/src/yolo_detector/scripts/yolo_wrapper.py',
                '--ros-args',
                '-p', ['model_path:=modules/', yolo_model],
                '-p', ['confidence_threshold:=', yolo_confidence],
                '-p', ['target_class:=', yolo_target_class],
                '-p', ['device:=', yolo_device],
            ],
            name='yolo_detector',
            output='screen'
        ),
        
        # ========== 启动完成信息 ==========
        LogInfo(msg=""),
        LogInfo(msg="✅ Ferrari跟踪系统启动完成！"),
        LogInfo(msg="=========================================="),
        LogInfo(msg="📊 监控话题:"),
        LogInfo(msg="  云台命令: ros2 topic echo /gimbal/angle_cmd"),
        LogInfo(msg="  目标位置: ros2 topic echo /target_position_pixel"),
        LogInfo(msg="  检测图像: ros2 topic echo /detection_result_image"),
        LogInfo(msg="  相机图像: ros2 topic echo /camera/image_rect"),
        LogInfo(msg=""),
        LogInfo(msg="🔧 实用命令:"),
        LogInfo(msg="  查看帧率: ros2 topic hz /camera/image_rect"),
        LogInfo(msg="  查看节点: ros2 node list"),
        LogInfo(msg="  重启系统: Ctrl+C 然后重新运行launch"),
        LogInfo(msg=""),
        LogInfo(msg="⚙️  参数调节提示:"),
        LogInfo(msg="  - 增加置信度可减少误检"),
        LogInfo(msg="  - 增加gimbal_kp可提高响应速度"),
        LogInfo(msg="  - 减少dead_zone可提高跟踪精度"),
        LogInfo(msg="=========================================="),
    ])
