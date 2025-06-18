from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='gimbal_controller',
            executable='gimbal_controller',
            name='gimbal_controller',
            parameters=[{
                # 图像参数
                'image_width': 640,
                'image_height': 480,
                'camera_horizontal_fov': 53.35,  # 真实水平视场角(度)
                'camera_vertical_fov': 41.30,    # 真实垂直视场角(度)
                'use_camera_info': True,          # 自动从camera_info计算FOV
                
                # PID参数
                'kp_yaw': 0.5,      # yaw轴比例系数
                'ki_yaw': 0.0,      # yaw轴积分系数
                'kd_yaw': 0.1,      # yaw轴微分系数
                'kp_pitch': 0.5,    # pitch轴比例系数
                'ki_pitch': 0.0,    # pitch轴积分系数
                'kd_pitch': 0.1,    # pitch轴微分系数
                
                # 控制参数
                'dead_zone_pixels': 20,      # 死区像素数
                'max_angle_step': 3.0,       # 最大单步角度(度)
                'yaw_limit_min': -90.0,      # yaw轴最小角度
                'yaw_limit_max': 90.0,       # yaw轴最大角度
                'pitch_limit_min': -30.0,    # pitch轴最小角度
                'pitch_limit_max': 30.0,     # pitch轴最大角度
            }],
            remappings=[
                ('/target_position_pixel', '/target_position_pixel'),
                ('/camera/camera_info', '/camera/camera_info'),
                ('/gimbal/angle_cmd', '/gimbal/angle_cmd'),
            ]
        )
    ])
