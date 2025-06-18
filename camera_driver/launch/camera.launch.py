from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='camera_driver',
            executable='camera_driver',
            name='camera_driver',
            parameters=[{
                'device':        '/dev/video0',  
                'width':         640,             # 图像宽度 (px)
                'height':        480,             # 图像高度 (px)
                'fps':           0,               # 0=自动协商帧率
                'calib_file':    'file:///home/sirius/ssd/ros2workspace/src/camera_driver/calib/usb_cam.yaml',

                # V4L2 控件参数及可选值
                'brightness':                  0,    # 亮度: -64…64
                'contrast':                    32,   # 对比度: 0…64
                'saturation':                  60,   # 饱和度: 0…128
                'hue':                         0,    # 色调: -40…40
                'gamma':                       100,  # 伽马校正: 72…500
                'gain':                        0,    # 增益: 0…100
                'power_line_frequency':        1,    # 频闪抑制: 0=None,1=50Hz,2=60Hz
                'sharpness':                   2,    # 锐度: 0…6
                'backlight_compensation':      1,    # 背光补偿: 0…2

                # 曝光控制
                'auto_exposure':               1,    # 曝光模式: 1=Manual,3=AperturePriority
                'exposure_time_absolute':      30,    # 绝对曝光时间 (单位 us)，仅在 Manual 或 ShutterPriority 生效 相机最高就是90附近，并非120
            }],
            remappings=[
                ('camera/image_raw',  '/camera/image_raw'),
                ('camera/camera_info','/camera/camera_info'),
            ]
        ),
        Node(
            package='camera_driver',
            executable='camera_rectify',
            name='camera_rectify',
            remappings=[
                ('/camera/image_raw',  '/camera/image_raw'),
                ('/camera/camera_info','/camera/camera_info'),
                ('/camera/image_rect', '/camera/image_rect'),
            ]
        )
    ])