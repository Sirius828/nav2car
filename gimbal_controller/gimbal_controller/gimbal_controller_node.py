#!/usr/bin/env python3
import math
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSDurabilityPolicy
from geometry_msgs.msg import Point, Vector3
from sensor_msgs.msg import CameraInfo


class GimbalController(Node):
    def __init__(self):
        super().__init__('gimbal_controller')
        
        # 参数声明
        self.declare_parameter('image_width', 640)
        self.declare_parameter('image_height', 480)
        self.declare_parameter('camera_horizontal_fov', 53.35)  # 根据相机标定的真实水平视场角(度)
        self.declare_parameter('camera_vertical_fov', 41.30)    # 根据相机标定的真实垂直视场角(度)
        self.declare_parameter('use_camera_info', True)         # 是否使用camera_info自动计算FOV（推荐开启）
        
        # PID参数
        self.declare_parameter('kp_yaw', 1.0)      # yaw轴比例系数
        self.declare_parameter('ki_yaw', 0.0)      # yaw轴积分系数  
        self.declare_parameter('kd_yaw', 0.1)      # yaw轴微分系数
        self.declare_parameter('kp_pitch', 1.0)    # pitch轴比例系数
        self.declare_parameter('ki_pitch', 0.0)    # pitch轴积分系数
        self.declare_parameter('kd_pitch', 0.1)    # pitch轴微分系数
        
        # 控制参数
        self.declare_parameter('dead_zone_pixels', 10)     # 死区像素数
        self.declare_parameter('max_angle_step', 5.0)      # 最大单步角度(度)
        self.declare_parameter('yaw_limit_min', -90.0)     # yaw轴最小角度
        self.declare_parameter('yaw_limit_max', 90.0)      # yaw轴最大角度
        self.declare_parameter('pitch_limit_min', -30.0)   # pitch轴最小角度
        self.declare_parameter('pitch_limit_max', 30.0)    # pitch轴最大角度
        
        # 获取参数
        self.image_width = self.get_parameter('image_width').value
        self.image_height = self.get_parameter('image_height').value
        self.h_fov = math.radians(self.get_parameter('camera_horizontal_fov').value)
        self.v_fov = math.radians(self.get_parameter('camera_vertical_fov').value)
        self.use_camera_info = self.get_parameter('use_camera_info').value
        
        # PID参数
        self.kp_yaw = self.get_parameter('kp_yaw').value
        self.ki_yaw = self.get_parameter('ki_yaw').value
        self.kd_yaw = self.get_parameter('kd_yaw').value
        self.kp_pitch = self.get_parameter('kp_pitch').value
        self.ki_pitch = self.get_parameter('ki_pitch').value
        self.kd_pitch = self.get_parameter('kd_pitch').value
        
        # 控制参数
        self.dead_zone = self.get_parameter('dead_zone_pixels').value
        self.max_step = self.get_parameter('max_angle_step').value
        self.yaw_min = self.get_parameter('yaw_limit_min').value
        self.yaw_max = self.get_parameter('yaw_limit_max').value
        self.pitch_min = self.get_parameter('pitch_limit_min').value
        self.pitch_max = self.get_parameter('pitch_limit_max').value
        
        # 控制状态
        self.current_yaw = 0.0
        self.current_pitch = 0.0
        self.last_error_yaw = 0.0
        self.last_error_pitch = 0.0
        self.integral_yaw = 0.0
        self.integral_pitch = 0.0
        self.last_time = self.get_clock().now()
        
        # 图像中心
        self.image_center_x = self.image_width // 2
        self.image_center_y = self.image_height // 2
        
        # QoS配置
        qos = QoSProfile(
            depth=1,
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.VOLATILE
        )
        
        # 订阅者
        self.target_sub = self.create_subscription(
            Point,
            '/target_position_pixel',
            self.target_callback,
            qos
        )
        
        # 如果启用camera_info，订阅相机信息来自动计算FOV
        if self.use_camera_info:
            self.camera_info_sub = self.create_subscription(
                CameraInfo,
                '/camera/camera_info',
                self.camera_info_callback,
                qos
            )
        
        # 发布者
        self.gimbal_cmd_pub = self.create_publisher(
            Vector3,
            '/gimbal/angle_cmd',
            qos
        )
        
        self.get_logger().info('Gimbal controller started')
        self.get_logger().info(f'Image size: {self.image_width}x{self.image_height}')
        self.get_logger().info(f'FOV: {math.degrees(self.h_fov):.1f}° x {math.degrees(self.v_fov):.1f}°')
        self.get_logger().info(f'Dead zone: ±{self.dead_zone} pixels')
    
    def camera_info_callback(self, msg):
        """根据相机内参自动计算视场角"""
        if msg.k[0] > 0 and msg.k[4] > 0:  # 检查内参矩阵是否有效
            # 从相机内参计算FOV
            fx = msg.k[0]  # 水平焦距
            fy = msg.k[4]  # 垂直焦距
            
            # FOV = 2 * arctan(image_size / (2 * focal_length))
            new_h_fov = 2 * math.atan(self.image_width / (2 * fx))
            new_v_fov = 2 * math.atan(self.image_height / (2 * fy))
            
            # 更新FOV（避免频繁更新）
            if abs(new_h_fov - self.h_fov) > 0.01 or abs(new_v_fov - self.v_fov) > 0.01:
                self.h_fov = new_h_fov
                self.v_fov = new_v_fov
                self.get_logger().info(f'FOV updated from camera_info: {math.degrees(self.h_fov):.1f}° x {math.degrees(self.v_fov):.1f}°')
    
    def target_callback(self, msg):
        """目标位置回调：计算控制角度"""
        try:
            # 计算像素偏移
            pixel_error_x = msg.x - self.image_center_x  # 正值表示目标在右侧
            pixel_error_y = msg.y - self.image_center_y  # 正值表示目标在下方
            
            # 死区检查
            if abs(pixel_error_x) < self.dead_zone and abs(pixel_error_y) < self.dead_zone:
                self.get_logger().debug('Target in dead zone, no movement needed')
                return
            
            # 像素偏移转角度偏移
            angle_error_yaw = pixel_error_x * (self.h_fov / self.image_width)
            angle_error_pitch = -pixel_error_y * (self.v_fov / self.image_height)  # 图像y向下，云台pitch向上为正
            
            # PID控制计算
            current_time = self.get_clock().now()
            dt = (current_time - self.last_time).nanoseconds / 1e9
            
            if dt > 0:
                # YAW轴PID
                yaw_cmd = self.pid_control(
                    angle_error_yaw, self.last_error_yaw, self.integral_yaw,
                    self.kp_yaw, self.ki_yaw, self.kd_yaw, dt
                )
                
                # PITCH轴PID  
                pitch_cmd = self.pid_control(
                    angle_error_pitch, self.last_error_pitch, self.integral_pitch,
                    self.kp_pitch, self.ki_pitch, self.kd_pitch, dt
                )
                
                # 更新历史误差
                self.last_error_yaw = angle_error_yaw
                self.last_error_pitch = angle_error_pitch
                self.last_time = current_time
                
                # 限制最大步长
                yaw_cmd = self.clamp(yaw_cmd, -self.max_step, self.max_step)
                pitch_cmd = self.clamp(pitch_cmd, -self.max_step, self.max_step)
                
                # 计算目标角度（相对当前位置）
                target_yaw = self.current_yaw + math.degrees(yaw_cmd)
                target_pitch = self.current_pitch + math.degrees(pitch_cmd)
                
                # 角度限位
                target_yaw = self.clamp(target_yaw, self.yaw_min, self.yaw_max)
                target_pitch = self.clamp(target_pitch, self.pitch_min, self.pitch_max)
                
                # 发布控制指令
                self.publish_gimbal_cmd(target_yaw, target_pitch, confidence=msg.z)
                
                # 更新当前角度估计
                self.current_yaw = target_yaw
                self.current_pitch = target_pitch
                
                self.get_logger().info(
                    f'Target: ({msg.x:.1f}, {msg.y:.1f}) '
                    f'Center: ({self.image_center_x}, {self.image_center_y}) '
                    f'PixelError: ({pixel_error_x:.1f}, {pixel_error_y:.1f}) '
                    f'AngleError: ({math.degrees(angle_error_yaw):.2f}°, {math.degrees(angle_error_pitch):.2f}°) '
                    f'Cmd: ({target_yaw:.2f}°, {target_pitch:.2f}°)'
                )
            
        except Exception as e:
            self.get_logger().error(f'Error in target callback: {e}')
    
    def pid_control(self, error, last_error, integral, kp, ki, kd, dt):
        """PID控制计算"""
        # 积分项
        integral += error * dt
        
        # 微分项
        derivative = (error - last_error) / dt if dt > 0 else 0.0
        
        # PID输出
        output = kp * error + ki * integral + kd * derivative
        
        return output
    
    def clamp(self, value, min_val, max_val):
        """数值限幅"""
        return max(min_val, min(max_val, value))
    
    def publish_gimbal_cmd(self, yaw, pitch, confidence=1.0):
        """发布云台控制指令"""
        cmd_msg = Vector3()
        cmd_msg.x = yaw      # yaw角度
        cmd_msg.y = pitch    # pitch角度  
        cmd_msg.z = 1.0      # 控制模式：1.0=绝对角度
        
        self.gimbal_cmd_pub.publish(cmd_msg)


def main(args=None):
    rclpy.init(args=args)
    node = GimbalController()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
