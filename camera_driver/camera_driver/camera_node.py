#!/usr/bin/env python3
import threading
import cv2
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSDurabilityPolicy
from cv_bridge import CvBridge
from sensor_msgs.msg import Image, CameraInfo
from camera_info_manager import CameraInfoManager, CameraInfoMissingError
import subprocess

class CameraNode(Node):
    def __init__(self):
        super().__init__('camera_driver')
        # 1. 参数声明
        dev    = self.declare_parameter('device', '/dev/video0').value
        width  = self.declare_parameter('width', 640).value
        height = self.declare_parameter('height', 480).value
        fps    = self.declare_parameter('fps', 0).value      # 0=自动协商
        calib  = self.declare_parameter('calib_file','').value
        auto_exposure = self.declare_parameter('auto_exposure', 1).value   # 曝光模式: 1=Manual, 3=AperturePriority
        exposure_time = self.declare_parameter('exposure_time_absolute', 20).value # 曝光时长

        # V4L2 控件参数
        brightness = self.declare_parameter('brightness', 0).value
        contrast = self.declare_parameter('contrast', 32).value
        saturation = self.declare_parameter('saturation', 60).value
        hue = self.declare_parameter('hue', 0).value
        gamma = self.declare_parameter('gamma', 100).value
        gain = self.declare_parameter('gain', 0).value
        power_line_frequency = self.declare_parameter('power_line_frequency', 1).value
        sharpness = self.declare_parameter('sharpness', 2).value
        backlight_compensation = self.declare_parameter('backlight_compensation', 1).value

        # 2. 打开摄像头（GStreamer 管线），可根据需求切换 CAP_V4L2
        if fps > 0:
            fr_str = f',framerate={fps}/1'
        else:
            fr_str = ''
        pipeline = (
            f'v4l2src device={dev} io-mode=2 ! '
            f'image/jpeg,width={width},height={height}{fr_str},format=(string)MJPG ! '
            'jpegparse ! jpegdec ! videoconvert ! '
            'queue max-size-buffers=1 leaky=downstream ! '
            'appsink drop=1'
        )
        # 使用 v4l2-ctl 设置手动/自动曝光参数
        try:
            subprocess.run([
                'v4l2-ctl', '-d', dev,
                '-c', f'auto_exposure={auto_exposure}',
                '-c', f'exposure_time_absolute={exposure_time}',
                '-c', f'brightness={brightness}',
                '-c', f'contrast={contrast}',
                '-c', f'saturation={saturation}',
                '-c', f'hue={hue}',
                '-c', f'gamma={gamma}',
                '-c', f'gain={gain}',
                '-c', f'power_line_frequency={power_line_frequency}',
                '-c', f'sharpness={sharpness}',
                '-c', f'backlight_compensation={backlight_compensation}'
            ], check=True)
            self.get_logger().info(
                f'Set exposure via v4l2-ctl: auto_exposure={auto_exposure}, exposure_time_absolute={exposure_time}')
        except subprocess.CalledProcessError as e:
            self.get_logger().warn(f'Failed to set exposure via v4l2-ctl: {e}')
        self.cap = cv2.VideoCapture(pipeline, cv2.CAP_GSTREAMER)
        if not self.cap.isOpened():
            self.get_logger().fatal(f'无法打开摄像头设备：{dev}')
            raise RuntimeError(f'Unable to open {dev}')

        # 3. 获取实际帧率，并限定缓冲深度
        eff = self.cap.get(cv2.CAP_PROP_FPS)
        self.eff_fps = eff if eff > 1e-3 else 30.0
        self.get_logger().info(f'摄像头实际帧率 ≈ {self.eff_fps:.1f} FPS')
        # self.cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)



        # 4. 后台线程不断拉帧到缓存
        self.lock   = threading.Lock()
        self.latest = None
        self.running = True
        threading.Thread(target=self._grab_loop, daemon=True).start()

        # 5. 创建 ROS2 发布器（最佳努力+深度1）
        qos = QoSProfile(
            depth=1,
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.VOLATILE
        )
        self.br       = CvBridge()
        self.pub_img  = self.create_publisher(Image,      'camera/image_raw',  qos)
        self.pub_info = self.create_publisher(CameraInfo, 'camera/camera_info', qos)
        
        # 相机信息管理器设置
        self.cim = CameraInfoManager(self, 'narrow_stereo', calib)  # 正确的初始化方式
        self.camera_info_loaded = False
        if calib:
            self.get_logger().info(f'Loading camera calibration from: {calib}')
            try:
                self.cim.loadCameraInfo()  # 无参数调用
                self.get_logger().info('Camera calibration loaded successfully')
            except Exception as e:
                self.get_logger().warn(f'Failed to load camera calibration with CameraInfoManager: {e}')
                # 手动加载标定文件
                self._load_calibration_manually(calib)

        # 6. 定时器用来轮询并发布新帧
        self.create_timer(0.005, self._pub_loop)

    def _grab_loop(self):
        """后台线程：不断从 cap.read() 拉帧，存到 self.latest"""
        while self.running:
            ret, frame = self.cap.read()
            if ret:
                with self.lock:
                    self.latest = frame
            else:
                self.get_logger().warn('Frame grab failed')
            # 可选 time.sleep(0.001)

    def _pub_loop(self):
        """定时器回调：检测缓存，有帧就发布 Image + CameraInfo"""
        with self.lock:
            frame = self.latest
            self.latest = None
        if frame is None:
            return

        # 构造 Image 消息
        stamp   = self.get_clock().now().to_msg()
        img_msg = self.br.cv2_to_imgmsg(frame, 'bgr8')
        img_msg.header.stamp    = stamp
        img_msg.header.frame_id = 'camera_optical_frame'

        # 构造 CameraInfo
        info_msg = CameraInfo()
        info_msg.width  = frame.shape[1]
        info_msg.height = frame.shape[0]
        
        # 应用标定参数
        if hasattr(self, 'camera_info_loaded') and self.camera_info_loaded:
            # 使用加载的标定参数
            info_msg.k = self.camera_matrix
            info_msg.d = self.distortion_coefficients  
            info_msg.r = self.rectification_matrix
            info_msg.p = self.projection_matrix
            info_msg.distortion_model = 'plumb_bob'
        else:
            # 尝试从CameraInfoManager获取，失败则使用默认值
            try:
                info_msg = self.cim.getCameraInfo()
            except CameraInfoMissingError:
                h, w = frame.shape[:2]
                # 默认单位内参矩阵，焦距=1，主点在中心
                info_msg.k = [
                    1.0, 0.0, float(w) / 2.0,
                    0.0, 1.0, float(h) / 2.0,
                    0.0, 0.0, 1.0
                ]
                # 默认零畸变
                info_msg.d = [0.0, 0.0, 0.0, 0.0, 0.0]
        info_msg.header = img_msg.header

        # 发布
        self.pub_img.publish(img_msg)
        self.pub_info.publish(info_msg)

    def _load_calibration_manually(self, calib_url):
        """手动加载标定文件"""
        import yaml
        import os
        
        try:
            # 处理file://前缀
            if calib_url.startswith('file://'):
                file_path = calib_url[7:]  # 移除file://前缀
            else:
                file_path = calib_url
                
            if not os.path.exists(file_path):
                self.get_logger().error(f'Calibration file not found: {file_path}')
                return
                
            with open(file_path, 'r') as f:
                calib_data = yaml.safe_load(f)
                
            # 提取标定参数
            self.camera_matrix = calib_data['camera_matrix']['data']
            self.distortion_coefficients = calib_data['distortion_coefficients']['data']
            self.rectification_matrix = calib_data['rectification_matrix']['data'] 
            self.projection_matrix = calib_data['projection_matrix']['data']
            
            self.camera_info_loaded = True
            self.get_logger().info('Camera calibration loaded manually successfully')
            
        except Exception as e:
            self.get_logger().error(f'Failed to manually load calibration: {e}')

    def destroy_node(self):
        """退出时清理"""
        self.running = False
        super().destroy_node()
        self.cap.release()

def main():
    rclpy.init()
    node = CameraNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()