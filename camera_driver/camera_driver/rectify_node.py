#!/usr/bin/env python3
import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSDurabilityPolicy
from cv_bridge import CvBridge
from sensor_msgs.msg import Image, CameraInfo
from message_filters import Subscriber, ApproximateTimeSynchronizer


class RectifyNode(Node):
    def __init__(self):
        super().__init__('camera_rectify')
        
        # QoS配置
        qos = QoSProfile(
            depth=1,
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.VOLATILE
        )
        
        # 初始化cv_bridge
        self.br = CvBridge()
        
        # 相机参数
        self.camera_matrix = None
        self.distortion_coeffs = None
        self.map1 = None
        self.map2 = None
        self.camera_info_received = False
        
        # 订阅者 - 使用消息同步器确保图像和相机信息对应
        self.image_sub = Subscriber(self, Image, '/camera/image_raw', qos_profile=qos)
        self.info_sub = Subscriber(self, CameraInfo, '/camera/camera_info', qos_profile=qos)
        
        # 时间同步器
        self.sync = ApproximateTimeSynchronizer(
            [self.image_sub, self.info_sub], 
            queue_size=10, 
            slop=0.1
        )
        self.sync.registerCallback(self.sync_callback)
        
        # 发布者
        self.pub_rect = self.create_publisher(Image, '/camera/image_rect', qos)
        
        self.get_logger().info('Camera rectify node started')
    
    def sync_callback(self, image_msg, camera_info_msg):
        """同步回调：处理图像和相机信息"""
        try:
            # 如果相机参数变化了，重新计算去畸变映射
            if not self.camera_info_received or self._camera_info_changed(camera_info_msg):
                self._update_camera_params(camera_info_msg)
            
            # 如果没有有效的相机参数，跳过处理
            if self.map1 is None or self.map2 is None:
                self.get_logger().warn('No valid camera calibration, skipping rectification')
                return
            
            # 转换ROS图像到OpenCV格式
            cv_image = self.br.imgmsg_to_cv2(image_msg, 'bgr8')
            
            # 执行去畸变
            rectified_image = cv2.remap(cv_image, self.map1, self.map2, cv2.INTER_LINEAR)
            
            # 转换回ROS图像消息
            rect_msg = self.br.cv2_to_imgmsg(rectified_image, 'bgr8')
            rect_msg.header = image_msg.header  # 保持相同的时间戳和frame_id
            
            # 发布去畸变后的图像
            self.pub_rect.publish(rect_msg)
            
        except Exception as e:
            self.get_logger().error(f'Error in rectification: {e}')
    
    def _camera_info_changed(self, camera_info_msg):
        """检查相机信息是否发生变化"""
        current_k = np.array(camera_info_msg.k).reshape(3, 3)
        current_d = np.array(camera_info_msg.d)
        
        if self.camera_matrix is None:
            return True
            
        return (not np.array_equal(current_k, self.camera_matrix) or 
                not np.array_equal(current_d, self.distortion_coeffs))
    
    def _update_camera_params(self, camera_info_msg):
        """更新相机参数并重新计算去畸变映射"""
        try:
            # 提取相机内参矩阵和畸变系数
            self.camera_matrix = np.array(camera_info_msg.k).reshape(3, 3)
            self.distortion_coeffs = np.array(camera_info_msg.d)
            
            # 图像尺寸
            image_size = (camera_info_msg.width, camera_info_msg.height)
            
            # 检查是否有有效的畸变系数
            if len(self.distortion_coeffs) == 0 or np.allclose(self.distortion_coeffs, 0):
                self.get_logger().info('No distortion coefficients, using identity mapping')
                # 如果没有畸变，创建单位映射
                self.map1, self.map2 = cv2.initUndistortRectifyMap(
                    self.camera_matrix,
                    np.zeros(5),  # 零畸变
                    np.eye(3),    # 单位校正矩阵
                    self.camera_matrix,
                    image_size,
                    cv2.CV_32FC1
                )
            else:
                # 计算去畸变映射
                self.map1, self.map2 = cv2.initUndistortRectifyMap(
                    self.camera_matrix,
                    self.distortion_coeffs,
                    np.eye(3),  # 校正矩阵，单目相机使用单位矩阵
                    self.camera_matrix,  # 新的相机矩阵，保持原始内参
                    image_size,
                    cv2.CV_32FC1
                )
            
            self.camera_info_received = True
            self.get_logger().info('Camera calibration parameters updated for rectification')
            self.get_logger().info(f'Camera matrix:\n{self.camera_matrix}')
            self.get_logger().info(f'Distortion coefficients: {self.distortion_coeffs}')
            
        except Exception as e:
            self.get_logger().error(f'Failed to update camera parameters: {e}')
            self.camera_matrix = None
            self.distortion_coeffs = None
            self.map1 = None
            self.map2 = None


def main():
    rclpy.init()
    node = RectifyNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
