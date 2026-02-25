#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rcl_interfaces.srv import SetParameters
from std_srvs.srv import Trigger
from zivid_interfaces.srv import CaptureAndSave
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import os
import time
from datetime import datetime

class ZividCaptureOnce(Node):
    def __init__(self):
        super().__init__('zivid_capture_once')

        self.bridge = CvBridge()
        self.image = None

        self.create_subscription(
            Image,
            '/color/image_color',
            self.image_callback,
            10
        )
        self.capture_2d = self.create_client(Trigger, '/capture_2d')
        self.capture_3d = self.create_client(CaptureAndSave, '/capture_and_save')
        self.param_client = self.create_client(
            SetParameters, '/zivid_camera/set_parameters'
        )

        self.get_logger().info('Waiting for services...')
        self.capture_2d.wait_for_service()
        self.capture_3d.wait_for_service()
        self.param_client.wait_for_service()
        self.get_logger().info('Services ready')

        self.set_2d_settings()
        self.set_3d_settings()

    def image_callback(self, msg):
        self.image = self.bridge.imgmsg_to_cv2(msg, 'bgr8') # 

    def set_2d_settings(self):
        self.get_logger().info('Setting 2D settings')
        param = Parameter(
            'settings_2d_yaml',
            # parameter 형식 건들면 안됨
            Parameter.Type.STRING,
            """
__version__:
  serializer: 1
  data: 3
Settings2D:
  Acquisitions:
    - Acquisition:
        Aperture: 2.83
        ExposureTime: 10000
        Gain: 2.5
"""
        ).to_parameter_msg()

        req = SetParameters.Request()
        req.parameters = [param]

        future = self.param_client.call_async(req)
        rclpy.spin_until_future_complete(self, future)

    def run(self):
        # 저장 경로 수정
        save_dir = os.path.join(os.getcwd(), "capture")
        os.makedirs(save_dir, exist_ok=True)
        ts = datetime.now().strftime("%Y.%m.%d_%H.%M.%S")

        # ---------- 2D ----------
        self.get_logger().info('Capturing 2D')
        self.capture_2d.call_async(Trigger.Request())

        for _ in range(10):
            rclpy.spin_once(self, timeout_sec=0.3)
            if self.image is not None:
                break

        if self.image is None:
            raise RuntimeError('2D image not received')

        png_path = f'{save_dir}/Zivid_{ts}.png'
        cv2.imwrite(png_path, self.image)
        self.get_logger().info(f'Saved {png_path}')

        # ---------- 3D ----------
        self.get_logger().info('Capturing 3D')
        zdf_path = f'{save_dir}/Zivid_{ts}.zdf'
        ply_path = f'{save_dir}/Zivid_{ts}.ply'

        req = CaptureAndSave.Request()
        req.file_path = zdf_path

        future = self.capture_3d.call_async(req)
        rclpy.spin_until_future_complete(self, future)

        if not future.result().success:
            raise RuntimeError(future.result().message)

        self.get_logger().info(f'Saved 3D ZDF: {zdf_path}')

        req_ply = CaptureAndSave.Request()
        req_ply.file_path = ply_path
        future = self.capture_3d.call_async(req_ply)
        rclpy.spin_until_future_complete(self, future)

        if not future.result().success:
            raise RuntimeError(future.result().message)

        self.get_logger().info(f'Saved 3D PLY: {ply_path}')


    def set_3d_settings(self):
        self.get_logger().info('Setting 3D settings')
        param = Parameter(
            'settings_yaml',
            # parameter 형식 건들면 안됨
            Parameter.Type.STRING,
            """
__version__: 27
Settings:
  Acquisitions:
    - Acquisition:
        Aperture: 5.66
        ExposureTime: 8333
        Gain: 1.0
"""
        ).to_parameter_msg()
        req = SetParameters.Request()
        req.parameters = [param]
        future = self.param_client.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        if not future.result():
            raise RuntimeError('Failed to set 3D settings')


def main():
    rclpy.init()
    node = ZividCaptureOnce()
    node.run()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

