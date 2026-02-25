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
import open3d as o3d
import numpy as np
import os
from datetime import datetime
import torch
from PIL import Image as PILImage
from sam3.model_builder import build_sam3_image_model
from sam3.model.sam3_image_processor import Sam3Processor


input_prompt="silver metal rod"

class ZividCaptureROI(Node):
    def __init__(self):
        super().__init__('zivid_capture_roi')

        self.bridge = CvBridge()
        self.image = None

        # SAM3 모델 로드
        device = "cuda" if torch.cuda.is_available() else "cpu"
        self.model = build_sam3_image_model().to(device)
        self.processor = Sam3Processor(self.model)  # 여기서 self.processor 생성

        # ROS2 구독 및 서비스 클라이언트
        self.create_subscription(Image, '/color/image_color', self.image_callback, 10)
        self.capture_2d = self.create_client(Trigger, '/capture_2d')
        self.capture_3d = self.create_client(CaptureAndSave, '/capture_and_save')
        self.param_client = self.create_client(SetParameters, '/zivid_camera/set_parameters')

        # 서비스 대기
        self.get_logger().info('Waiting for Zivid services...')
        self.capture_2d.wait_for_service()
        self.capture_3d.wait_for_service()
        self.param_client.wait_for_service()
        self.get_logger().info('Zivid services ready')

        self.set_2d_settings()
        self.set_3d_settings()

    def image_callback(self, msg):
        self.image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')

    def set_2d_settings(self):
        self.get_logger().info('Setting 2D settings')
        param = Parameter(
            'settings_2d_yaml',
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

    def set_3d_settings(self):
        self.get_logger().info('Setting 3D settings')
        param = Parameter(
            'settings_yaml',
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

    def run(self):
        save_dir = os.path.join(os.getcwd(), "capture")
        os.makedirs(save_dir, exist_ok=True)
        ts = datetime.now().strftime("%Y.%m.%d_%H.%M.%S")

        # ---------- 3D capture (1회) ----------
        self.get_logger().info('Capturing 3D (ZDF + PLY)...')
        self.image = None  # 최신 이미지 대기용

        zdf_path = f'{save_dir}/{input_prompt}_zdf_{ts}.zdf'
        ply_path = f'{save_dir}/{input_prompt}_ply_{ts}.ply'

        req = CaptureAndSave.Request()
        # req.file_path = zdf_path
        # future = self.capture_3d.call_async(req)
        # rclpy.spin_until_future_complete(self, future)

        # if not future.result().success:
        #     raise RuntimeError(future.result().message)

        # 같은 프레임으로 PLY 저장
        req.file_path = ply_path
        future = self.capture_3d.call_async(req)
        rclpy.spin_until_future_complete(self, future)

        if not future.result().success:
            raise RuntimeError(future.result().message)

        self.get_logger().info(f'Saved ZDF: {zdf_path}')
        self.get_logger().info(f'Saved PLY: {ply_path}')

        # ---------- RGB 이미지 수신 대기 ----------
        for _ in range(10):
            rclpy.spin_once(self, timeout_sec=0.3)
            if self.image is not None:
                break

        if self.image is None:
            raise RuntimeError("No RGB image received from Zivid")

        # ---------- SAM3 ----------
        self.get_logger().info('Generating SAM3 mask...')
        frame_rgb = cv2.cvtColor(self.image, cv2.COLOR_BGR2RGB)
        pil_image = PILImage.fromarray(frame_rgb)

        inference_state = self.processor.set_image(pil_image)
        output = self.processor.set_text_prompt(
            state=inference_state,
            prompt=input_prompt
        )

        masks = output.get("masks", [])
        if len(masks) == 0:
            self.get_logger().warn("No masks detected by SAM3!")
            return

        if torch.is_tensor(masks):
            masks_np = masks.detach().cpu().numpy()
        else:
            masks_np = np.array(masks)

        masks_np = np.squeeze(masks_np)
        if masks_np.ndim == 2:
            masks_np = masks_np[None, ...]

        areas = masks_np.reshape(masks_np.shape[0], -1).sum(axis=1)
        mask = masks_np[np.argmax(areas)] > 0
        mask_h, mask_w = mask.shape

        self.get_logger().info(f"SAM3 mask shape: {mask.shape}")
        
        
        # ---------- 2D Mask Visualization (Save Only) ----------
        overlay = self.image.copy()
        # grayscale → BGR (안전)
        if overlay.ndim == 2:
            overlay = cv2.cvtColor(overlay, cv2.COLOR_GRAY2BGR)
        # mask overlay (green)
        overlay[mask] = (
            0.7 * overlay[mask] + 0.3 * np.array([0, 200, 0])
        ).astype(np.uint8)
        # 저장 경로
        overlay_path = f"{save_dir}/{input_prompt}_SAM3_overlay_{ts}.png"
        cv2.imwrite(overlay_path, overlay)
        self.get_logger().info(f"Saved SAM3 overlay image: {overlay_path}")

        # ---------- 3D ROI (PLY 기반) ----------
        pcd = o3d.io.read_point_cloud(ply_path)
        points = np.asarray(pcd.points)

        # Zivid PLY는 organized cloud
        xyz = points.reshape(mask_h, mask_w, 3)

        roi_points = xyz[mask]
        roi_points = roi_points[np.isfinite(roi_points).all(axis=1)]

        roi_pcd = o3d.geometry.PointCloud()
        roi_pcd.points = o3d.utility.Vector3dVector(roi_points)

        roi_path = f'{save_dir}/{input_prompt}_ROI_{ts}.ply'
        o3d.io.write_point_cloud(roi_path, roi_pcd)
        
        self.get_logger().info(f'Saved ROI PLY: {roi_path}')
        o3d.visualization.draw_geometries([roi_pcd])



def main():
    rclpy.init()
    node = ZividCaptureROI()
    node.run()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
