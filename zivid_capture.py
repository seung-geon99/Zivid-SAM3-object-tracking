#!/usr/bin/env python3
import os
import json
from datetime import datetime

import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rcl_interfaces.srv import SetParameters

from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge

from std_srvs.srv import Trigger
from zivid_interfaces.srv import CaptureAndSave


class ZividCapture(Node):
    def __init__(self):
        super().__init__("zivid_capture")

        self.bridge = CvBridge()
        self.last_rgb_msg = None

        # ---- params ----
        self.declare_parameter("input_prompt", "silver metal rod")
        self.declare_parameter("save_dir", os.path.join(os.getcwd(), "capture"))

        # zivid_camera param service + target node name
        self.declare_parameter("zivid_node_name", "/zivid_camera")
        self.declare_parameter("zivid_setparam_service", "/zivid_camera/set_parameters")

        # 2D/3D settings YAML (기존 코드 그대로)
        self.declare_parameter(
            "settings_2d_yaml",
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
        )
        self.declare_parameter(
            "settings_3d_yaml",
            """
__version__: 27
Settings:
  Acquisitions:
    - Acquisition:
        Aperture: 5.66
        ExposureTime: 8333
        Gain: 1.0
"""
        )

        self.input_prompt = self.get_parameter("input_prompt").value
        self.save_dir = self.get_parameter("save_dir").value
        os.makedirs(self.save_dir, exist_ok=True)

        self.zivid_node_name = self.get_parameter("zivid_node_name").value
        self.zivid_setparam_service = self.get_parameter("zivid_setparam_service").value
        self.settings_2d_yaml = self.get_parameter("settings_2d_yaml").value
        self.settings_3d_yaml = self.get_parameter("settings_3d_yaml").value

        # ---- Zivid subscribe ----
        self.create_subscription(Image, "/color/image_color", self.rgb_cb, 10)

        # ---- Zivid service clients ----
        self.capture_3d = self.create_client(CaptureAndSave, "/capture_and_save")
        self.capture_2d = self.create_client(Trigger, "/capture_2d")

        # ---- Zivid set_parameters client (핵심) ----
        self.param_client = self.create_client(SetParameters, self.zivid_setparam_service)

        self.get_logger().info("Waiting for Zivid services...")
        self.capture_3d.wait_for_service()
        self.capture_2d.wait_for_service()
        self.param_client.wait_for_service()
        self.get_logger().info("Zivid services ready.")

        # ---- publish to sam3_zivid ----
        from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy

        qos_latched = QoSProfile(depth=1)
        qos_latched.durability = DurabilityPolicy.TRANSIENT_LOCAL
        qos_latched.reliability = ReliabilityPolicy.RELIABLE

        self.rgb_pub  = self.create_publisher(Image,  "/sam3_zivid/rgb",  qos_latched)
        self.meta_pub = self.create_publisher(String, "/sam3_zivid/meta", qos_latched)

    def rgb_cb(self, msg: Image):
        self.last_rgb_msg = msg

    # ---------- settings ----------
    def _set_zivid_param(self, name: str, value: str):
        param_msg = Parameter(name, Parameter.Type.STRING, value).to_parameter_msg()
        req = SetParameters.Request()
        req.parameters = [param_msg]
        future = self.param_client.call_async(req)
        rclpy.spin_until_future_complete(self, future)

        if not future.result():
            raise RuntimeError(f"SetParameters failed (no response) for {name}")

        # best-effort: check result flags if available
        results = getattr(future.result(), "results", None)
        if results and len(results) > 0 and not results[0].successful:
            reason = getattr(results[0], "reason", "")
            raise RuntimeError(f"SetParameters failed for {name}: {reason}")

    def set_2d_settings(self):
        self.get_logger().info("Setting Zivid 2D settings via parameters")
        self._set_zivid_param("settings_2d_yaml", self.settings_2d_yaml)

    def set_3d_settings(self):
        self.get_logger().info("Setting Zivid 3D settings via parameters")
        # zivid_camera가 요구하는 이름이 settings_file_path / settings_yaml 이었음
        self._set_zivid_param("settings_yaml", self.settings_3d_yaml)

    # ---------- main job ----------
    def run_once(self):
        ts = datetime.now().strftime("%Y.%m.%d_%H.%M.%S")
        ply_path = f"{self.save_dir}/{self.input_prompt}_ply_{ts}.ply"

        # 1) 세팅 먼저 적용 (핵심)
        self.set_2d_settings()
        self.set_3d_settings()

        # 2) 2D 캡처 트리거 -> 최신 RGB 기다리기
        self.last_rgb_msg = None
        self.get_logger().info("Capturing 2D (trigger) and waiting for RGB")
        self.capture_2d.call_async(Trigger.Request())

        for _ in range(30):  # 3초 정도
            rclpy.spin_once(self, timeout_sec=0.1)
            if self.last_rgb_msg is not None:
                break
        if self.last_rgb_msg is None:
            raise RuntimeError("No RGB received from Zivid after capture_2d.")

        # 3) 3D 캡처 (PLY 저장)
        self.get_logger().info(f"Capturing 3D PLY -> {ply_path}")
        req = CaptureAndSave.Request()
        req.file_path = ply_path
        future = self.capture_3d.call_async(req)
        rclpy.spin_until_future_complete(self, future)

        if not future.result() or not future.result().success:
            raise RuntimeError(f"Capture failed: {future.result().message if future.result() else 'no response'}")

        # 4) RGB publish to sam3_zivid
        self.get_logger().info(f"About to publish RGB. last_rgb_msg is {'SET' if self.last_rgb_msg else 'NONE'}")
        self.rgb_pub.publish(self.last_rgb_msg)

        # 5) meta publish
        meta = {
            "prompt": self.input_prompt,
            "ply_path": ply_path,
            "stamp_sec": int(self.last_rgb_msg.header.stamp.sec),
            "stamp_nanosec": int(self.last_rgb_msg.header.stamp.nanosec),
        }
        meta_msg = String()
        meta_msg.data = json.dumps(meta)
        self.meta_pub.publish(meta_msg)

        self.get_logger().info("[DONE] Captured 2D+3D, saved PLY, and published RGB+meta to /sam3_zivid/*")
        for _ in range(40):  # 2.0 sec
         rclpy.spin_once(self, timeout_sec=0.05)


def main():
    rclpy.init()
    node = ZividCapture()
    try:
        node.run_once()
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
