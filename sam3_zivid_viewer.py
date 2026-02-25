#!/usr/bin/env python3
import json
import math
import cv2
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped
from cv_bridge import CvBridge

from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy

import numpy as np
import open3d as o3d

def quat_to_R(qx, qy, qz, qw):
    # quaternion (x,y,z,w) -> rotation matrix
    x, y, z, w = qx, qy, qz, qw
    R = np.array([
        [1 - 2*(y*y + z*z), 2*(x*y - z*w),     2*(x*z + y*w)],
        [2*(x*y + z*w),     1 - 2*(x*x + z*z), 2*(y*z - x*w)],
        [2*(x*z - y*w),     2*(y*z + x*w),     1 - 2*(x*x + y*y)],
    ], dtype=np.float64)
    return R

def rotation_matrix_to_rpy_zyx(R):
    sy = math.sqrt(R[0,0]**2 + R[1,0]**2)
    singular = sy < 1e-6
    if not singular:
        roll  = math.atan2(R[2,1], R[2,2])
        pitch = math.atan2(-R[2,0], sy)
        yaw   = math.atan2(R[1,0], R[0,0])
    else:
        roll  = math.atan2(-R[1,2], R[1,1])
        pitch = math.atan2(-R[2,0], sy)
        yaw   = 0.0
    return roll, pitch, yaw

class Sam3ZividViewer(Node):
    def __init__(self):
        super().__init__("sam3_zivid_viewer")
        self.bridge = CvBridge()

        self.last_mask = None
        self.last_pose = None  # PoseStamped

        # 3D viewer state
        self.vis = None
        self.o3d_inited = False
        self.geom_added = False

        self.pcd_meas = o3d.geometry.PointCloud()
        self.pcd_ref  = o3d.geometry.PointCloud()
        self.frame    = o3d.geometry.TriangleMesh.create_coordinate_frame(size=150.0, origin=[0,0,0])
        self.origin   = o3d.geometry.TriangleMesh.create_sphere(radius=15.0)
        self.origin.paint_uniform_color([0,0,0])

        qos = QoSProfile(depth=1)
        qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        qos.reliability = ReliabilityPolicy.RELIABLE

        # 2D mask
        self.create_subscription(Image, "/sam3_zivid/mask_vis", self.cb_mask, qos)

        # 3D ply paths (JSON: {"meas_ply": "...", "ref_ply": "..."})
        self.create_subscription(String, "/sam3_zivid/icp_ply", self.cb_icp_ply, qos)

        # pose
        self.create_subscription(PoseStamped, "/sam3_zivid/object_pose", self.cb_pose, 10)

        # OpenCV windows
        cv2.namedWindow("MASK (SAM3)", cv2.WINDOW_NORMAL)
        cv2.resizeWindow("MASK (SAM3)",900,900)

        self.timer = self.create_timer(1.0/30.0, self.draw)
        self.get_logger().info("Viewer started: MASK + pose text (OpenCV), ICP 3D (Open3D)")

    # ---------- OpenCV ----------
    def cb_mask(self, msg: Image):
        try:
            self.last_mask = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except Exception as e:
            self.get_logger().error(f"mask decode failed: {e}")

    def cb_pose(self, msg: PoseStamped):
        self.last_pose = msg

    def _draw_pose_text(self, img_bgr):
        if self.last_pose is None:
            return img_bgr

        p = self.last_pose.pose.position
        q = self.last_pose.pose.orientation

        # PoseStamped is in meters in your publisher -> convert to mm
        dx = p.x * 1000.0
        dy = p.y * 1000.0
        dz = p.z * 1000.0

        R = quat_to_R(q.x, q.y, q.z, q.w)
        roll, pitch, yaw = rotation_matrix_to_rpy_zyx(R)

        # small-ish text
        font = cv2.FONT_HERSHEY_SIMPLEX
        scale = 0.8
        thickness = 1
        color = (255, 255, 255)

        lines = [
            f"dx={dx:7.1f} mm  dy={dy:7.1f} mm  dz={dz:7.1f} mm",
            f"roll={math.degrees(roll):6.1f} deg  pitch={math.degrees(pitch):6.1f} deg  yaw={math.degrees(yaw):6.1f} deg",
        ]

        x, y0 = 12, 24
        dy_pix = 22

        # black background box for readability
        box_w = 880
        box_h = dy_pix * len(lines) + 10
        cv2.rectangle(img_bgr, (8, 8), (8 + box_w, 8 + box_h), (0,0,0), -1)

        for i, line in enumerate(lines):
            y = y0 + i * dy_pix
            cv2.putText(img_bgr, line, (x, y), font, scale, color, thickness, cv2.LINE_AA)

        return img_bgr

    # ---------- Open3D ----------
    def _init_open3d(self):
        if self.o3d_inited:
            return
        self.get_logger().info("Creating Open3D window now...")
        self.vis = o3d.visualization.Visualizer()
        ok = self.vis.create_window(window_name="ICP 3D (Camera frame)", width=900, height=900, visible=True)
        self.get_logger().info(f"Open3D create_window returned: {ok}")
        self.o3d_inited = True

    def _set_camera_view(self):
        """
        Force view to be defined in camera frame:
        - Look at origin (0,0,0)
        - Put eye at some +Z +X direction so you see the scene
        """
        if not self.o3d_inited or self.vis is None:
            return

        ctr = self.vis.get_view_control()
        # camera looking towards origin
        ctr.set_lookat([0.0, 0.0, 0.0])

        # Up direction (Y up)
        ctr.set_up([0.0, -1.0, 0.0])  # this often matches typical camera view; if flipped, use [0,1,0]

        # front direction (where camera points from eye to lookat)
        ctr.set_front([0.0, 0.0, -1.0])

        # zoom: smaller -> farther away (more visible)
        ctr.set_zoom(0.55)

    def cb_icp_ply(self, msg: String):
        try:
            data = json.loads(msg.data)
            meas_ply = data["meas_ply"]
            ref_ply  = data["ref_ply"]
        except Exception as e:
            self.get_logger().error(f"icp_ply JSON parse failed: {e}")
            return

        self._init_open3d()

        try:
            pcd_meas = o3d.io.read_point_cloud(meas_ply)
            pcd_ref  = o3d.io.read_point_cloud(ref_ply)
            if len(pcd_meas.points) == 0 or len(pcd_ref.points) == 0:
                self.get_logger().error("Loaded PLY has no points.")
                return

            pcd_meas.paint_uniform_color([0, 1, 0])  # green
            pcd_ref.paint_uniform_color([1, 0, 0])   # red

            self.pcd_meas = pcd_meas
            self.pcd_ref  = pcd_ref

            if not self.geom_added:
                self.vis.add_geometry(self.frame)
                self.vis.add_geometry(self.origin)
                self.vis.add_geometry(self.pcd_meas)
                self.vis.add_geometry(self.pcd_ref)
                self.geom_added = True
            else:
                self.vis.clear_geometries()
                self.vis.add_geometry(self.frame)
                self.vis.add_geometry(self.origin)
                self.vis.add_geometry(self.pcd_meas)
                self.vis.add_geometry(self.pcd_ref)

            # IMPORTANT: don't reset to object center; force camera-frame view
            self._set_camera_view()

            self.get_logger().info(f"Open3D updated (camera frame): meas={meas_ply} ref={ref_ply}")

        except Exception as e:
            self.get_logger().error(f"Open3D load/update failed: {e}")

    def draw(self):
        if self.last_mask is not None:
            img = self.last_mask.copy()
            img = self._draw_pose_text(img)
            cv2.imshow("MASK (SAM3)", img)

        if self.o3d_inited and self.vis is not None:
            self.vis.poll_events()
            self.vis.update_renderer()

        key = cv2.waitKey(1) & 0xFF
        if key == 27:
            self.get_logger().info("ESC pressed -> shutting down viewer")
            cv2.destroyAllWindows()
            if self.vis is not None:
                self.vis.destroy_window()
            rclpy.shutdown()

def main():
    rclpy.init()
    node = Sam3ZividViewer()
    try:
        rclpy.spin(node)
    finally:
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == "__main__":
    main()
