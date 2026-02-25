import os
import open3d as o3d
import numpy as np

def rotation_matrix_to_rpy(R):
    """
    ZYX convention (yaw-pitch-roll)
    return roll, pitch, yaw [rad]
    """
    sy = np.sqrt(R[0,0]**2 + R[1,0]**2)

    singular = sy < 1e-6

    if not singular:
        roll  = np.arctan2(R[2,1], R[2,2])
        pitch = np.arctan2(-R[2,0], sy)
        yaw   = np.arctan2(R[1,0], R[0,0])
    else:
        roll  = np.arctan2(-R[1,2], R[1,1])
        pitch = np.arctan2(-R[2,0], sy)
        yaw   = 0

    return roll, pitch, yaw
 
# ===============================
# 0. 파일 경로 (필요에 따라 수정)
# ===============================
_capture_dir = os.path.join(os.getcwd(), "capture")
measured_ply  = os.path.join(_capture_dir, "measured_ROI.ply")   # 측정 ROI PLY
reference_ply = os.path.join(_capture_dir, "reference.ply")      # 레퍼런스 PLY

# ===============================
# 1. Load point clouds
# ===============================
pcd_meas = o3d.io.read_point_cloud(measured_ply)   # camera frame (m)
pcd_ref  = o3d.io.read_point_cloud(reference_ply)  # object frame (mm)

print("[INFO] Measured pts :", len(pcd_meas.points))
print("[INFO] Reference pts:", len(pcd_ref.points))

# ===============================
# 2. Unit unify (m → mm)
# ===============================
pcd_meas.points = o3d.utility.Vector3dVector(
    np.asarray(pcd_meas.points) * 1000.0
)

# ===============================
# 3. Define object frame (reference at origin)
# ===============================
pcd_ref.translate(-pcd_ref.get_center())

# ===============================
# 4. Downsampling
# ===============================
voxel_size = 3.0  # mm
pcd_meas_ds = pcd_meas.voxel_down_sample(voxel_size)
pcd_ref_ds  = pcd_ref.voxel_down_sample(voxel_size)

print("[INFO] Measured DS pts :", len(pcd_meas_ds.points))
print("[INFO] Reference DS pts:", len(pcd_ref_ds.points))

# ===============================
# 5. Estimate normals
# ===============================
pcd_meas_ds.estimate_normals(
    o3d.geometry.KDTreeSearchParamHybrid(radius=10.0, max_nn=50)
)
pcd_ref_ds.estimate_normals(
    o3d.geometry.KDTreeSearchParamHybrid(radius=10.0, max_nn=50)
)

# ===============================
# 6. Initial guess (centroid → origin)
# ===============================
init_T = np.eye(4)
init_T[:3, 3] = -pcd_meas.get_center()

# ===============================
# 7. Coarse ICP (Point-to-Point)
# ===============================
reg_coarse = o3d.pipelines.registration.registration_icp(
    source=pcd_meas_ds,   # measured (camera)
    target=pcd_ref_ds,    # reference (object)
    max_correspondence_distance=200.0,  # mm
    init=init_T,
    estimation_method=o3d.pipelines.registration.TransformationEstimationPointToPoint(),
    criteria=o3d.pipelines.registration.ICPConvergenceCriteria(max_iteration=50)
)

print("\n[Coarse ICP]")
print("Fitness:", reg_coarse.fitness)
print("RMSE   :", reg_coarse.inlier_rmse)

# ===============================
# 8. Fine ICP (Point-to-Plane)
# ===============================
reg_fine = o3d.pipelines.registration.registration_icp(
    source=pcd_meas_ds,
    target=pcd_ref_ds,
    max_correspondence_distance=30.0,  # mm
    init=reg_coarse.transformation,
    estimation_method=o3d.pipelines.registration.TransformationEstimationPointToPlane(),
    criteria=o3d.pipelines.registration.ICPConvergenceCriteria(max_iteration=100)
)

T_cam_to_obj = reg_fine.transformation

print("\n[Fine ICP]")
print("Fitness:", reg_fine.fitness)
print("RMSE   :", reg_fine.inlier_rmse)

# ===============================
# 9. 결과 해석
# ===============================
print("\n[Camera → Object] 4x4 Transform")
print(T_cam_to_obj)


# ===============================
# 10. Object → Camera 변환
# ===============================
T_obj_to_cam = np.linalg.inv(T_cam_to_obj)

print("\n[Object → Camera] 4x4 Transform")
print(T_obj_to_cam)

# ===============================
# 11. Sanity check
# ===============================
print("\nMeasured centroid [mm]:", pcd_meas.get_center())

obj_origin = np.array([0, 0, 0, 1])
print("Object origin via ICP [mm]:",(T_obj_to_cam @ obj_origin)[:3])

x, y, z = (T_obj_to_cam @ obj_origin)[:3]
roll, pitch, yaw = rotation_matrix_to_rpy(T_obj_to_cam[:3, :3])

# ===============================
# 카메라 기준 측정 물체 좌표 (최종 요약)
# ===============================
print("\n" + "="*50)
print("카메라 기준 측정 물체 좌표 (Camera-frame object pose)")
print("="*50)
print("위치 [mm]:")
print("  X = {:.4f}".format(x))
print("  Y = {:.4f}".format(y))
print("  Z = {:.4f}".format(z))
print("자세 [deg] (ZYX: Roll-Pitch-Yaw):")
print("  Roll  = {:.4f}".format(np.degrees(roll)))
print("  Pitch = {:.4f}".format(np.degrees(pitch)))
print("  Yaw   = {:.4f}".format(np.degrees(yaw)))
print("="*50)

# ===============================
# 12. Visualization (Camera = Origin)
# ===============================

# Copy point clouds
pcd_meas_vis = o3d.geometry.PointCloud(pcd_meas_ds)  # already in camera frame
pcd_ref_vis  = o3d.geometry.PointCloud(pcd_ref_ds)   # object frame (centered)

pcd_meas_vis.paint_uniform_color([0, 1, 0])  # measured (green)
pcd_ref_vis.paint_uniform_color([1, 0, 0])   # reference (red)

# Transform reference → camera frame
pcd_ref_vis.transform(T_obj_to_cam)

# -------------------------------
# Camera origin (0,0,0)
# -------------------------------
camera_origin = o3d.geometry.TriangleMesh.create_sphere(radius=15.0)  # mm
camera_origin.paint_uniform_color([0, 0, 0])  # black

# -------------------------------
# Camera coordinate frame (Zivid)
# -------------------------------
camera_frame = o3d.geometry.TriangleMesh.create_coordinate_frame(
    size=150.0,      # axis length [mm]
    origin=[0, 0, 0]
)
# X: Red, Y: Green, Z: Blue (Open3D default = Zivid와 일치)

# -------------------------------
# Draw
# -------------------------------
o3d.visualization.draw_geometries(
    [
        camera_frame,     # XYZ axes at camera
        camera_origin,    # camera origin
        pcd_meas_vis,     # measured points
        pcd_ref_vis      # reference box (aligned)
    ],

    window_name="Camera Frame Visualization (Camera = Origin)"
)
