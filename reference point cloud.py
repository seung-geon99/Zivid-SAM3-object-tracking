import open3d as o3d
import numpy as np



# # 박스 치수 (mm)
# W, D, H = 275, 225, 225

# # 박스 메쉬 생성
# mesh = o3d.geometry.TriangleMesh.create_box(width=W, height=D, depth=H)
# mesh.translate([-W/2, -D/2, -H/2])  # 중심 원점

# # 표면 포인트 샘플링
# pcd_ref = mesh.sample_points_uniformly(number_of_points=5000)

# o3d.io.write_point_cloud("box_reference.ply", pcd_ref)

# ===============================
# Cylinder specification (mm)
# ===============================
RADIUS = 8.5     # mm
HEIGHT = 106.0  # mm
N_POINTS = 6000

# ===============================
# Create cylinder mesh
# ===============================
mesh = o3d.geometry.TriangleMesh.create_cylinder(
    radius=RADIUS,
    height=HEIGHT,
    resolution=64,   # 원통 둘레 정밀도
    split=4
)

mesh.compute_vertex_normals()

# ===============================
# Move center to origin
# (Open3D cylinder: base at z=0 → center at z=H/2)
# ===============================
mesh.translate([0, 0, -HEIGHT / 2])

# ===============================
# Sample surface points
# ===============================
pcd_ref = mesh.sample_points_uniformly(
    number_of_points=N_POINTS
)

# ===============================
# Save reference point cloud
# ===============================
o3d.io.write_point_cloud(
    "cylinder_reference.ply",
    pcd_ref
)

# ===============================
# Visualization check
# ===============================


o3d.visualization.draw_geometries(
    [pcd_ref],
    window_name="Cylinder Reference (Z-axis height)"
)