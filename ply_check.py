import open3d as o3d
import numpy as np
import sys
import os

# PLY 파일 경로: 인자로 전달하거나 아래 기본값 사용
if len(sys.argv) > 1:
    ply_file = sys.argv[1]
else:
    ply_file = os.path.join(os.getcwd(), "capture", "pointcloud.ply")
 
# 1️⃣ PLY 파일 읽기
pcd = o3d.io.read_point_cloud(ply_file)

# 2️⃣ 포인트 좌표
points = np.asarray(pcd.points)
print(f"Points shape: {points.shape}")  # (N, 3)
print("First 5 points:\n", points[0:5])

# 3️⃣ RGB 컬러 정보
if pcd.has_colors():
    colors = np.asarray(pcd.colors)
    print(f"Colors shape: {colors.shape}")  # (N, 3)
    print("First 5 colors:\n", colors[:5])

# 4️⃣ 시각화
o3d.visualization.draw_geometries([pcd])
