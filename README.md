# Zivid-SAM3-object-tracking

Zivid 3D 카메라와 SAM3(Segment Anything Model 3)를 ROS2로 연동하여, 2D/3D 촬영 → 마스크 추정 → 포인트 클라우드 ROI 추출 → ICP 기반 레퍼런스 추적까지 수행하고, **카메라 기준 객체 위치(pose)**를 ROS 토픽으로 퍼블리시하는 코드 모음입니다.

## 개요

1. **Zivid**로 2D RGB 이미지와 3D 포인트 클라우드(PLY) 촬영  
2. **SAM3**로 2D 이미지에 텍스트 프롬프트 기반 마스크 추정  
3. 마스크와 3D 포인트 클라우드를 **병합**하여 객체 ROI 포인트 클라우드 생성  
4. **ICP** 알고리즘으로 레퍼런스 포인트 클라우드와 정합 → **카메라 좌표계 기준 객체 pose** 산출  
5. 위 과정에서 생성된 **마스크 시각화, pose, ICP 결과** 등을 **ROS2 토픽으로 퍼블리시**

## 폴더 구조

| 파일 | 설명 |
|------|------|
| `zivid_capture.py` | Zivid 2D/3D 촬영 후 `/sam3_zivid/rgb`, `/sam3_zivid/meta` 퍼블리시 (SAM3 파이프라인 트리거용) |
| `zivid_capture_once.py` | Zivid 1회 촬영 (2D/3D) 스크립트 |
| `zivid_sam3_mat.py` | Zivid + SAM3 단일 실행: 촬영 → SAM3 마스크 → 2D/3D 병합 → ROI PLY 저장 (ROS 서비스/이미지 구독) |
| `reference point cloud.py` | 레퍼런스용 포인트 클라우드 생성 (실린더/박스 등 메쉬 → PLY) |
| `icp based 3D pose.py` | 측정 PLY vs 레퍼런스 PLY ICP 정합 → 카메라 기준 위치/자세(roll-pitch-yaw) 계산 (스탠드얼론) |
| `sam3_zivid_viewer.py` | `/sam3_zivid/mask_vis`, `/sam3_zivid/object_pose`, `/sam3_zivid/icp_ply` 구독하여 2D 마스크 + pose 텍스트 + 3D 뷰어 시각화 |
| `ply_check.py` | PLY 파일 로드/시각화 유틸 |
| `start` | 실행 순서 메모 (Zivid → SAM3 노드 → 캡처 → 뷰어) |

## 요구 사항

- **ROS2** (Humble 권장)
- **Zivid ROS2 드라이버** (`zivid_camera`), Zivid 카메라 연결
- **Python 3.8+**
- **Conda** (선택): SAM3용 환경 예) `zivid_sam3`

### Python 패키지

- `rclpy`, `sensor_msgs`, `std_msgs`, `geometry_msgs`, `std_srvs`, `cv_bridge`
- `zivid_interfaces` (Zivid ROS2 패키지)
- `opencv-python`, `numpy`, `open3d`
- `torch`, `PIL`
- **SAM3** (`sam3`): [Segment Anything 3](https://github.com/facebookresearch/segment-anything-3) 이미지 모델 빌드/프로세서 사용

## 사용법

### 1. Zivid ROS2 연동

카메라 IP 등 설정 후 Zivid 노드 실행:

```bash
# 필요 시 conda 비활성화 (시스템 Python/ROS 사용)
conda deactivate
ros2 run zivid_camera zivid_camera
```

### 2. SAM3 + Zivid 파이프라인 노드 실행

SAM3가 적용된 노드(예: `sam3_subscriber` / `sam3_zivid`)를 먼저 띄웁니다.  
이 노드는 `/sam3_zivid/rgb`, `/sam3_zivid/meta`를 구독하고, 마스크·ICP·pose를 계산한 뒤 아래 토픽으로 퍼블리시합니다.

```bash
conda activate zivid_sam3   # SAM3 환경
cd <your_ros_workspace>/ros2_zivid_sam3
ros2 run sam3_subscriber sam3_zivid
```

### 3. 촬영 트리거 (객체 검출 명령)

Zivid로 한 번 촬영하고, 촬영된 2D/3D를 SAM3 파이프라인으로 보냅니다.

```bash
conda deactivate
cd /path/to/zivid_sam3_coor_track
python3 zivid_capture.py --ros-args -p input_prompt:="silver metal rod"
```

`input_prompt`는 SAM3 텍스트 프롬프트(검출할 객체 설명)입니다.  
촬영 후 PLY 경로 등이 `/sam3_zivid/meta`로 전달됩니다. 저장 경로는 실행 디렉터리 기준 `./capture` 이며, `-p save_dir:=/원하는/경로` 로 변경 가능합니다.

### 4. 시각화 (뷰어)

마스크 오버레이, 카메라 기준 pose, ICP 정합 결과를 보려면:

```bash
conda activate zivid_sam3
cd /path/to/zivid_sam3_coor_track
python3 sam3_zivid_viewer.py
```

### 5. 레퍼런스 포인트 클라우드 생성

객체 형상에 맞는 레퍼런스 PLY가 필요할 때:

```bash
python3 reference\ point\ cloud.py
```

생성된 `cylinder_reference.ply`(또는 설정한 파일명)를 ICP에서 사용합니다.

### 6. ICP만 따로 실행 (스크립트)

측정 ROI PLY와 레퍼런스 PLY 경로를 스크립트 상단 `measured_ply`, `reference_ply`(또는 `_capture_dir`)에서 수정한 뒤:

```bash
python3 icp\ based\ 3D\ pose.py
```

카메라 기준 위치(mm) 및 Roll-Pitch-Yaw(deg)가 터미널에 출력됩니다. 기본 경로는 실행 디렉터리 기준 `./capture/measured_ROI.ply`, `./capture/reference.ply` 입니다.

### 7. Zivid + SAM3 한 번에 실행 (단일 노드)

`zivid_sam3_mat.py`는 Zivid 서비스/이미지 토픽에 직접 연결해, 촬영 → SAM3 마스크 → 2D/3D 병합 → ROI PLY 저장까지 한 번에 수행합니다.  
`input_prompt`와 저장 경로는 스크립트 내 `save_dir`(기본 `./capture`)에서 수정 후:

```bash
python3 zivid_sam3_mat.py
```

## ROS2 토픽 요약

| 토픽 | 메시지 타입 | 방향 | 설명 |
|------|-------------|------|------|
| `/color/image_color` | `sensor_msgs/Image` | Zivid → 구독 | Zivid 2D RGB |
| `/capture_2d` | `std_srvs/Trigger` | 클라이언트 호출 | 2D 촬영 트리거 |
| `/capture_and_save` | `zivid_interfaces/CaptureAndSave` | 클라이언트 호출 | 3D 촬영 및 PLY 저장 |
| `/zivid_camera/set_parameters` | `rcl_interfaces/SetParameters` | 클라이언트 호출 | 2D/3D 설정 |
| `/sam3_zivid/rgb` | `sensor_msgs/Image` | 퍼블리시 | 촬영된 RGB (SAM3 입력용) |
| `/sam3_zivid/meta` | `std_msgs/String` | 퍼블리시 | prompt, ply_path, stamp 등 JSON |
| `/sam3_zivid/mask_vis` | `sensor_msgs/Image` | 퍼블리시 | SAM3 마스크 시각화 이미지 |
| `/sam3_zivid/object_pose` | `geometry_msgs/PoseStamped` | 퍼블리시 | 카메라 기준 객체 pose (미터) |
| `/sam3_zivid/icp_ply` | `std_msgs/String` | 퍼블리시 | ICP용 meas_ply, ref_ply 경로 JSON |

## 설정 변경

- **촬영 저장 경로**: `zivid_capture.py`의 `save_dir` 파라미터, `zivid_sam3_mat.py`의 `save_dir` 변수 (기본: 실행 디렉터리 `./capture`)
- **SAM3 프롬프트**: `zivid_capture.py`의 `input_prompt` 파라미터, `zivid_sam3_mat.py`의 `input_prompt` 변수
- **레퍼런스 형상**: `reference point cloud.py`에서 실린더 반경/높이 등 수정
- **ICP 경로**: `icp based 3D pose.py` 상단 `_capture_dir`, `measured_ply`, `reference_ply` 경로

## 라이선스 및 참고

- Zivid: [Zivid](https://www.zivid.com/) 드라이버 및 ROS2 패키지 사용 규정 따름  
- SAM3: [Segment Anything 3](https://github.com/facebookresearch/segment-anything-3) 라이선스 따름  
- Open3D: [Open3D](http://www.open3d.org/) 라이선스 따름  

이 리포지토리는 위 도구들을 연동한 스크립트 모음이며, 각 의존 패키지의 라이선스를 준수해 사용하세요.

---

## GitHub에 올리는 방법

### 1. Git 초기화 (아직 안 했다면)

```bash
cd /path/to/zivid_sam3_coor_track
git init
```

### 2. 파일 추가 및 첫 커밋

```bash
git add .
git status   # .gitignore에 의해 .ply, __pycache__ 등은 제외됨
git commit -m "Initial commit: Zivid + SAM3 coordinate tracking"
```

### 3. GitHub에서 새 저장소 만들기

1. [GitHub](https://github.com) 로그인 후 **New repository** 클릭  
2. Repository name: 예) `zivid_sam3_coor_track`  
3. Public/Private 선택, **Create repository** (README나 .gitignore 추가하지 말고 비워둠)  
4. 생성된 페이지에 나오는 저장소 URL 복사 (예: `https://github.com/사용자명/zivid_sam3_coor_track.git`)

### 4. 원격 저장소 연결 및 푸시

```bash
git remote add origin https://github.com/사용자명/zivid_sam3_coor_track.git
git branch -M main
git push -u origin main
```

이미 `origin`이 있으면 URL만 변경:

```bash
git remote set-url origin https://github.com/사용자명/zivid_sam3_coor_track.git
git push -u origin main
```

### 참고

- **SSH 사용 시**: URL을 `git@github.com:사용자명/저장소명.git` 로 바꾸면 됩니다.  
- **비공개 저장소**: GitHub에서 Private로 만들면 위와 동일한 절차로 푸시 가능합니다.
