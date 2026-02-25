# Zivid-SAM3-object-tracking

Zivid 3D 카메라와 SAM3(Segment Anything Model 3)를 이용한 텍스트 프롬프트 기반 객체 분할 및 ICP 기반 3D 포즈 추정 프로젝트입니다. ROS2 환경에서 동작합니다.

## 개요

- **Zivid**: 2D/3D 캡처 및 `zivid_camera` ROS2 노드 연동
- **SAM3**: 텍스트 프롬프트로 객체 마스크 생성 → ROI 포인트클라우드 추출
- **ICP**: 측정 포인트클라우드와 레퍼런스 포인트클라우드 정합으로 3D 포즈(위치·자세) 계산
- **Viewer**: 마스크 시각화(OpenCV) + 3D 포인트클라우드/포즈 시각화(Open3D)

## 요구사항

- ROS2
- [Zivid Camera](https://www.zivid.com/) 및 `zivid_camera` ROS2 패키지
- Python 3, Open3D, OpenCV, PyTorch
- [SAM3](https://github.com/facebookresearch/segment-anything-3) (Segment Anything Model 3) 및 `sam3` Python 패키지

## 프로젝트 구조

| 파일 | 설명 |
|------|------|
| `zivid_capture.py` | Zivid 2D/3D 캡처 + SAM3 텍스트 프롬프트 분할 → ROI PLY 저장 (ROS2) |
| `zivid_capture_once.py` | 1회 캡처 전용 노드 (ROS2) |
| `zivid_sam3_mat.py` | Zivid 연동 + SAM3로 마스크/ROI 처리 (ROS2) |
| `sam3_zivid_viewer.py` | 마스크·ICP 결과·객체 포즈 시각화 (OpenCV + Open3D, ROS2) |
| `icp based 3D pose.py` | 측정 PLY vs 레퍼런스 PLY ICP 정합 → 3D 포즈 계산 (스크립트) |
| `reference point cloud.py` | 레퍼런스 포인트클라우드 생성 (실린더 등 메쉬 → PLY) |
| `ply_check.py` | PLY 파일 확인용 유틸 |
| `start` | 실행 순서 정리 (테스트/세분화 태스크) |

## 실행 방법

### 1. Zivid ROS2 연동 (카메라 IP: 172.28.60.5)

```bash
conda deactivate
ros2 run zivid_camera zivid_camera
```

### 2. SAM3 + Zivid 연동 (촬영 데이터 → SAM3 처리)

```bash
conda activate zivid_sam3
cd ros2_zivid_sam3
ros2 run sam3_subscriber sam3_zivid
```

### 3. 객체 검출 명령 (촬영 트리거 + SAM3로 전송)

```bash
conda deactivate
cd ros2_zivid
python3 zivid_capture.py --ros-args -p input_prompt:="silver metal rod"
# 예: python3 zivid_capture.py --ros-args -p input_prompt:="silver nuts"
```

### 4. 시각화 (선택)

```bash
conda activate zivid_sam3
cd ros2_zivid
python3 sam3_zivid_viewer.py
```

객체 검출을 위해 **1, 2, 3**은 필수 실행, **4**는 시각화용입니다.

## 레퍼런스·측정 데이터

- 레퍼런스 PLY: `reference point cloud.py`로 생성 후 `capture/reference.ply` 등으로 저장
- 측정 ROI PLY: `zivid_capture.py`(또는 연동 파이프라인) 결과로 `capture/measured_ROI.ply` 등 사용
- `icp based 3D pose.py`에서 위 경로를 수정해 ICP 및 3D 포즈 계산 가능

## 라이선스

저장소 기본 라이선스를 따릅니다.
