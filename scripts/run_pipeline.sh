#!/bin/bash
# 전체 파이프라인: LiDAR SLAM → TUM 궤적 → 이미지 DB 구축 → 평가
# 사용: scripts/run_pipeline.sh <bag파일> [config]
#
# 실행 주체는 원본 작업 레포(빌드/venv 보유):
#   /Users/deepfine/C++Project/Cpp_SLAM, /Users/deepfine/C++Project/VSLAM_repository
# 통합 레포의 서브모듈은 "검증된 커밋 기록"용이고 실행체가 아니다.
# 주의: 1단계(SLAM)는 Pangolin 뷰어가 뜨므로 데스크톱 세션에서 실행할 것.
set -e

BAG=${1:?bag 파일 경로 필요}
CFG=${2:-/Users/deepfine/C++Project/LIO_VisualReloc/configs/hilti2021.yaml}
SLAM_DIR=/Users/deepfine/C++Project/Cpp_SLAM
VSLAM_DIR=/Users/deepfine/C++Project/VSLAM_repository

echo "=== [1/3] LiDAR SLAM (Cpp_SLAM) ==="
( cd "$SLAM_DIR/build" && ./slam "$BAG" /hesai/pandar /alphasense/imu )
head -3 "$SLAM_DIR/output/slam_trajectory.tum"

echo "=== [2/3] 이미지 DB 구축 (SLAM 궤적 기반 3D 태깅) ==="
( cd "$VSLAM_DIR" && .venv/bin/python -m src.build_db "$CFG" )

echo "=== [3/3] held-out 평가 ==="
( cd "$VSLAM_DIR" && .venv/bin/python -m src.eval "$CFG" --limit 30 )

echo "완료 — DB: /Users/deepfine/C++Project/LIO_VisualReloc/output/db_hilti2021/"
