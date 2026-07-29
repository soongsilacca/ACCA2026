# 🧹 LiDAR Filtering Package (`lidar_filtering`)

3D LiDAR Point Cloud 전처리, 지면 제거(Ground Filter), 도로 차선 인텐시티 필터링, 그리고 클러스터링 기반 3D 바운딩 박스 추정을 수행하는 패키지입니다.

> 💡 **라이선스 참고**: 본 패키지의 지면 제거 및 3D OBB 클러스터링 알고리즘은 **Autoware (Apache-2.0 License)** 오픈소스 알고리즘을 기반으로 작성되었습니다.

---

## 📄 구성 노드 (Implemented Nodes)

1. **`pointcloud_crop_node`**:
   - 관심 영역(ROI: x: 0~50m, y: -15~15m, z: -2~3m) 외부의 포인트 클라우드를 크롭 필터링.

2. **`ring_ground_filter_node`**:
   - Ring-Aware Ray Ground Filter 알고리즘을 사용하여 LiDAR 점군에서 지면(Ground)과 비지면(Filtered) 점군을 실시간으로 상호 분리.

3. **`pointcloud_lane_filter_node`**:
   - 지면 점군 중 반사도(Intensity) 필터를 적용하여 차선 표식 점군 추출.

4. **`shape_estimation_cluster_node`**:
   - 거리 감응형(Distance-Adaptive) 유클리드 클러스터링 및 L-Shape Fitting 기반 3D Oriented Bounding Box(OBB) 생성.

---

## 🚀 실행 방법 (Usage)

전체 라이다 필터링 및 클러스터링 파이프라인 동시 구동:

```bash
roslaunch lidar_filtering lidar_pipeline.launch
```

---

## 📡 주요 토픽 (Topics)

- **입력**: `/velodyne_points` (`sensor_msgs/PointCloud2`)
- **출력**:
  - `/velodyne_points_cropped` (ROI 크롭 점군)
  - `/velodyne_points_ground` (지면 점군)
  - `/velodyne_points_filtered` (비지면 장애물 점군)
  - `/clusters_markers` (`visualization_msgs/MarkerArray`: 3D 장애물 바운딩 박스 마커)
  - `/clusters_poses` (`geometry_msgs/PoseArray`: 장애물 중심 Pose)
