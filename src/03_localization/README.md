# 03_localization — 위치 추정

GPS, IMU, Wheel Odometry, LiDAR NDT를 융합한 자차 위치 추정 스택입니다.

## 패키지

| 패키지 | 설명 | 출처 |
|--------|------|------|
| `localization` | EKF 기반 GPS/Odom 융합 로컬라이제이션 | 자체 개발 |
| `hdl_localization` | HDL Graph SLAM NDT 맵 매칭 | GitHub (koide3) |
| `ndt_omp` | OpenMP 병렬 NDT 라이브러리 | GitHub |
| `fast_gicp` | Fast GICP ICP 라이브러리 | GitHub |

## 빠른 시작

```bash
# EKF 글로벌 Localization (GPS + Wheel Odom)
roslaunch localization localization_global.launch

# NDT 맵 매칭 Localization
roslaunch hdl_localization hdl_localization.launch
```
