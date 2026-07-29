# 06_control — 차량 제어

횡방향/종방향 차량 제어 알고리즘 패키지입니다.

## 패키지

| 패키지 | 설명 |
|--------|------|
| `control` | Stanley / Pure Pursuit 컨트롤러 |

## 빠른 시작

```bash
# Stanley (지역 경로 + 동적 속도 프로파일)
roslaunch control stanley_local_path.launch

# Stanley (전역 경로 + 고정 목표 속도)
roslaunch control stanley_global_path.launch
```
