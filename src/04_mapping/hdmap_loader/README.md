# hdmap_loader

MGeo 형식의 HD Map 데이터를 파싱하여 RViz에 시각화하는 패키지입니다.  
Link, Node, 차선 경계, 교차로 정보를 `visualization_msgs/MarkerArray` 로 발행합니다.

## 노드

| 노드 | 설명 |
|------|------|
| `mgeo_marker_node.py` | MGeo JSON 파싱 후 RViz 마커 발행 |

## Launch 파일

```bash
roslaunch hdmap_loader mgeo_marker.launch
```

## 토픽

| 토픽 | 타입 | 설명 |
|------|------|------|
| `/mgeo/link_marker` | `MarkerArray` | 도로 Link 시각화 |
| `/mgeo/node_marker` | `MarkerArray` | 교차로 Node 시각화 |

## 데이터

MGeo 데이터는 `scripts/` 디렉토리의 JSON 파일로 제공됩니다:

- `link_set.json` — 도로 링크 좌표 및 속성
- `node_set.json` — 교차로 노드 정보
- `global_info.json` — 좌표계 원점 (ENU Origin)
