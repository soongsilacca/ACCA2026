# safety_monitor 작업 안내

- 설계 결정, 보고서 대응, 알려진 한계는 README.md에 있다. 코드를 바꾸기 전에 읽는다.
- 한 패키지에 C++ 인지 노드(`src/perception/`)와 Python 판정 로직(`src/safety_monitor/`)이 같이 있다.
  메시지(`msg/`)를 쓰는 Python 코드는 `from safety_monitor.msg import ...`.
- 판정 로직은 `src/safety_monitor/`에 두고 ROS를 import하지 않는다. `nodes/`는 입출력만.
- 변경 후 `python3 -m pytest -q test` 실행. 새 판정 규칙에는 `test/test_monitor_scenarios.py`에 시나리오를 추가한다.
- Python 3.8 호환 유지 (Ubuntu 20.04): `X | None`, `list[int]` 표기, walrus 연산자 사용 금지.
- 보고서 6장과 다르게 구현한 부분은 README "설계 결정"에 적는다.
- 충돌 판정 입력은 인지 출력(`/rear_lidar/tracked_objects`)과 자차 상태, local route, MGeo만 쓴다. planner 내부 상태를 입력으로 쓰지 않는다.
- 자차 이동 정보 출처는 하나만 쓴다 (인지 노드는 출처가 바뀌면 트랙을 모두 지운다).
