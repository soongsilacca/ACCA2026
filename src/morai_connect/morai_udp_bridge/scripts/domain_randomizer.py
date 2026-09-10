#!/usr/bin/env python3
"""
MORAI Scenario Domain Randomizer Script & ROS Node
==================================================
사용자가 정의한 구역별 규칙에 맞추어 도메인 랜덤화(Domain Randomization)를 수행합니다.

수정사항:
1. 정적 장애물: 전체 MORAI 물건/더미차량/차단벽 오브젝트(ALL_STATIC_OBSTACLE_DATA_IDS) 중 1개 전면 무작위 선택
2. 동적 장애물: 시작점 (X1, Y1)과 끝점 (X2, Y2)을 연속 좌표계 공간 내에서 완전히 무작위 생성하고,
   이동 방향 헤딩(yaw = atan2(dy, dx))과 waypointDataList(PedPath)를 매번 다르게 동적 반영
"""

import csv
import json
import random
import math
import os
import sys
import argparse
from collections import defaultdict, deque
from copy import deepcopy

# 전체 MORAI 정적 물건 / 라바콘 / 드럼통 / 차단벽 / 더미차량 DataID 전체 리스트
ALL_STATIC_OBSTACLE_DATA_IDS = [
    40100001, # CargoBox (대형 화물상자)
    40100002, # WoodBox (목재 상자)
    40100003, # Box (상자)
    40100004, # Barrel (드럼통)
    40100005, # Traffic_Cone (라바콘)
    40100006, # Bollard (볼라드)
    40100007, # Construction_Cone (공사 콘)
    40100008, # OBJ_Hyundai_Grandeur
    40100009, # OBJ_Hyundai_Genesis
    40100010, # OBJ_Hyundai_Avante
    40100011, # OBJ_Hyundai_Ioniq
    40100012, # OBJ_Hyundai_Tucson
    40100013, # OBJ_Hyundai_Sonata
    40100014, # OBJ_Kia_Morning
    40100015, # OBJ_Kia_Ray
    40100016, # OBJ_Kia_Carnival
    40100017, # OBJ_Kia_Staria
    40100018, # OBJ_Kia_K5
    40100019, # OBJ_Kia_Sportage
    40100020, # OBJ_Kia_Niro
    40100027, # PE_Drum (안전 PE 드럼)
    40100028, # PE_Firewall_Orange (방음/차단벽 주황)
    40100029, # PE_Firewall_White (방음/차단벽 하양)
    40100033, # Steel_Barricade (스틸 바리케이트)
    40100047, # Concrete_Barricade01 (콘크리트 방호벽)
    40100049, # PE_Barricade01 (PE 바리케이트)
    40100055, # Sign/Structure
    40100059, # Fence/Barricade
    40100065, # Debris/Box
    40100067, # Safety Signal
    40100086, # NCAP_GVT_BIG
    40100092  # obj_emptybox
]

# Assets with predictable, road-obstacle-sized native dimensions. Large
# walls/fences/sign structures are deliberately excluded: choosing all object
# IDs with one common scale produced implausibly huge obstacles.
COMPACT_STATIC_OBSTACLE_DATA_IDS = [
    40100002,  # WoodBox
    40100003,  # Box
    40100004,  # Barrel
    40100005,  # Traffic cone
    40100006,  # Bollard
    40100007,  # Construction cone
    40100008,  # Parked Grandeur
    40100009,  # Parked Genesis
    40100010,  # Parked Avante
    40100011,  # Parked Ioniq
    40100012,  # Parked Tucson
    40100013,  # Parked Sonata
    40100014,  # Parked Morning
    40100015,  # Parked Ray
    40100016,  # Parked Carnival
    40100017,  # Parked Staria
    40100018,  # Parked K5
    40100019,  # Parked Sportage
    40100020,  # Parked Niro
    40100027,  # PE drum
    40100033,  # Steel barricade
    40100047,  # Concrete barricade
    40100049,  # PE barricade
    40100065,  # Debris / box
    40100067,  # Safety signal
    40100086,  # NCAP vehicle target
    40100092,  # Empty box
]

# Randomly placed obstacles should not include vehicles to prevent road blocking issues.
NON_VEHICLE_OBSTACLE_DATA_IDS = [
    40100002, 40100003, 40100004, 40100005, 40100006, 40100007,
    40100027, 40100033, 40100047, 40100049, 40100065, 40100067, 40100092
]

# Curated, non-intersection points on the authored global route. z is sampled
# from MGeo link_set.json rather than guessed from the shaded-area mesh.
STATIC_ROUTE_ZONES = [
    (-69.943, -209.168, 28.410, 70.2),
    (-60.947,   71.507, 28.370, 89.5),
    (-99.304,  260.316, 28.363, 90.5),
    ( 62.292,  242.997, 28.268, -89.5),
    ( 67.552,  -65.373, 28.353, -89.3),
    ( 70.238, -372.248, 28.235, -89.7),
]

# Route-centre crossing candidates: x, y, road z, route heading(deg).
# Pedestrians walk perpendicular to the route and activate only when the ego
# is near, which produces useful moving-obstacle interactions across episodes.
DYNAMIC_CROSSING_ZONES = [
    (-59.900,  -79.531, 28.384,   90.4),
    (-19.129,  168.656, 28.438,  134.1),
    (-16.948,  345.571, 28.342,    1.7),
    ( 65.785,  110.582, 28.271,  -86.7),
    ( 73.068, -196.645, 28.371,  -83.9),
    ( -9.606, -509.714, 28.436, -151.5),
]

# 승용차, SUV, 경차, 승합차 NPC 차량 리스트 (대형 버스/트럭 제외)
PASSENGER_NPC_VEHICLE_DATA_IDS = [
    30100004, # 쏘나타 / K5 세단
    30100005, # 그랜저 / 제네시스 세단
    30100010, # 아이오닉 세단
    30100014, # 아반떼 세단
    30200003, # 니로 / 투싼 SUV
    30300002, # 모닝 / 레이 경차
    30400004  # 카니발 / 스타리아 승합차
]

# Only models verified in the installed MORAI asset set. Inventing contiguous
# DataID ranges leaves an actor in JSON but MORAI silently skips its creation.
PEDESTRIAN_DATA_IDS = [
    50100001, 50100002, 50100003, 50100004, 50100005,
    50200001, 50200002,
]

def apply_zone_rules(scenario):
    new_scen = deepcopy(scenario)

    # -------------------------------------------------------------
    # 0. Ego 자율주행 차량 설정 (위치: sample_scene / 기어&모드: sample.json 키보드 P기어)
    # -------------------------------------------------------------
    if 'egoVehicle' in new_scen:
        ego = new_scen['egoVehicle']
        ego['initPosition']['pos'] = {
            "x": -131.48599243164062,
            "y": -427.96099853515625,
            "z": 28.882999420166016,
            "_x": "-131.486",
            "_y": "-427.961",
            "_z": "28.883"
        }
        ego['initPosition']['rot'] = {
            "roll": "-359.540",
            "pitch": "0.121",
            "yaw": "62.515"
        }
        ego['initPosition']['gear'] = 1
        ego['beforeControlMode'] = 3  # 키보드 수동 제어 모드
        ego['currentControlMode'] = 1

    # -------------------------------------------------------------
    # 1. 원본 고정 정적 장애물
    #    위치/DataID/UNIQUEID/scale은 기준 시나리오 값을 변경하지 않는다.
    #    원본의 비활성 spawn 필드만 실제 고정 위치에 맞춰 정규화한다.
    # -------------------------------------------------------------
    if 'objectList' in new_scen and len(new_scen['objectList']) > 0:
        obj = new_scen['objectList'][0]
        obj['initPos'] = deepcopy(obj['pos'])
        obj['initRot'] = deepcopy(obj.get('rot', {
            "roll": "0.000", "pitch": "0.000", "yaw": "0.000"}))
        obj['isWaypointMode'] = False
        obj['moveSpeed'] = 0.0
        obj['activeDistance'] = 999.0

    # -------------------------------------------------------------
    # 2. 동적 장애물 구역 (Dynamic Zone)
    #    보도 좌표 기반 도로 횡단:
    #    side      (우측 보도): X ≈ -53,   Y ∈ [-36, -4]
    #    other side(좌측 보도): X ≈ -73.5, Y ∈ [-36, -4]
    #    -> 무작위로 출발 방향 결정 후 같은 Y 위치에서 반대편으로 횡단
    # -------------------------------------------------------------
    # 출발 보도 무작위 결정 (True = side→other_side, False = other_side→side)
    crossing_dir = random.choice([True, False])

    # 횡단 Y 지점: 두 보도가 공유하는 Y 범위 [-36, -4] 내 무작위 선택
    cross_y = random.uniform(-35.5, -4.5)

    # side 보도 X: -52.5 ~ -53.9 중간값 ≈ -53.2
    # other side 보도 X: -73.3 ~ -74.2 중간값 ≈ -73.6
    SIDE_X      = round(random.uniform(-53.9, -52.5), 3)   # 우측 보도
    OTHER_X     = round(random.uniform(-74.2, -73.3), 3)   # 좌측 보도

    z_val = 28.5
    if crossing_dir:
        # side -> other side (서쪽 방향 횡단, yaw ≈ 180°)
        x1, y1 = SIDE_X,  cross_y
        x2, y2 = OTHER_X, cross_y
    else:
        # other side -> side (동쪽 방향 횡단, yaw ≈ 0°)
        x1, y1 = OTHER_X, cross_y
        x2, y2 = SIDE_X,  cross_y

    # float과 string(_x/_y/_z)을 완벽히 동기화
    rx1 = round(x1, 3);  ry1 = round(y1, 3)
    rx2 = round(x2, 3);  ry2 = round(y2, 3)
    rz  = round(z_val, 3)

    # pos: 세계 ENU 좌표 (MORAI가 실제 스폰 위치로 사용)
    p1_world = {"x": rx1, "y": ry1, "z": rz,
                "_x": f"{rx1:.3f}", "_y": f"{ry1:.3f}", "_z": f"{rz:.3f}"}
    p2_world = {"x": rx2, "y": ry2, "z": rz,
                "_x": f"{rx2:.3f}", "_y": f"{ry2:.3f}", "_z": f"{rz:.3f}"}

    # waypointDataList / initPos / standardPos 는 Unity 로컬 좌표계 사용
    # 변환 공식 (원본 샘플 실측):
    #   waypoint_x = -world_x
    #   waypoint_y =  world_z
    #   waypoint_z = -world_y
    def to_wp(wx, wy, wz):
        wpx = round(-wx, 3);  wpy = round(wz, 3);  wpz = round(-wy, 3)
        return {"x": wpx, "y": wpy, "z": wpz,
                "_x": f"{wpx:.3f}", "_y": f"{wpy:.3f}", "_z": f"{wpz:.3f}"}

    p1_wp = to_wp(rx1, ry1, rz)
    p2_wp = to_wp(rx2, ry2, rz)

    # 이동 방향 헤딩 각도 계산 (yaw) - world 좌표 기준
    yaw_deg = math.degrees(math.atan2(y2 - y1, x2 - x1))

    chosen_ped_id = random.choice(PEDESTRIAN_DATA_IDS)
    if 'pedestrianList' in new_scen and len(new_scen['pedestrianList']) > 0:
        ped = new_scen['pedestrianList'][0]
        ped_uid = ped.get('UNIQUEID', 3)
        ped['DataID'] = chosen_ped_id
        ped['pos'] = p1_world          # 세계 ENU: 스폰 위치
        ped['initPos'] = p1_wp         # Unity 좌표계: waypoint[0]와 일치해야 이동 시작
        ped['standardPos'] = p1_wp     # Unity 좌표계: 기준 위치
        ped['rot']['yaw'] = f"{yaw_deg:.3f}"
        ped['speed'] = round(random.uniform(7.0, 12.0), 1)
        ped['activeDistance'] = 50.0
        ped['loop'] = True
        ped['isWaypointMode'] = True

        # waypointDataList에서 해당 보행자 경로 갱신 (Unity 좌표계로 설정)
        ped_path_found = False
        for wp_data in new_scen.get('waypointDataList', []):
            if f"PedPath - {ped_uid}" in wp_data.get('pathName', '') or "PedPath" in wp_data.get('pathName', ''):
                wp_data['waypointData'] = [
                    {"pos": p1_wp, "rot": {"roll": "0.000", "pitch": "0.000", "yaw": f"{yaw_deg:.3f}"}},
                    {"pos": p2_wp, "rot": {"roll": "0.000", "pitch": "0.000", "yaw": f"{yaw_deg:.3f}"}}
                ]
                ped_path_found = True
                break

        if not ped_path_found:
            new_scen.setdefault('waypointDataList', []).append({
                "pathName": f"PedPath - {ped_uid}",
                "waypointData": [
                    {"pos": p1_wp, "rot": {"roll": "0.000", "pitch": "0.000", "yaw": f"{yaw_deg:.3f}"}},
                    {"pos": p2_wp, "rot": {"roll": "0.000", "pitch": "0.000", "yaw": f"{yaw_deg:.3f}"}}
                ]
            })


    # -------------------------------------------------------------
    # 3. 회전교차로 구역 (Roundabout Zone: X[-132.4, -77.1], Y[307.4, 370.6])
    # -------------------------------------------------------------
    for veh in new_scen.get('vehicleList', []):
        if -140 <= veh['initPosition']['pos']['x'] <= -70 and 300 <= veh['initPosition']['pos']['y'] <= 380:
            veh['DataID'] = random.choice(PASSENGER_NPC_VEHICLE_DATA_IDS)

    for sp in new_scen.get('spawnPointList', []):
        if -140 <= sp['pos']['x'] <= -70 and 300 <= sp['pos']['y'] <= 380:
            sp['spawnVehicleTypeID'] = random.choice([-1] + PASSENGER_NPC_VEHICLE_DATA_IDS)
            sp['maximumSpawnVehicle'] = 999
            sp['minSpawnPeriod'] = round(random.uniform(4.0, 8.0), 1)
            sp['maxSpawnPeriod'] = round(random.uniform(4.0, 8.0), 1)

    # -------------------------------------------------------------
    # 4. 고속도로 구역 (Highway Zone: X[59.6, 97.8], Y[-379.1, 333.5])
    # -------------------------------------------------------------
    for veh in new_scen.get('vehicleList', []):
        if 50 <= veh['initPosition']['pos']['x'] <= 105 and -390 <= veh['initPosition']['pos']['y'] <= 350:
            veh['DataID'] = random.choice(PASSENGER_NPC_VEHICLE_DATA_IDS)
            veh['desiredVelocity'] = round(random.uniform(40.0, 90.0), 1)
            veh['bias'] = round(random.uniform(-0.4, 0.4), 2)

    for sp in new_scen.get('spawnPointList', []):
        if 50 <= sp['pos']['x'] <= 105 and -390 <= sp['pos']['y'] <= 350:
            sp['spawnVehicleTypeID'] = random.choice([-1] + PASSENGER_NPC_VEHICLE_DATA_IDS)
            period = round(random.uniform(3.0, 10.0), 1)
            sp['minSpawnPeriod'] = period
            sp['maxSpawnPeriod'] = period
            sp['MinDesiredVelocity_Custom'] = round(random.uniform(50.0, 80.0), 1)
            sp['MaxDesiredVelocity_Custom'] = round(random.uniform(80.0, 100.0), 1)

    # -------------------------------------------------------------
    # 5. 음영 구역 (Shaded Area Zone: X[-76.480, 30.735], Y[-550.001, -487.910])
    # -------------------------------------------------------------
    if 'shadedAreaList' in new_scen and len(new_scen['shadedAreaList']) > 0:
        sa = new_scen['shadedAreaList'][0]
        sa['pos'] = {"x": -22.873, "y": -518.956, "z": 30.363, "_x": "-22.873", "_y": "-518.956", "_z": "30.363"}
        sa['size'] = {"x": 107.215, "y": 62.091, "z": 12.248, "_x": "107.215", "_y": "62.091", "_z": "12.248"}

    shaded_mode = random.choice(['static'])

    if shaded_mode == 'static':
        sx = random.uniform(-40.0, -10.0)
        sy = round(-518.0 + (sx - (-25.0)) * 0.53, 3)
        sz = 30.363
        new_scen.setdefault('objectList', []).append({
            "DataID": random.choice(ALL_STATIC_OBSTACLE_DATA_IDS),
            "UNIQUEID": 101,
            "m_eobstacleObjType": 4,
            "pos": {"x": sx, "y": sy, "z": sz, "_x": f"{sx:.3f}", "_y": f"{sy:.3f}", "_z": f"{sz:.3f}"},
            "rot": {"roll": "0.000", "pitch": "0.000", "yaw": f"{random.uniform(-180, 180):.3f}"},
            "scale": {"x": 1.0, "y": 1.0, "z": 1.0, "_x": "1.000", "_y": "1.000", "_z": "1.000"},
            "initPos": {"x": sx, "y": sy, "z": sz, "_x": f"{sx:.3f}", "_y": f"{sy:.3f}", "_z": f"{sz:.3f}"},
            "initRot": {"roll": "0.000", "pitch": "0.000", "yaw": "0.000"},
            "isWaypointMode": False,
            "moveSpeed": 0.0,
            "activeDistance": 999.0,
            "waypointEventList": []
        })
    elif shaded_mode == 'dynamic':
        sx = random.uniform(-30.0, -10.0)
        sy = round(-518.0 + (sx - (-25.0)) * 0.53, 3)
        sz = 30.363
        p1 = {"x": sx, "y": sy - 5.0, "z": sz, "_x": f"{sx:.3f}", "_y": f"{sy - 5.0:.3f}", "_z": f"{sz:.3f}"}
        p2 = {"x": sx, "y": sy + 5.0, "z": sz, "_x": f"{sx:.3f}", "_y": f"{sy + 5.0:.3f}", "_z": f"{sz:.3f}"}

        ped_uid = 102
        new_scen.setdefault('pedestrianList', []).append({
            "DataID": random.choice(PEDESTRIAN_DATA_IDS),
            "UNIQUEID": ped_uid,
            "m_eobstacleObjType": 2,
            "pos": p1,
            "rot": {"roll": "0.000", "pitch": "0.000", "yaw": "90.000"},
            "activeDistance": 999.0,
            "speed": 3.0,
            "active": True,
            "loop": True,
            "initPos": p1,
            "initRot": {"roll": "0.000", "pitch": "0.000", "yaw": "0.000"},
            "movingDistance": 10.0,
            "movingDistanceAmount": 0.0,
            "standardPos": p1,
            "isWaypointMode": True,
            "waypointEventList": []
        })
        new_scen.setdefault('waypointDataList', []).append({
            "pathName": f"PedPath - {ped_uid}",
            "waypointData": [
                {"pos": p1, "rot": {"roll": "0.000", "pitch": "0.000", "yaw": "0.000"}},
                {"pos": p2, "rot": {"roll": "0.000", "pitch": "0.000", "yaw": "0.000"}}
            ]
        })
    elif shaded_mode == 'npc_vehicle':
        new_scen.setdefault('vehicleList', []).append({
            "DataID": random.choice(PASSENGER_NPC_VEHICLE_DATA_IDS),
            "UNIQUEID": 103,
            "m_eobstacleObjType": 1,
            "initPosition": {
                "initPositionMode": "Absolute",
                "pos": {"x": 29.464, "y": -487.889, "z": 30.363, "_x": "29.464", "_y": "-487.889", "_z": "30.363"},
                "rot": {"roll": "0.000", "pitch": "0.000", "yaw": "-148.600"},
                "initLink": "A2256W000001", "initLinkRatio": 0, "gear": 1
            },
            "velocityType": 2,
            "desiredVelocity": 20.0,
            "currentVelocity": 20.0,
            "isCloseLoop": False,
            "isLaneChange": False,
            "spawnPointUniqueID": 104,
            "destinationMode": 1,
            "activeDistance": 999.0
        })

    return new_scen

def _route_zones(global_path_csv, fractions):
    """Sample obstacle centres from the active global path and MGeo height."""
    with open(global_path_csv, newline='') as stream:
        rows = list(csv.DictReader(stream))
    if len(rows) < 10:
        raise RuntimeError("global path is too short: %s" % global_path_csv)

    link_file = os.path.join(os.path.dirname(os.path.dirname(
        os.path.dirname(os.path.dirname(os.path.abspath(__file__))))),
        'map_viz', 'scripts', 'link_set.json')
    with open(link_file, encoding='utf-8') as stream:
        links = json.load(stream)
    mgeo_points = [point for link in links for point in link.get('points', [])]

    zones = []
    for low, high in fractions:
        fraction = random.uniform(low, high)
        row = rows[int(fraction * (len(rows) - 1))]
        x, y = float(row['x']), float(row['y'])
        yaw = float(row.get('yaw', 0.0))
        if abs(yaw) <= 2.0 * math.pi + 0.1:
            yaw = math.degrees(yaw)
        nearest = min(mgeo_points,
                      key=lambda point: (point[0] - x) ** 2 +
                                        (point[1] - y) ** 2)
        zones.append((x, y, float(nearest[2]), yaw))
    return zones


def _remove_static_blocking_npc_routes(scenario, static_objects,
                                       clearance_m=3.0):
    """Keep all NPC traffic and remove static objects intersecting its routes."""
    link_file = os.path.join(os.path.dirname(os.path.dirname(
        os.path.dirname(os.path.dirname(os.path.abspath(__file__))))),
        'map_viz', 'scripts', 'link_set.json')
    with open(link_file, encoding='utf-8') as stream:
        links = json.load(stream)
    by_id = {link['idx']: link for link in links}
    outgoing = defaultdict(list)
    for link in links:
        outgoing[link.get('from_node_idx')].append(link['idx'])

    def shortest_link_path(start_id, end_id):
        if start_id not in by_id or end_id not in by_id:
            return []
        queue = deque([start_id])
        parent = {start_id: None}
        while queue:
            current = queue.popleft()
            if current == end_id:
                break
            node = by_id[current].get('to_node_idx')
            for following in outgoing.get(node, []):
                if following not in parent:
                    parent[following] = current
                    queue.append(following)
        if end_id not in parent:
            return []
        path = []
        current = end_id
        while current is not None:
            path.append(current)
            current = parent[current]
        return list(reversed(path))

    npc_route_points = []
    for spawn in scenario.get('spawnPointList', []):
        start = spawn.get('startLinkInfo', {}).get('linkIdx')
        end = spawn.get('endLinkInfo', {}).get('linkIdx')
        route_ids = shortest_link_path(start, end)
        npc_route_points.extend(
            point for link_id in route_ids
            for point in by_id[link_id].get('points', []))

    clearance_sq = clearance_m ** 2
    removed_ids = []
    for obj in static_objects:
        pos = obj.get('pos')
        if not pos:
            continue
        ox, oy = float(pos['x']), float(pos['y'])
        blocked = any(
            (float(point[0]) - ox) ** 2 + (float(point[1]) - oy) ** 2
            <= clearance_sq for point in npc_route_points)
        if blocked:
            removed_ids.append(int(obj.get('UNIQUEID', -1)))
    if removed_ids:
        removed_id_set = set(removed_ids)
        # Filter only rejected additions from the complete list. Replacing the
        # list with ``retained`` here used to discard every authored object.
        scenario['objectList'] = [
            obj for obj in scenario.get('objectList', [])
            if int(obj.get('UNIQUEID', -1)) not in removed_id_set]
        removed_names = {"ObstaclePath - %d" % uid for uid in removed_ids}
        scenario['waypointDataList'] = [
            path for path in scenario.get('waypointDataList', [])
            if path.get('pathName') not in removed_names]
    return removed_ids


def apply_obstacle_only_rules(scenario, global_path_csv):
    """Preserve every reference actor and append randomized obstacles.

    The competition sample already contains validated ego, NPC, pedestrian,
    static obstacle, spawn-point, traffic-light and waypoint settings. None of
    those existing entries are mutated; all randomization uses new entries.
    """
    new_scen = deepcopy(scenario)

    # Let MORAI choose the model independently for every vehicle emitted by
    # every NPC spawner. The reference scene fixes spawn #4 to 30100004 while
    # the other spawners already use -1; normalise all of them to MORAI's
    # random-vehicle sentinel without changing position, route, timing or
    # traffic density.
    for spawn in new_scen.get('spawnPointList', []):
        spawn['spawnVehicleTypeID'] = -1

    objects = new_scen.setdefault('objectList', [])
    object_template = deepcopy(objects[0]) if objects else None
    added_static_objects = []
    authored_waypoints = {
        str(path.get('pathName', '')): path.get('waypointData', [])
        for path in new_scen.get('waypointDataList', [])}
    with open(global_path_csv, newline='') as stream:
        route_rows = list(csv.DictReader(stream))
    # Authored static objects remain authored objects.  Only their model ID and
    # position are randomized, with position constrained to a true 1 m radius
    # around the source scenario. UNIQUEID, rotation and behavior fields stay
    # untouched. Scale is reset to native size because carrying CargoBox's
    # authored 2x3x2 scale into a randomly selected car/cone makes it huge.
    used_ids = []
    for value in new_scen.values():
        if not isinstance(value, list):
            continue
        for item in value:
            if isinstance(item, dict) and 'UNIQUEID' in item:
                try:
                    used_ids.append(int(item['UNIQUEID']))
                except (TypeError, ValueError):
                    pass
    next_uid = max(used_ids + [999])

    for obj in objects:
        old_uid = obj.get('UNIQUEID')
        # Preserve manually authored moving objects byte-for-byte. Some fixed
        # objects also have isWaypointMode=True with an empty ObstaclePath, so
        # that flag alone must not classify them as dynamic.
        object_path = authored_waypoints.get(
            "ObstaclePath - %s" % old_uid, [])
        is_authored_dynamic = (float(obj.get('moveSpeed', 0.0)) > 0.0 or
                               bool(object_path))
        if is_authored_dynamic:
            if old_uid is None:
                next_uid += 1
                obj['UNIQUEID'] = next_uid
            continue

        pos = obj.get('pos')
        if not isinstance(pos, dict):
            continue
        base_x = float(pos['x'])
        base_y = float(pos['y'])
        radius = math.sqrt(random.random()) * 1.0
        angle = random.uniform(-math.pi, math.pi)
        x = base_x + radius * math.cos(angle)
        y = base_y + radius * math.sin(angle)
        z = float(pos['z'])

        obj['DataID'] = random.choice(COMPACT_STATIC_OBSTACLE_DATA_IDS)
        obj['scale'] = {
            "x": 1.0, "y": 1.0, "z": 1.0,
            "_x": "1.000", "_y": "1.000", "_z": "1.000"}
        obj['pos'] = {
            "x": x, "y": y, "z": z,
            "_x": f"{x:.3f}", "_y": f"{y:.3f}", "_z": f"{z:.3f}"}

        if old_uid is not None:
            obj['UNIQUEID'] = old_uid
        else:
            next_uid += 1
            obj['UNIQUEID'] = next_uid
    # Three general sections plus one dedicated shaded-road section. All
    # coordinates, headings and heights still come from the active route.
    static_zones = _route_zones(global_path_csv, [
        (0.10, 0.18), (0.32, 0.42), (0.58, 0.68)])
    dynamic_zones = _route_zones(global_path_csv, [
        (0.20, 0.29), (0.45, 0.55), (0.70, 0.80)])
    shaded_zone = _route_zones(global_path_csv, [(0.875, 0.895)])[0]

    shaded_is_dynamic = random.random() < 0.50
    if not shaded_is_dynamic:
        static_zones.append(shaded_zone)

    # Distribute four new static obstacles over separate route sections.
    # Place added static objects within 1 m left/right of global-path centre.
    for zone_x, zone_y, zone_z, zone_yaw in static_zones:
        heading = math.radians(zone_yaw)
        longitudinal = random.uniform(-2.0, 2.0)
        lateral = random.uniform(-1.0, 1.0)
        obstacle_x = (zone_x + longitudinal * math.cos(heading) -
                      lateral * math.sin(heading))
        obstacle_y = (zone_y + longitudinal * math.sin(heading) +
                      lateral * math.cos(heading))
        obstacle_yaw = zone_yaw + random.uniform(-12.0, 12.0)
        next_uid += 1
        obstacle_pos = {
            "x": obstacle_x, "y": obstacle_y, "z": zone_z,
            "_x": f"{obstacle_x:.3f}", "_y": f"{obstacle_y:.3f}",
            "_z": f"{zone_z:.3f}"
        }
        if object_template is None:
            continue
        obstacle = deepcopy(object_template)
        obstacle['DataID'] = random.choice(NON_VEHICLE_OBSTACLE_DATA_IDS)
        obstacle['UNIQUEID'] = next_uid
        obstacle['pos'] = obstacle_pos
        obstacle['rot'] = {
            "roll": "0.000", "pitch": "0.000",
            "yaw": f"{obstacle_yaw:.3f}"}
        obstacle['scale'] = {
            "x": 1.0, "y": 1.0, "z": 1.0,
            "_x": "1.000", "_y": "1.000", "_z": "1.000"}
        # The authored object happens to use waypoint mode with an empty path.
        # Cloning those fields makes newly appended objects disappear in MORAI
        # or remain tied to the template's original/zero init position.  Added
        # random obstacles are ordinary fixed world objects.
        obstacle['initPos'] = deepcopy(obstacle_pos)
        obstacle['initRot'] = deepcopy(obstacle['rot'])
        obstacle['isWaypointMode'] = False
        obstacle['moveSpeed'] = 0.0
        obstacle['activeDistance'] = 999.0
        obstacle['waypointEventList'] = []
        objects.append(obstacle)
        added_static_objects.append(obstacle)

    # Ensure multiple pedestrian events per episode instead of just 20% chance.
    # We place pedestrians in 2 to 3 random dynamic zones, plus possibly the shaded zone.
    pedestrians = new_scen.setdefault('pedestrianList', [])
    waypoint_data = new_scen.setdefault('waypointDataList', [])
    pedestrian_template = deepcopy(pedestrians[0]) if pedestrians else None

    # Authored pedestrians are fixed dynamic obstacles: preserve their model,
    # pose, movement state, UID and waypoint association exactly as authored.

    pedestrian_events = []

    chosen_dynamic_zones = random.sample(dynamic_zones, k=random.randint(2, 3))
    for zone in chosen_dynamic_zones:
        event_type = random.choices(
            ('stationary', 'crossing', 'sudden_entry'),
            weights=(0.20, 0.40, 0.40), k=1)[0]
        pedestrian_events.append((zone, event_type))

    if shaded_is_dynamic:
        event_types = ('crossing', 'sudden_entry')
        pedestrian_events.append((shaded_zone, random.choice(event_types)))

    for (center_x, center_y, road_z, route_yaw), event_type in pedestrian_events:
        next_uid += 1
        cross_yaw = route_yaw + 90.0
        cross_rad = math.radians(cross_yaw)
        # Authored fixed pedestrian has movingDistance=5.0. Match that scale.
        half_crossing = random.uniform(2.0, 3.0)
        direction = random.choice((-1.0, 1.0))
        dx = direction * half_crossing * math.cos(cross_rad)
        dy = direction * half_crossing * math.sin(cross_rad)
        p1 = {
            "x": center_x - dx, "y": center_y - dy, "z": road_z,
            "_x": f"{center_x - dx:.3f}", "_y": f"{center_y - dy:.3f}",
            "_z": f"{road_z:.3f}"
        }
        p2 = {
            "x": center_x + dx, "y": center_y + dy, "z": road_z,
            "_x": f"{center_x + dx:.3f}", "_y": f"{center_y + dy:.3f}",
            "_z": f"{road_z:.3f}"
        }

        # Unity 좌표계 변환 함수 (보행자의 initPos, standardPos, waypoint 용)
        def to_wp(p):
            wpx, wpy, wpz = round(-p["x"], 3), round(p["z"], 3), round(-p["y"], 3)
            return {"x": wpx, "y": wpy, "z": wpz, "_x": f"{wpx:.3f}", "_y": f"{wpy:.3f}", "_z": f"{wpz:.3f}"}

        p1_wp = to_wp(p1)
        p2_wp = to_wp(p2)

        yaw_text = f"{math.degrees(math.atan2(p2['y'] - p1['y'], p2['x'] - p1['x'])):.3f}"

        # Match authored fixed pedestrian: activeDistance must be high enough
        # so it starts moving before ego comes to a full stop (deadlock).
        if event_type == 'stationary':
            shoulder = random.uniform(2.0, 4.0)
            p1 = {
                "x": center_x + shoulder * math.cos(cross_rad),
                "y": center_y + shoulder * math.sin(cross_rad),
                "z": road_z,
                "_x": f"{center_x + shoulder * math.cos(cross_rad):.3f}",
                "_y": f"{center_y + shoulder * math.sin(cross_rad):.3f}",
                "_z": f"{road_z:.3f}"
            }
            p1_wp = to_wp(p1)
            p2_wp = to_wp(p1) # 정지 상태이므로 p2도 p1과 같게 유지

            speed = 0.0
            active_distance = 15.0
        elif event_type == 'sudden_entry':
            speed = random.uniform(8.0, 11.0)
            active_distance = 50.0
        else:
            speed = random.uniform(8.0, 11.0)
            active_distance = 50.0

        # Start from the scenario's known-good pedestrian entry so MORAI also
        # receives its animation/state metadata.  A hand-built partial actor
        # dictionary can be displayed but never advance along PedPath.
        pedestrian = (deepcopy(pedestrian_template)
                      if pedestrian_template is not None else {})
        pedestrian.update({
            "DataID": random.choice(PEDESTRIAN_DATA_IDS),
            "UNIQUEID": next_uid,
            "m_eobstacleObjType": 2,
            "pos": dict(p1),
            "rot": {"roll": "0.000", "pitch": "0.000", "yaw": yaw_text},
            "activeDistance": active_distance,
            "speed": speed,
            "active": True,
            "loop": event_type != 'stationary',
            "initPos": dict(p1_wp),
            "initRot": {"roll": "0.000", "pitch": "0.000", "yaw": yaw_text},
            "movingDistance": 2.0 * half_crossing,
            "movingDistanceAmount": 0.0,
            "standardPos": dict(p1_wp),
            "pedestrianBehavior": 1,
            "pedestrianType": 1,
            "listObjIDForBehavior": [],
            "isWaypointMode": True,
            "waypointEventList": [
                {"index": -1, "waypointEventData": []},
                {"index": -1, "waypointEventData": []}
            ]
        })
        pedestrians.append(pedestrian)
        waypoint_data.append({
            "pathName": f"PedPath - {next_uid}",
            "waypointData": [
                {"pos": dict(p1_wp), "rot": {"roll": "0.000", "pitch": "0.000", "yaw": yaw_text}},
                {"pos": dict(p2_wp), "rot": {"roll": "0.000", "pitch": "0.000", "yaw": yaw_text}}
            ]
        })
    removed_static = _remove_static_blocking_npc_routes(
        new_scen, added_static_objects, clearance_m=3.0)
    if removed_static:
        print("[Domain Randomizer] Removed static obstacles blocking NPC "
              "routes: %s" % sorted(removed_static))
    return new_scen


def generate_randomized_scenario(base_json_path, output_json_path,
                                 global_path_csv=None):
    with open(base_json_path, 'r', encoding='utf-8') as f:
        scenario = json.load(f)

    if global_path_csv is None:
        global_path_csv = os.path.expanduser('~/acca_ws/global_path/global_path.csv')

    # Keep every authored ego/NPC/spawn/traffic-light/fixed-obstacle setting.
    # Only append the global-path-based randomized actors. The legacy
    # apply_zone_rules() rewrote NPC routes and added overlapping obstacles,
    # so it must not participate in automated teacher-data episodes.
    new_scen = apply_obstacle_only_rules(scenario, global_path_csv)

    output_dir = os.path.dirname(output_json_path) or '.'
    os.makedirs(output_dir, exist_ok=True)
    temporary_path = "%s.tmp.%d" % (output_json_path, os.getpid())
    try:
        with open(temporary_path, 'w', encoding='utf-8') as f:
            json.dump(new_scen, f, indent=2)
            f.flush()
            os.fsync(f.fileno())
        # MORAI can inspect the scenario directory concurrently. Atomic
        # replacement prevents it from parsing a half-written JSON document.
        os.replace(temporary_path, output_json_path)
    finally:
        try:
            os.unlink(temporary_path)
        except FileNotFoundError:
            pass

    print(f"[Domain Randomizer] Successfully generated scenario: {output_json_path}")

def parse_ros_args():
    input_path = "/home/acca/MoraiLauncher_Lin/MoraiLauncher_Lin_Data/SaveFile/Scenario/R_KR_PR_K-city_2025/2026_molit_comp_sample_scene.json"
    output_path = "/home/acca/MoraiLauncher_Lin/MoraiLauncher_Lin_Data/SaveFile/Scenario/R_KR_PR_K-city_2025/randomized_final.json"
    count = 1

    clean_argv = []
    for arg in sys.argv[1:]:
        if arg.startswith('_input:='):
            input_path = arg.split(':=', 1)[1]
        elif arg.startswith('_output:='):
            output_path = arg.split(':=', 1)[1]
        elif arg.startswith('_count:='):
            count = int(arg.split(':=', 1)[1])
        elif not arg.startswith('_'):
            clean_argv.append(arg)

    return input_path, output_path, count, clean_argv

def main():
    input_path, output_path, count, clean_argv = parse_ros_args()

    parser = argparse.ArgumentParser(description="Custom Rules MORAI Domain Randomizer")
    parser.add_argument("--input", default=input_path)
    parser.add_argument("--output", default=output_path)
    parser.add_argument("--count", type=int, default=count)
    args, _ = parser.parse_known_args(clean_argv)

    if args.count == 1:
        generate_randomized_scenario(args.input, args.output)
    else:
        base_dir = os.path.dirname(args.output)
        base_name, ext = os.path.splitext(os.path.basename(args.output))
        for i in range(1, args.count + 1):
            out = os.path.join(base_dir, f"{base_name}_{i}{ext}")
            generate_randomized_scenario(args.input, out)

if __name__ == '__main__':
    main()
