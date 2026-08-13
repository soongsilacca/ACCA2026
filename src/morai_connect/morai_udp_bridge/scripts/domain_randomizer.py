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

import json
import random
import math
import os
import sys
import argparse
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

PEDESTRIAN_DATA_IDS = [
    50100001, 50100002, 50100003, 50100004, 50100005, 50200001, 50200002
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
    # 1. 정적 장애물 구역 (Static Zone: X[-65.317, -59.866], Y[-148.949, -131.115])
    #    -> 전체 MORAI 물건 목록(ALL_STATIC_OBSTACLE_DATA_IDS) 중 무작위 선택 & 원본 1.0 스케일 유지
    # -------------------------------------------------------------
    static_x = random.uniform(-64.5, -60.5)
    static_y = random.uniform(-147.0, -133.0)
    static_yaw = random.uniform(-180.0, 180.0)
    chosen_static_id = random.choice(ALL_STATIC_OBSTACLE_DATA_IDS)

    if 'objectList' in new_scen and len(new_scen['objectList']) > 0:
        obj = new_scen['objectList'][0]
        obj['DataID'] = chosen_static_id
        obj['pos'] = {"x": static_x, "y": static_y, "z": 28.37, "_x": f"{static_x:.3f}", "_y": f"{static_y:.3f}", "_z": "28.370"}
        obj['rot']['yaw'] = f"{static_yaw:.3f}"
        obj['scale'] = {"x": 1.0, "y": 1.0, "z": 1.0, "_x": "1.000", "_y": "1.000", "_z": "1.000"}
        # initPos = pos와 동일 (0,0,0이면 시뮬레이터가 위치를 무시하는 버그 방지)
        obj['initPos'] = {"x": static_x, "y": static_y, "z": 28.37, "_x": f"{static_x:.3f}", "_y": f"{static_y:.3f}", "_z": "28.370"}
        obj['initRot'] = {"roll": "0.000", "pitch": "0.000", "yaw": f"{static_yaw:.3f}"}
        obj['isWaypointMode'] = False   # True이면 waypoint 없을 때 시뮬레이터가 스폰 안 함
        obj['moveSpeed'] = 0.0
        obj['activeDistance'] = 50.0   # 0이면 절대 스폰 안 됨 → 50m 반경으로 설정

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

def generate_randomized_scenario(base_json_path, output_json_path):
    with open(base_json_path, 'r', encoding='utf-8') as f:
        scenario = json.load(f)

    new_scen = apply_zone_rules(scenario)

    os.makedirs(os.path.dirname(output_json_path), exist_ok=True)
    with open(output_json_path, 'w', encoding='utf-8') as f:
        json.dump(new_scen, f, indent=2)

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
