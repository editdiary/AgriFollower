#!/usr/bin/env python3
"""토마토 온실 Ignition Gazebo world(SDF 1.6) 생성기.

식물을 줄(row)로 배치하고 줄 사이에 로봇 통로를 두며, 온실 외벽으로 경계를 만든다.
외부 메시 의존성 없이 인라인 프리미티브(box/plane)만 사용한다.

옛 ws의 Gazebo Classic 생성기를 Ignition용으로 개조한 것:
  - OGRE `.material` 스크립트 → 인라인 PBR 머티리얼(<pbr><metal><albedo_map>)
  - <include>model://sun</include> → 인라인 directional light
  - physics type="ode" → physics-system 플러그인 위임(type="ignored")
  - 좌표를 평행이동해 "통로 입구 = 원점(0,0, yaw=0)"으로 둔다.
    → robot_gazebo 의 spwan_model.launch.py 가 (0,0)에 ROSOrin 을 스폰하므로 수정 없이 통로 입구에 선다.

텍스처는 SDF 기준 상대경로(media/materials/textures/*.jpg)로 참조하며,
greenhouse.launch.py 가 IGN_GAZEBO_RESOURCE_PATH 에 패키지 share 를 추가해 해결한다.

출력: src/greenhouse_sim/worlds/greenhouse.sdf
실행: python3 src/greenhouse_sim/scripts/gen_greenhouse_world.py
"""

import os
import random
from glob import glob

# ---- 레이아웃 파라미터 (RL/실험 시 여기만 조정) ----
RANDOM_SEED = 0       # 작물 면 텍스처 랜덤 배치 시드 (재현성). 이미지 집합 바뀌면 배치도 바뀜.
NUM_ROWS = 4          # 식물 줄 개수 (y 방향으로 늘어섬). 짝수 → 중앙 통로가 y=0
                      #   (로봇/타겟 reset_y=0 과 일치). 4열 → 통로 3개(y=-1/0/+1)
PLANTS_PER_ROW = 12   # 줄당 식물 수 (x 방향으로 늘어섬)
PLANT_SPACING_X = 0.5  # 같은 줄 내 식물 간격 [m]
AISLE_WIDTH = 0.8      # 인접한 줄의 잎 면 사이 실제 통로 폭 [m]

# 작물 = 무성한 잎 "커튼"(초록 박스). 한 줄 안에서 잎 박스들이 거의 맞닿아 연속 벽을 이룬다.
FOLIAGE_W_X = PLANT_SPACING_X * 0.96  # 잎 가로(x) — 간격보다 살짝 작아 인접 잎과 거의 맞닿음
FOLIAGE_D_Y = 0.2     # 잎 깊이(y) — 통로 쪽으로 차지하는 두께
FOLIAGE_H = 1.4        # 잎 높이(z)
FOLIAGE_BASE_Z = 0.0   # 잎 박스 바닥 높이(지면에 붙음) — LiDAR(~0.17m)가 연속 벽으로 검출
FOLIAGE_PANEL_T = 0.01  # 통로 면에 덧대는 사진 패널 두께(±y 표면 바로 바깥)
FOLIAGE_V_SEGMENTS = 3  # 잎 패널 세로 분할 칸 수 (칸마다 다른 랜덤 이미지 → 비율 보존)

WALL_HEIGHT = 2.5      # 작물(약 1.48m)보다 높은 유리 외벽
WALL_THICK = 0.05
WALL_MARGIN = 1.5      # 가장자리 작물과 외벽 사이 여유 [m] (통로 끝 여유 공간)

# 텍스처 경로 prefix (SDF 기준 상대경로; 런치에서 IGN_GAZEBO_RESOURCE_PATH 로 해결)
TEX_PREFIX = "../media/materials/textures"

SOIL_IMAGE = "soil_img.jpg"   # 바닥 흙 텍스처 파일(잎 패널에는 사용 안 함)
SOIL_TILE_SIZE = 1.0          # 흙 타일 한 변 [m] (작을수록 흙 입자 촘촘; world 재생성 필요)
SOIL_TILE_THICK = 0.002       # 흙 타일(얕은 박스) 두께 [m]
SOIL_TILE_Z = 0.003           # 베이스 평면 위로 살짝 띄워 z-fighting 방지 [m]


def scan_foliage_images(textures_dir):
    """textures 디렉터리의 작물 사진 파일명 목록(정렬). 이미지를 더 넣으면 자동 포함."""
    names = []
    for pat in ("*.jpg", "*.jpeg", "*.png"):
        names += [os.path.basename(p) for p in glob(os.path.join(textures_dir, pat))]
    return sorted(n for n in set(names) if n != SOIL_IMAGE)


def foliage_albedo_material(image_name):
    """잎 패널용 인라인 PBR 머티리얼. albedo_map 으로 작물 사진을 입힌다."""
    return (f"<material>"
            f"<diffuse>1 1 1 1</diffuse><specular>0 0 0 1</specular>"
            f"<pbr><metal>"
            f"<albedo_map>{TEX_PREFIX}/{image_name}</albedo_map>"
            f"<metalness>0.0</metalness><roughness>1.0</roughness>"
            f"</metal></pbr>"
            f"</material>")


def soil_albedo_material():
    """바닥 타일용 인라인 PBR 머티리얼 (흙 사진). foliage 와 동일 기법 재사용."""
    return foliage_albedo_material(SOIL_IMAGE)


def plant_model(name, x, y, image_names):
    """한 그루 = static 모델 1개.
    - foliage_col: 충돌 박스(LiDAR 검출). 변경 금지.
    - foliage_core: 단색 초록 박스 → 윗면/끝면/작물 사이 틈을 자연스러운 초록으로.
    - 통로 면(\u00b1y)에 얇은 패널을 세로 FOLIAGE_V_SEGMENTS 칸으로 쌓고 칸마다 다른 랜덤 사진(PBR albedo).
      칸 면이 거의 정사각이라 원본 사진 비율 왜곡이 줄어든다.
      -y 패널은 박스 UV가 뒤집혀 보이므로 pitch=\u03c0 로 면내 180\u00b0 회전해 보정.
    """
    foliage_z = FOLIAGE_BASE_Z + FOLIAGE_H / 2.0
    panel_y = FOLIAGE_D_Y / 2.0 + FOLIAGE_PANEL_T / 2.0  # 박스 ±y 표면 바로 바깥
    seg_h = FOLIAGE_H / FOLIAGE_V_SEGMENTS

    panels = []
    for side, sign, pitch in (("py", 1.0, "0"), ("ny", -1.0, "3.14159")):
        for k in range(FOLIAGE_V_SEGMENTS):
            seg_z = FOLIAGE_BASE_Z + (k + 0.5) * seg_h
            img = random.choice(image_names)
            panels.append(
                f'        <visual name="foliage_panel_{side}_{k}">\n'
                f'          <pose>0 {sign * panel_y:.3f} {seg_z:.3f} 0 {pitch} 0</pose>\n'
                f'          <geometry><box><size>{FOLIAGE_W_X:.3f} {FOLIAGE_PANEL_T:.3f} {seg_h:.3f}</size></box></geometry>\n'
                f'          {foliage_albedo_material(img)}\n'
                f'        </visual>')
    panels_xml = "\n".join(panels)

    return f"""    <model name="{name}">
      <static>true</static>
      <pose>{x:.3f} {y:.3f} 0 0 0 0</pose>
      <link name="link">
        <collision name="foliage_col">
          <pose>0 0 {foliage_z:.3f} 0 0 0</pose>
          <geometry><box><size>{FOLIAGE_W_X:.3f} {FOLIAGE_D_Y:.3f} {FOLIAGE_H:.3f}</size></box></geometry>
        </collision>
        <visual name="foliage_core">
          <pose>0 0 {foliage_z:.3f} 0 0 0</pose>
          <geometry><box><size>{FOLIAGE_W_X:.3f} {FOLIAGE_D_Y:.3f} {FOLIAGE_H:.3f}</size></box></geometry>
          <material><ambient>0.1 0.4 0.1 1</ambient><diffuse>0.15 0.55 0.15 1</diffuse></material>
        </visual>
{panels_xml}
      </link>
    </model>
"""


def wall_model(name, x, y, sx, sy):
    return f"""    <model name="{name}">
      <static>true</static>
      <pose>{x:.3f} {y:.3f} {WALL_HEIGHT/2:.3f} 0 0 0</pose>
      <link name="link">
        <collision name="c"><geometry><box><size>{sx:.3f} {sy:.3f} {WALL_HEIGHT}</size></box></geometry></collision>
        <visual name="v"><geometry><box><size>{sx:.3f} {sy:.3f} {WALL_HEIGHT}</size></box></geometry>
          <material>
            <ambient>0.55 0.62 0.65 1</ambient>
            <diffuse>0.78 0.85 0.88 1</diffuse>
            <specular>0.9 0.9 0.9 1</specular>
          </material>
        </visual>
      </link>
    </model>
"""


def door_model(name, half_x, px, py):
    """통로 입구(서쪽 벽)의 장식용 문. 벽은 그대로 막혀 있고 시각만 '문'(문틀+문짝+손잡이).
    px/py: 평행이동 후 모델 원점 위치. 내부 x 는 -half_x 기준(서쪽 벽)으로 계산하고
    모델 pose 로 전체를 (px,py) 만큼 옮긴다."""
    inner = -half_x + WALL_THICK / 2.0  # 서쪽 벽 안쪽 면
    frame_x = inner + 0.02
    slab_x = inner + 0.05
    handle_x = slab_x + 0.03
    return f"""    <model name="{name}">
      <static>true</static>
      <pose>{px:.3f} {py:.3f} 0 0 0 0</pose>
      <link name="link">
        <visual name="frame">
          <pose>{frame_x:.3f} 0 1.05 0 0 0</pose>
          <geometry><box><size>0.06 1.00 2.10</size></box></geometry>
          <material><ambient>0.5 0.5 0.5 1</ambient><diffuse>0.8 0.8 0.8 1</diffuse></material>
        </visual>
        <visual name="slab">
          <pose>{slab_x:.3f} 0 1.00 0 0 0</pose>
          <geometry><box><size>0.04 0.90 2.00</size></box></geometry>
          <material><ambient>0.25 0.12 0.05 1</ambient><diffuse>0.45 0.22 0.10 1</diffuse></material>
        </visual>
        <visual name="handle">
          <pose>{handle_x:.3f} 0.35 1.00 0 0 0</pose>
          <geometry><box><size>0.04 0.08 0.04</size></box></geometry>
          <material><ambient>0.3 0.25 0.05 1</ambient><diffuse>0.75 0.6 0.2 1</diffuse></material>
        </visual>
      </link>
    </model>
"""


def compute_layout():
    """줄/식물 좌표와 외벽 크기, 그리고 로봇 스폰(통로 입구) 좌표 계산."""
    # 줄(y) 위치: 원점 기준 대칭. 잎 깊이를 감안해 줄 중심 간격(pitch) = 통로폭 + 잎 깊이
    row_pitch = AISLE_WIDTH + FOLIAGE_D_Y
    y0 = -(NUM_ROWS - 1) * row_pitch / 2.0
    row_y = [y0 + i * row_pitch for i in range(NUM_ROWS)]

    # 식물(x) 위치: 원점 기준 대칭.
    x0 = -(PLANTS_PER_ROW - 1) * PLANT_SPACING_X / 2.0
    plant_x = [x0 + j * PLANT_SPACING_X for j in range(PLANTS_PER_ROW)]

    # 외벽 반경.
    half_x = abs(x0) + FOLIAGE_W_X / 2.0 + WALL_MARGIN
    half_y = abs(y0) + FOLIAGE_D_Y / 2.0 + WALL_MARGIN

    # 로봇 스폰(통로 입구, 서쪽 벽 안쪽). 줄이 홀수면 가운데 통로가 row_pitch/2 에 옴.
    spawn_y = 0.0 if NUM_ROWS % 2 == 0 else row_pitch / 2.0
    spawn_x = -(abs(x0) + FOLIAGE_W_X / 2.0 + 0.6)
    return dict(row_pitch=row_pitch, row_y=row_y, plant_x=plant_x,
                half_x=half_x, half_y=half_y, spawn_x=spawn_x, spawn_y=spawn_y)


def build_world(image_names, lay):
    random.seed(RANDOM_SEED)  # 작물 면 텍스처 랜덤 배치 재현성

    # 통로 입구를 원점으로 두기 위한 평행이동: 모든 좌표에서 spawn 좌표를 뺀다.
    ox, oy = lay['spawn_x'], lay['spawn_y']

    half_x, half_y = lay['half_x'], lay['half_y']
    span_x = 2 * half_x + WALL_THICK
    span_y = 2 * half_y + WALL_THICK

    # 흙 바닥 타일 격자: 온실 footprint 를 SOIL_TILE_SIZE 칸으로 덮어 흙 사진을 타일링한다.
    # (Ignition <plane> 은 UV 0..1 고정이라 한 장이 늘어남 → 작은 타일 반복으로 타일링 효과)
    S = SOIL_TILE_SIZE
    nx = int(2 * half_x / S) + 1
    ny = int(2 * half_y / S) + 1
    soil_mat = soil_albedo_material()
    _tiles = []
    for ti in range(nx):
        cx = -ox + (ti - (nx - 1) / 2.0) * S
        for tj in range(ny):
            cy = -oy + (tj - (ny - 1) / 2.0) * S
            _tiles.append(
                f'        <visual name="soil_tile_{ti}_{tj}">\n'
                f'          <cast_shadows>false</cast_shadows>\n'
                f'          <pose>{cx:.3f} {cy:.3f} {SOIL_TILE_Z} 0 0 0</pose>\n'
                f'          <geometry><box><size>{S} {S} {SOIL_TILE_THICK}</size></box></geometry>\n'
                f'          {soil_mat}\n'
                f'        </visual>\n')
    soil_tiles = ''.join(_tiles)
    print(f'  soil tiles={nx * ny} ({nx}x{ny}, tile={S}m) -> {SOIL_IMAGE}')

    models = []
    for i, y in enumerate(lay['row_y']):
        for j, x in enumerate(lay['plant_x']):
            models.append(plant_model(f"tomato_r{i}_p{j}", x - ox, y - oy, image_names))

    models.append(wall_model("wall_north", 0.0 - ox, half_y - oy, span_x, WALL_THICK))
    models.append(wall_model("wall_south", 0.0 - ox, -half_y - oy, span_x, WALL_THICK))
    models.append(wall_model("wall_east", half_x - ox, 0.0 - oy, WALL_THICK, span_y))
    models.append(wall_model("wall_west", -half_x - ox, 0.0 - oy, WALL_THICK, span_y))

    # 로봇 스폰(통로 입구, 서쪽 벽)에 장식용 문. 전체를 (-ox, -oy) 만큼 평행이동.
    models.append(door_model("entrance_door", half_x, -ox, lay['spawn_y'] - oy))

    header = f"""<?xml version="1.0" ?>
<sdf version="1.6">
  <world name="greenhouse_world">

    <physics name="default_physics" type="ignored">
      <max_step_size>0.001</max_step_size>
      <real_time_factor>0</real_time_factor>
    </physics>
    <plugin filename="ignition-gazebo-physics-system"
            name="ignition::gazebo::systems::Physics"></plugin>
    <plugin filename="ignition-gazebo-user-commands-system"
            name="ignition::gazebo::systems::UserCommands"></plugin>
    <plugin filename="ignition-gazebo-scene-broadcaster-system"
            name="ignition::gazebo::systems::SceneBroadcaster"></plugin>
    <plugin filename="ignition-gazebo-contact-system"
            name="ignition::gazebo::systems::Contact"></plugin>

    <!-- 장면 조명: sun 하나만으로는 외벽·작물 그림자에 바닥이 어두워 ambient 상향. -->
    <scene>
      <ambient>0.6 0.6 0.6 1</ambient>
      <background>0.7 0.8 0.9 1</background>
      <shadows>true</shadows>
    </scene>

    <light type="directional" name="sun">
      <cast_shadows>true</cast_shadows>
      <pose>0 0 10 0 0 0</pose>
      <diffuse>0.9 0.9 0.9 1</diffuse>
      <specular>0.2 0.2 0.2 1</specular>
      <attenuation>
        <range>1000</range>
        <constant>0.9</constant>
        <linear>0.01</linear>
        <quadratic>0.001</quadratic>
      </attenuation>
      <direction>-0.4 0.2 -0.9</direction>
    </light>

    <!-- 흙 바닥: 100x100 평면(충돌+바깥 배경 갈색) + 온실 footprint 를 덮는 흙 사진 타일 격자. -->
    <model name="soil_ground">
      <static>true</static>
      <link name="link">
        <collision name="collision">
          <geometry><plane><normal>0 0 1</normal><size>100 100</size></plane></geometry>
          <surface><friction><ode><mu>100</mu><mu2>50</mu2></ode></friction></surface>
        </collision>
        <visual name="base">
          <cast_shadows>false</cast_shadows>
          <geometry><plane><normal>0 0 1</normal><size>100 100</size></plane></geometry>
          <material>
            <ambient>0.30 0.20 0.12 1</ambient>
            <diffuse>0.40 0.27 0.16 1</diffuse>
            <specular>0.0 0.0 0.0 1</specular>
          </material>
        </visual>
{soil_tiles}      </link>
    </model>

"""
    footer = "  </world>\n</sdf>\n"
    return header + "".join(models) + footer


def main():
    # scripts/ 의 부모 = greenhouse_sim 패키지 루트
    pkg_root = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    textures_dir = os.path.join(pkg_root, "media", "materials", "textures")
    out_path = os.path.join(pkg_root, "worlds", "greenhouse.sdf")

    image_names = scan_foliage_images(textures_dir)
    if not image_names:
        raise SystemExit(f"no foliage textures found in {textures_dir}")

    lay = compute_layout()
    with open(out_path, "w") as f:
        f.write(build_world(image_names, lay))

    n_plants = NUM_ROWS * PLANTS_PER_ROW
    print(f"wrote {out_path}")
    print(f"  rows={NUM_ROWS} plants/row={PLANTS_PER_ROW} total_plants={n_plants} aisle={AISLE_WIDTH}m")
    print(f"  foliage panels={n_plants * 2 * FOLIAGE_V_SEGMENTS} randomly textured from {len(image_names)} images (seed={RANDOM_SEED})")
    print(f"  좌표 평행이동: 통로 입구가 원점(0,0)에 오도록 이동했음.")
    print(f"  → robot_gazebo spwan_model.launch.py 의 (-x 0 -y 0) 스폰이 통로 입구·+x 방향과 일치.")


if __name__ == "__main__":
    main()
