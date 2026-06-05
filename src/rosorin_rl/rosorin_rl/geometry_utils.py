"""geometry_utils.py — 카메라/좌표 변환 기하 유틸리티.

설계 출처: docs/rl_design/rl_state_space.md §2.1, §2.5
환경 수치 출처: docs/environment.md "카메라 Intrinsic & Cam–LiDAR Extrinsic"

이 모듈이 하는 일:
1. 시뮬 카메라의 intrinsic(내부 파라미터) 상수 정의 — 시뮬은 ground truth라 캘리브레이션 불필요.
2. 쿼터니언 → yaw 변환.
3. /world/.../dynamic_pose/info (TFMessage) 에서 특정 엔티티의 pose 추출.
4. 로봇·타겟의 월드 좌표(ground truth) → "카메라가 마커를 봤다면 얻었을" 특징
   [x_norm, y_norm, d_t, θ_t] 로 변환 (rl_state_space.md §2.1 의 역연산).

⚠️ 좌표계 주의 (docs/environment.md):
  camera_link0 은 optical frame 이 아니라 X-forward body 좌표계다.
  픽셀 투영 시 ROS 광학 관례(Z-forward, X-right, Y-down)로 회전해야 한다:
    X_opt = -Y_body,  Y_opt = -Z_body,  Z_opt = X_body
"""

import math

# ---------------------------------------------------------------------------
# 카메라 intrinsic (시뮬 ground truth — docs/environment.md 표)
# fx = fy = (W/2) / tan(hfov/2) = 320 / tan(0.5235) ≈ 554.26
# ---------------------------------------------------------------------------
CAM = {
    'fx': 554.26, 'fy': 554.26,   # 초점거리 [px]
    'cx': 320.0, 'cy': 200.0,     # 주점(영상 중심) [px]
    'width': 640, 'height': 400,  # 해상도
    'hfov': 1.047,                # 수평 화각 60° [rad]
}

# camera_link0 의 base_link 기준 마운트 위치 [m]
# (스케일 포크 S=1.83 반영값 — greenhouse_sim/urdf/ascamera_scaled.xacro)
CAM_MOUNT = {'x': 0.10499, 'y': 0.00014, 'z': 0.16811}

# 온실 통로가 뻗은 방향 (월드 +x). R_pose_center 의 θ_aisle 기준값.
AISLE_HEADING = 0.0

# ---------------------------------------------------------------------------
# 차폐물(occluder) — 작물 줄 3개의 2D AABB (xmin, ymin, xmax, ymax) [m, 월드좌표]
#
# 출처: greenhouse_sim/scripts/gen_greenhouse_world.py 레이아웃
#   NUM_ROWS=3, row_pitch=AISLE_WIDTH(0.8)+FOLIAGE_D_Y(0.2)=1.0,
#   원점 평행이동(통로 입구=원점) 후 줄 중심 y = −1.5 / −0.5 / +0.5, 잎 깊이 ±0.1
#   x 범위: 식물 중심 0.84..6.34 ± 잎 가로 반폭 0.24 → 0.60..6.58
# 잎 높이 1.4m > 마커 높이 0.35m 이므로 수평(2D) 교차 검사만으로 충분하다.
#
# 왜 필요한가 (4차 수정): 가시성 판정이 화각만 검사해서 작물 벽 너머의 타겟도
# visible=1 로 발행되는 "벽 투시" 버그가 있었다. 실물 AprilTag 검출에선 불가능한
# 정보라 Sim-to-Real 갭이며, 로봇이 다른 통로에서 타겟을 "아는" 행동의 원인.
OCCLUDERS = [
    (0.60, -1.6, 6.58, -1.4),   # 남쪽 줄
    (0.60, -0.6, 6.58, -0.4),   # 가운데 줄 (로봇 통로의 남쪽 잎벽)
    (0.60,  0.4, 6.58,  0.6),   # 북쪽 줄 (로봇 통로의 북쪽 잎벽)
]


def quat_to_yaw(qx, qy, qz, qw):
    """쿼터니언 → yaw(z축 회전각, rad).

    로봇은 평지 주행이라 roll/pitch ≈ 0 이므로 yaw 만 있으면 충분하다.
    """
    siny_cosp = 2.0 * (qw * qz + qx * qy)
    cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz)
    return math.atan2(siny_cosp, cosy_cosp)


def segment_hits_aabb_2d(x1, y1, x2, y2, box):
    """2D 선분 (x1,y1)→(x2,y2) 가 AABB box=(xmin,ymin,xmax,ymax) 와 교차하는가.

    표준 slab 방식: 선분을 매개변수 t∈[0,1] 로 두고, 각 축에서 박스 구간에
    들어가는 t 범위를 구해 교집합이 남으면 교차. (차폐 검사 전용)
    """
    xmin, ymin, xmax, ymax = box
    dx, dy = x2 - x1, y2 - y1
    t0, t1 = 0.0, 1.0
    for p, d, lo, hi in ((x1, dx, xmin, xmax), (y1, dy, ymin, ymax)):
        if abs(d) < 1e-12:
            if p < lo or p > hi:        # 축과 평행 + slab 밖 → 교차 불가
                return False
        else:
            ta, tb = (lo - p) / d, (hi - p) / d
            if ta > tb:
                ta, tb = tb, ta
            t0, t1 = max(t0, ta), min(t1, tb)
            if t0 > t1:
                return False
    return True


def extract_pose(tf_msg, name):
    """TFMessage(= 브리지된 ignition Pose_V) 에서 엔티티 `name` 의 pose 를 찾는다.

    /world/greenhouse_world/dynamic_pose/info 는 월드 내 모든 '움직이는' 모델의
    ground-truth 월드 pose 를 담고 있다 (드리프트 없는 참값 — /odom 은 적분이라 드리프트).

    Returns:
        (x, y, z, yaw) 튜플, 못 찾으면 None.
    """
    for tr in tf_msg.transforms:
        if tr.child_frame_id == name:
            t = tr.transform.translation
            q = tr.transform.rotation
            return (t.x, t.y, t.z, quat_to_yaw(q.x, q.y, q.z, q.w))
    return None


def world_to_target_features(robot_x, robot_y, robot_yaw,
                             target_x, target_y, marker_z=0.20):
    """월드 좌표(ground truth) → 카메라 기준 타겟 특징 [x_norm, y_norm, d_t, θ_t].

    실물에서는 RGB-D 영상에서 마커(AprilTag)를 검출해 이 4개 값을 얻지만(rl_state_space.md §2.1),
    시뮬에서는 객체검출 대신 ground-truth pose 로 같은 값을 역산한다(roadmap 2단계 (b)).
    출력 인터페이스를 동일하게 유지해야 Sim-to-Real 때 비전 노드로 갈아끼울 수 있다.

    Args:
        robot_x/y/yaw: 로봇 base_link 의 월드 pose
        target_x/y:    타겟(원기둥) 중심의 월드 위치
        marker_z:      마커가 붙어 있다고 가정하는 높이 [m].
                       카메라(z=0.168m, 수평 마운트)의 수직 화각(±19.8°)에 마커
                       '전체'(±0.08m)가 근거리에서도 들어오도록 기본 0.20m 채택.
                       (초기값 0.35는 추종 거리 0.65m에서 마커 중심이 화면 상단
                        경계에 걸려 절반 잘림 — RViz 실측 후 인하. 0.20은 카메라
                        0.31m 근접까지 전체 시야 유지. ⚠️ 인하로 GT 역산 d_t 가
                        ~2.6cm 작아짐 — 수직 오프셋 0.182→0.032m. 노이즈 3% 수준.)

    Returns:
        dict(x_norm, y_norm, d_t, theta_t, visible)
        - x_norm, y_norm: 화면 중심 기준 정규화 픽셀 위치 [-1, 1] (좌상단이 음수)
        - d_t:    카메라 → 마커 3D 유클리디안 거리 [m]
        - theta_t: 수평 상대 각도 [rad] (오른쪽 +)
        - visible: 카메라 전방 + 수평 FOV(±30°) 이내 + 작물 줄에 가려지지 않음
          (수직 방향은 검사하지 않음 — 실제 사람은 키가 1.6m라 근접해도
           몸통 일부가 항상 보인다는 가정의 단순화. 주석으로 명시해 둠.
           차폐는 카메라→타겟 2D 선분 vs OCCLUDERS AABB 교차로 판정 — 4차 수정)
    """
    # --- 1) 타겟을 로봇 base_link 좌표계로 변환 (월드 → 로봇: -yaw 회전 + 평행이동) ---
    dx = target_x - robot_x
    dy = target_y - robot_y
    cos_y, sin_y = math.cos(-robot_yaw), math.sin(-robot_yaw)
    x_base = dx * cos_y - dy * sin_y          # 로봇 전방(+x) 성분
    y_base = dx * sin_y + dy * cos_y          # 로봇 좌측(+y) 성분
    z_base = marker_z                          # 지면 기준 마커 높이 (로봇 base ≈ 지면)

    # --- 2) base_link → camera_link0 (마운트 offset 빼기, 회전 없음) ---
    x_cam = x_base - CAM_MOUNT['x']
    y_cam = y_base - CAM_MOUNT['y']
    z_cam = z_base - CAM_MOUNT['z']

    # --- 3) body(X-forward) → optical(Z-forward) 관례 회전 ---
    #   X_opt(오른쪽+) = -Y_body, Y_opt(아래+) = -Z_body, Z_opt(전방+) = X_body
    x_opt = -y_cam
    y_opt = -z_cam
    z_opt = x_cam

    # --- 4) 특징 계산 (rl_state_space.md §2.5 수식) ---
    d_t = math.sqrt(x_opt ** 2 + y_opt ** 2 + z_opt ** 2)   # 3D 실측 거리
    theta_t = math.atan2(x_opt, max(z_opt, 1e-6))           # 수평 상대 각도

    # 핀홀 투영으로 픽셀 위치 → 정규화. 카메라 뒤(z_opt<=0)면 투영 불가.
    if z_opt > 0.05:
        u = CAM['fx'] * x_opt / z_opt + CAM['cx']
        v = CAM['fy'] * y_opt / z_opt + CAM['cy']
        x_norm = (u - CAM['cx']) / CAM['cx']
        y_norm = (v - CAM['cy']) / CAM['cy']
        # 극단값은 클리핑 (마커가 화면 가장자리 밖으로 나가는 순간의 폭주 방지)
        x_norm = max(-1.5, min(1.5, x_norm))
        y_norm = max(-1.5, min(1.5, y_norm))
    else:
        x_norm, y_norm = 0.0, 0.0

    # --- 5) 가시성 판정 ---
    # (a) 카메라 전방 + 수평 화각(±hfov/2) 이내
    visible = (z_opt > 0.05) and (abs(theta_t) <= CAM['hfov'] / 2.0)

    # (b) 차폐(occlusion): 카메라 월드 위치 → 타겟 선분이 작물 줄 AABB 를
    #     통과하면 가려진 것 (벽 투시 버그 수정 — 실물 AprilTag 과 동일 조건).
    #     카메라 월드 위치 = 로봇 pose + yaw 회전한 마운트 offset.
    if visible:
        cos_r, sin_r = math.cos(robot_yaw), math.sin(robot_yaw)
        cam_wx = robot_x + cos_r * CAM_MOUNT['x'] - sin_r * CAM_MOUNT['y']
        cam_wy = robot_y + sin_r * CAM_MOUNT['x'] + cos_r * CAM_MOUNT['y']
        for box in OCCLUDERS:
            if segment_hits_aabb_2d(cam_wx, cam_wy, target_x, target_y, box):
                visible = False
                break

    return {'x_norm': x_norm, 'y_norm': y_norm,
            'd_t': d_t, 'theta_t': theta_t, 'visible': visible}
