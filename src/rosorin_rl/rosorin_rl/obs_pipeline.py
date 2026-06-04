"""obs_pipeline.py — 원시 센서 데이터 → 48차원 상태 벡터 가공 파이프라인.

설계 출처: docs/rl_design/rl_state_space.md (전체)

[ 단일 프레임 16차원 구성 (rl_state_space.md §3) ]
  인덱스  0-3 : 타겟 특징 [x_norm, y_norm, d_t, θ_t]        (§2.1, /target/features)
  인덱스  4-6 : 하단 뎁스 범퍼 [좌, 중, 우] 최단 거리       (§2.2, /depth_cam/depth_image)
  인덱스  7-12: LiDAR 6구역 안전 마진 [좌전방, 정면, 우전방,
                우후방, 후면, 좌후방]                       (§2.3, /scan)
  인덱스 13-15: 로봇 속도 [vx, vy, ω]                       (§2.4, /odom twist)

[ 최종 상태 48차원 ]
  최근 3프레임을 이어붙임: S_t = [s_t, s_{t-1}, s_{t-2}]  (§3, Frame Stacking)
  → 에이전트가 타겟의 이동 방향·자기 관성을 시간차로 유추할 수 있게 함.

[ 노이즈 필터링 (§2.3) ]
  - 백분위수 필터: 구역 최솟값 대신 하위 5퍼센타일 (단일 점 노이즈 무시)
  - 결측치 클리핑: NaN/Inf → 센서 최대 거리
  - 지수 이동 평균(EMA): 프레임 간 급격한 튐 평활화
"""

from collections import deque

import numpy as np


# LiDAR 스캔 스펙 (greenhouse_sim/urdf/lidar_scaled.gazebo.xacro)
LIDAR_SAMPLES = 450          # 0.8°/포인트
LIDAR_ANGLE_MIN = -np.pi     # 인덱스 0 의 각도 [rad]
LIDAR_ANGLE_MAX = np.pi
# 각도 → 인덱스: i = (angle - angle_min) / increment
LIDAR_INCREMENT = (LIDAR_ANGLE_MAX - LIDAR_ANGLE_MIN) / LIDAR_SAMPLES


def _angle_to_idx(angle_deg):
    """각도(도, 정면=0, 반시계+)를 스캔 배열 인덱스로 변환."""
    rad = np.deg2rad(angle_deg)
    return int((rad - LIDAR_ANGLE_MIN) / LIDAR_INCREMENT)


# 6개 부채꼴 구역 정의 (rl_state_space.md §2.3 — 각 60°)
# 상태 벡터에 들어가는 순서: [좌전방, 정면, 우전방, 우후방, 후면, 좌후방]
# LaserScan 각도 관례: 0=정면(+x), 양수=반시계(왼쪽)
SECTORS = [
    ('front_left', _angle_to_idx(30), _angle_to_idx(90)),     # 좌전방 [30°, 90°)
    ('front', _angle_to_idx(-30), _angle_to_idx(30)),         # 정면 [-30°, 30°)
    ('front_right', _angle_to_idx(-90), _angle_to_idx(-30)),  # 우전방 [-90°, -30°)
    ('rear_right', _angle_to_idx(-150), _angle_to_idx(-90)),  # 우후방 [-150°, -90°)
    # 후면은 ±180° 에 걸쳐 있어 두 구간으로 나눠 처리 (아래 sector_minima 참조)
    ('rear', None, None),                                     # 후면 [150°, 180°]∪[-180°, -150°)
    ('rear_left', _angle_to_idx(90), _angle_to_idx(150)),     # 좌후방 [90°, 150°)
]

# 보상 계산용 측면 거리 (통로 중앙 유지): 좌/우 ±90° 주변의 좁은 창
SIDE_LEFT_IDX = (_angle_to_idx(75), _angle_to_idx(105))      # 왼쪽 90°±15°
SIDE_RIGHT_IDX = (_angle_to_idx(-105), _angle_to_idx(-75))   # 오른쪽 -90°±15°


class ObsBuilder:
    """원시 센서 메시지를 16차원 프레임으로 정제하고 3프레임 스택을 관리한다."""

    def __init__(self, cfg, footprint=None):
        """cfg: rl_params.yaml 의 'obs' 섹션 dict.
        footprint: 'robot.footprint' dict {front, rear, half_width} [m, LiDAR 원점 기준]
                   — env_margin()(충돌 판정)에 필요. None 이면 env_margin 사용 불가.
        """
        self.max_range = cfg['lidar_max_range']        # LiDAR 결측치 치환값
        self.percentile = cfg['sector_percentile']     # 백분위수 필터 (하위 N%)
        self.ema_alpha = cfg['ema_alpha']               # EMA 계수
        self.depth_max = cfg['depth_max_range']         # 뎁스 결측치 치환값
        self.n_stack = cfg['frame_stack']                # 스택 프레임 수 (3)
        self.roi_v_start = cfg['depth_roi_v_start']      # 뎁스 범퍼 ROI 시작 행
        self.cam_height = cfg['cam_height']              # 카메라 광학중심 높이 [m]
        self.floor_margin = cfg['floor_margin']          # 바닥 차감 여유 계수 (0.9)

        self._ema_lidar = None                           # 구역별 EMA 상태 (6,)
        self._frames = deque(maxlen=self.n_stack)        # 프레임 스택 버퍼
        self._floor_thresh = None                        # 행별 바닥 깊이 임계 (지연 생성)

        # --- 풋프린트 반경 사전 계산 (충돌 판정용, 3차 수정에서 추가) ---
        # 로봇은 직사각형(반길이≠반폭)이라 "LiDAR 중심 기준 단일 반경" 임계로는
        # 전면/후면 접촉(중심에서 0.23~0.29m)을 절대 감지할 수 없다 (측면은 ~0.19m).
        # 각 빔 각도 θ 에 대해 "그 방향으로 로봇 외곽까지의 거리" r(θ) 를 미리 계산해
        # margin = (빔 측정거리) − r(θ) 로 외곽 기준 여유를 직접 판정한다.
        if footprint is not None:
            f, b, w = footprint['front'], footprint['rear'], footprint['half_width']
            theta = LIDAR_ANGLE_MIN + np.arange(LIDAR_SAMPLES) * LIDAR_INCREMENT
            cos_t, sin_t = np.cos(theta), np.sin(theta)
            # 직사각형 경계까지의 광선 거리: x방향 한계와 y방향 한계 중 먼저 닿는 쪽
            x_extent = np.where(cos_t >= 0, f, b) / np.maximum(np.abs(cos_t), 1e-6)
            y_extent = w / np.maximum(np.abs(sin_t), 1e-6)
            self._footprint_r = np.minimum(x_extent, y_extent).astype(np.float32)
        else:
            self._footprint_r = None

    # ------------------------------------------------------------------
    # LiDAR: 450 포인트 → 6구역 안전 마진 (rl_state_space.md §2.3)
    # ------------------------------------------------------------------
    def sector_minima(self, ranges):
        """LaserScan.ranges → 필터링된 6구역 거리 벡터 (m).

        전처리 순서: ① NaN/Inf/0 → max_range 클리핑 → ② 구역 분할 →
        ③ 하위 5퍼센타일 (raw 최솟값의 외란 노이즈 방지) → ④ EMA 평활화.
        """
        r = np.asarray(ranges, dtype=np.float32)
        # ① 결측치 클리핑: NaN, Inf, 0 이하(무효 반사)를 최대 거리로 치환
        r = np.where(np.isfinite(r) & (r > 0.01), r, self.max_range)
        r = np.clip(r, 0.0, self.max_range)

        out = np.zeros(6, dtype=np.float32)
        for k, (name, i0, i1) in enumerate(SECTORS):
            if name == 'rear':
                # 후면 구역은 배열 양 끝(±180° 근방)을 이어 붙임
                seg = np.concatenate([r[:_angle_to_idx(-150)],
                                      r[_angle_to_idx(150):]])
            else:
                seg = r[i0:i1]
            # ③ 백분위수 필터: 하위 N% 지점을 그 구역의 '안전 마진'으로 사용
            out[k] = np.percentile(seg, self.percentile)

        # ④ EMA: y = α·x + (1-α)·y_prev (프레임 간 튐 방지)
        if self._ema_lidar is None:
            self._ema_lidar = out
        else:
            self._ema_lidar = self.ema_alpha * out + (1 - self.ema_alpha) * self._ema_lidar
        return self._ema_lidar.copy()

    def env_margin(self, ranges):
        """충돌 판정용: 로봇 '외곽'에서 가장 가까운 장애물까지의 여유 거리 [m].

        margin_i = range_i − r(θ_i)  (r = 해당 방향 풋프린트 반경)
        의 하위 2퍼센타일(450빔 중 ~9빔 — 단일 점 노이즈 방어)을 반환한다.

        EMA 를 쓰지 않는 이유: 충돌은 지연 없이 즉각 반응해야 하는 안전 판정이라
        평활화가 오히려 위험(감지 지연). 외란 노이즈는 퍼센타일로만 거른다.

        반환값이 0 이면 외곽이 장애물에 닿은 것, collision_margin(기본 0.05) 미만이면
        에피소드 종료(reward.py).
        """
        assert self._footprint_r is not None, 'footprint 미설정 — ObsBuilder(cfg, footprint=...)'
        r = np.asarray(ranges, dtype=np.float32)
        r = np.where(np.isfinite(r) & (r > 0.01), r, self.max_range)
        margins = r - self._footprint_r
        return float(np.percentile(margins, 2))

    def side_distances(self, ranges):
        """보상용 좌/우 측면 거리 (±90°±15° 창의 하위 퍼센타일).

        통로 중앙 유지 보상(R_pose_center)의 |d_left - d_right| 계산에 쓴다.
        6구역(60° 폭)은 전방/후방이 섞여 있어 순수 측면 거리로는 ±90° 창이 더 정확.
        """
        r = np.asarray(ranges, dtype=np.float32)
        r = np.where(np.isfinite(r) & (r > 0.01), r, self.max_range)
        left = np.percentile(r[SIDE_LEFT_IDX[0]:SIDE_LEFT_IDX[1]], self.percentile)
        right = np.percentile(r[SIDE_RIGHT_IDX[0]:SIDE_RIGHT_IDX[1]], self.percentile)
        return float(left), float(right)

    # ------------------------------------------------------------------
    # 뎁스 카메라: 하단 ROI → 가상 뎁스 범퍼 3구역 (rl_state_space.md §2.2)
    # ------------------------------------------------------------------
    def depth_bumper(self, depth_msg):
        """sensor_msgs/Image(32FC1) → [좌, 중, 우] 하단 최단 거리 (m).

        2D LiDAR 평면(z≈0.23m) 아래의 3차원 사각지대를 방어하는 가상 범퍼.
        ROI 는 보수적 단순화로 화면 하반(v > cy)을 사용 (§2.5 — LiDAR 평면보다
        항상 아래쪽만 포함되므로 안전 측).

        [ 바닥 차감 (Floor Subtraction) — 검증 중 발견한 필수 보정 ]
        카메라(높이 0.168m, 수평)의 하반 시야에는 '바닥'이 항상 보인다.
        바닥까지의 깊이(맨 아랫행 기준 ~0.47m)가 min 을 지배해 범퍼가 상수가
        되어 버리므로, 각 행 v 에서 평평한 바닥이라면 보일 기대 깊이
            Z_floor(v) = fy · h_cam / (v − cy)
        보다 '충분히 가까운'(× floor_margin) 픽셀만 장애물로 취급한다.
        → 바닥은 무시되고, 바닥보다 솟아 있는 물체(파이프·대차·작물·작업자
          다리 등)만 거리로 잡힌다.

        영상 좌우 ↔ 로봇 좌우: 카메라가 전방을 보므로
        영상 왼쪽(작은 u) = 로봇 왼쪽. 따라서 열 0~213 = 좌, 214~426 = 중, 이후 = 우.
        """
        h, w = depth_msg.height, depth_msg.width
        # 32FC1 인코딩: float32 미터 단위. cv_bridge 없이 직접 변환 (의존성 절약).
        img = np.frombuffer(depth_msg.data, dtype=np.float32).reshape(h, w)

        roi = img[self.roi_v_start:, :]                          # 하반부만
        roi = np.where(np.isfinite(roi) & (roi > 0.01), roi, self.depth_max)

        # 행별 바닥 깊이 임계 사전 계산 (이미지 크기 고정이라 1회만)
        if self._floor_thresh is None:
            from rosorin_rl.geometry_utils import CAM
            rows = np.arange(self.roi_v_start, h, dtype=np.float32)
            z_floor = CAM['fy'] * self.cam_height / np.maximum(rows - CAM['cy'], 1.0)
            self._floor_thresh = np.minimum(z_floor * self.floor_margin,
                                            self.depth_max).reshape(-1, 1)

        # 바닥(및 바닥 근처) 픽셀은 max_range 로 치환 → 실제 장애물만 남김
        masked = np.where(roi < self._floor_thresh, roi, self.depth_max)

        third = w // 3
        out = np.zeros(3, dtype=np.float32)
        out[0] = masked[:, :third].min()              # 좌측 하단 최단
        out[1] = masked[:, third:2 * third].min()     # 중앙(전방) 하단 최단
        out[2] = masked[:, 2 * third:].min()          # 우측 하단 최단
        return np.clip(out, 0.0, self.depth_max)

    # ------------------------------------------------------------------
    # 프레임 조립 & 스택
    # ------------------------------------------------------------------
    def build_frame(self, target_feat, depth3, lidar6, vel3):
        """4개 그룹을 이어붙여 16차원 단일 프레임 생성.

        Args:
            target_feat: [x_norm, y_norm, d_t, theta_t] (visible 은 제외 — env 가 별도 관리)
            depth3:      depth_bumper() 결과 (3,)
            lidar6:      sector_minima() 결과 (6,)
            vel3:        [vx, vy, omega] (/odom twist)
        """
        frame = np.concatenate([
            np.asarray(target_feat, dtype=np.float32),
            np.asarray(depth3, dtype=np.float32),
            np.asarray(lidar6, dtype=np.float32),
            np.asarray(vel3, dtype=np.float32),
        ])
        assert frame.shape == (16,), f'프레임 차원 오류: {frame.shape}'
        return frame

    def stack(self, frame):
        """프레임을 버퍼에 넣고 [s_t, s_{t-1}, s_{t-2}] 48차원 벡터 반환.

        에피소드 시작 직후처럼 과거 프레임이 부족하면 현재 프레임을 복제해 채운다.
        (스택 순서: 최신 프레임이 앞 — [t, t-1, t-2])
        """
        self._frames.appendleft(frame)
        while len(self._frames) < self.n_stack:
            self._frames.append(frame.copy())
        return np.concatenate(list(self._frames))

    def reset(self):
        """에피소드 리셋: EMA 상태·프레임 스택 비우기."""
        self._ema_lidar = None
        self._frames.clear()
