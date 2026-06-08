"""reward.py — 보상 함수 및 에피소드 종료 조건.

설계 출처: docs/rl_design/rl_reward_function.md (수식의 단일 출처)
계수 수치: src/rosorin_rl/config/rl_params.yaml (튜닝 출발점)

[ 전체 구조 (§2 + 2차·5차·6차 조정) ]
    R_t = w1·R_tracking + R_approach + w2·R_safety + w3_eff·R_pose_center
          + R_smooth + R_gaze

  - R_tracking   : 목표 거리(0.87m) 유지 — 가우시안 형태 연속 보상 (§3.1)
  - R_approach   : 접근 shaping k·(d_prev−d_t) — 원거리에서도 추적 기울기 제공
                   (2차 조정에서 추가 — 학습 정체 진단 후. compute() 내 주석 참조)
  - R_safety     : 전방/측면 임계치 이원화, 클리핑된 2차 페널티 (§3.2)
                   ⚠️ 임계값은 LiDAR 'raw 거리' 기준 — 풋프린트(반폭 0.19m 등)를
                   더한 값으로 잡아야 충돌 전에 실제로 발동한다 (5차 조정에서 보정)
  - R_pose_center: 통로 정렬 + 중앙 유지 (§3.3) — 통로 진입에 따라 선형 램프로
                   활성 (§4, 5차 조정: 하드 스위치 → 램프. 입구 불연속 벌점 제거)
                   (7차 조정: yaw 항은 d_side_min 램프, 중앙 항은 d_side_max 게이트로
                   분리 — 입구에서 한쪽만 트인 상태의 |Δ좌우| 포화 벌점 제거)
  - R_smooth     : 행동 변화 페널티 −k·‖a_t−a_{t−1}‖² (5차 조정: 잔떨림 비용 부여)
  - R_gaze       : 타겟 주시 −η·|θ_t| (6차 조정) — lost 절벽(FOV 이탈 15프레임
                   → −50)의 dense 선행 신호. 타겟을 화면 중앙에 유지하도록 유도해
                   불필요한 ω 사용("고개 휙 돌림")을 억제. 비가시 시 0.

[ 종료 조건 (§5) ]  — 스칼라 값은 rl_params.yaml terminal 섹션이 단일 출처
  성공(+) / 환경충돌(−) / 타겟충돌(−) / 타겟이탈(−) / 정체(−)
"""

import math
from collections import deque


class RewardCalculator:
    """스텝마다 보상·종료 여부를 계산한다. 에피소드 단위 상태(정체 감지 창 등)를 보유."""

    def __init__(self, cfg):
        """cfg: rl_params.yaml 전체 dict."""
        rw = cfg['reward']
        self.w1, self.w2, self.w3 = rw['w1'], rw['w2'], rw['w3']
        self.alpha = rw['alpha']
        self.beta_f, self.beta_s = rw['beta_f'], rw['beta_s']
        self.gamma_, self.zeta = rw['gamma'], rw['zeta']
        self.k_approach = rw['k_approach']   # 접근 shaping 계수 (2차 조정에서 추가)
        self.k_smooth = rw['k_smooth']       # 행동 변화 페널티 계수 (5차 조정에서 추가)
        self.eta = rw['eta']                 # 타겟 주시 페널티 계수 (6차 조정에서 추가)

        th = cfg['thresholds']
        self.front_th = th['front']            # 전방 안전 임계 (raw 거리 기준)
        self.side_th = th['side']              # 측면 안전 임계 (raw 거리 기준)
        self.aisle_mode_th = th['aisle_mode']  # 통로/교차로 모드 전환 1.5m
        self.aisle_ramp = th['aisle_ramp']     # 모드 전환 선형 램프 폭 (5차 조정)
        self.center_gate_th = th['center_gate']      # 중앙유지 활성 d_side_max 임계 (7차 조정)
        self.center_gate_ramp = th['center_gate_ramp']  # 중앙유지 선형 램프 폭 (7차 조정)
        self.collision_margin = th['collision_margin']  # 외곽 기준 충돌 여유 임계

        tg = cfg['target']
        self.d_opt = tg['opt_distance']            # 최적 추종 거리 0.87m
        self.d_tgt_collision = tg['collision_distance']  # 타겟 충돌 0.4m
        self.d_lost = tg['lost_distance']          # 타겟 이탈 3.0m

        ep = cfg['episode']
        self.max_steps = ep['max_steps']
        self.lost_frames_th = ep['target_lost_frames']
        self.stuck_window = ep['stuck_window']
        self.warmup = ep['warmup_steps']
        self.collision_warmup = ep['collision_warmup']  # 충돌 판정 전용 (짧음)

        self.term = cfg['terminal']  # 터미널 보상 스칼라들

        self.reset()

    def reset(self):
        """에피소드 시작 시 내부 상태 초기화."""
        self.reward_history = deque(maxlen=self.stuck_window)  # 정체 감지용
        self.lost_count = 0                                    # 연속 미검출 스텝 수
        self.d_prev = None                                     # 접근 shaping 용 직전 거리
        self.a_prev = None                                     # 행동 변화 페널티용 직전 행동

    # ------------------------------------------------------------------
    def compute(self, *, d_t, visible, theta_t, d_front_merged, d_left, d_right,
                yaw_err, env_margin, step_idx, action=None):
        """한 스텝의 보상과 종료 여부 계산.

        Args:
            d_t:            타겟 실측 거리 [m] (/target/features — 노이즈 포함)
            visible:        이번 스텝에 타겟이 검출됐는지 (bool)
            theta_t:        타겟 방위각 [rad] (카메라 광축 기준 — R_gaze 용)
            d_front_merged: 전방 센서 퓨전 거리 = min(LiDAR 정면, 뎁스범퍼 중앙) [m] (§3.2)
            d_left/d_right: 좌/우 측면 최소 거리 [m] (±90° 창)
            yaw_err:        |로봇 헤딩 - 통로 방향| [rad] (§3.3)
            env_margin:     로봇 '외곽' 기준 최소 장애물 여유 [m]
                            (obs_pipeline.env_margin — 직사각형 풋프린트 반영, 3차 수정)
            step_idx:       현재 에피소드 내 스텝 번호 (0부터)
            action:         이번 스텝의 정규화 행동 [ax, ay, aω] (행동 변화 페널티용)

        Returns:
            (reward, terminated, info)
            - info['terminal']: 종료 사유 문자열 (종료 시), 아니면 None
            - info['r_track' / 'r_safety' / 'r_pose']: 컴포넌트별 값 (디버깅/튜닝용)
        """
        # ===== 1) 하위 보상 계산 =====

        # --- R_tracking (§3.1): 목표 거리 0.87m 와의 오차에 대한 가우시안 ---
        # d=0.87 일 때 1.0(최대), 멀어질수록 0 으로 부드럽게 감소.
        r_track = math.exp(-self.alpha * (d_t - self.d_opt) ** 2)

        # --- 접근 shaping (2차 조정에서 추가): k·(d_prev − d_t) ---
        # 가우시안은 타겟이 멀어지면(d>2m) 기울기가 사실상 0이라 "어느 방향이
        # 나아지는지"를 알려주지 못한다. 이 항은 거리와 무관하게 "직전 스텝보다
        # 가까워졌으면 +, 멀어졌으면 −" 신호를 즉각 제공한다.
        # potential-based shaping(Φ=−k·d)이라 이론상 최적 정책을 바꾸지 않는다.
        # 단, 이미 d < d_opt(너무 가까움)일 때 '더 접근'을 부추기면 안 되므로
        # 목표 거리 안쪽에서는 0 처리한다.
        if self.d_prev is None or d_t < self.d_opt:
            r_approach = 0.0
        else:
            r_approach = self.k_approach * (self.d_prev - d_t)
        self.d_prev = d_t

        # --- R_safety (§3.2): 임계치 안으로 들어온 만큼만 2차 페널티 (클리핑) ---
        # 1/x 발산형 대신 max(0, 임계-거리)^2 — Exploding Gradient 방지.
        r_safe_front = -self.beta_f * max(0.0, self.front_th - d_front_merged) ** 2
        d_side = min(d_left, d_right)
        r_safe_side = -self.beta_s * max(0.0, self.side_th - d_side) ** 2
        r_safety = r_safe_front + r_safe_side

        # --- R_pose_center (§3.3, 7차 조정: yaw/중앙 게이트 분리) ---
        # [7차 진단] 5차 램프는 d_side_min 이 입구에서 '서서히' 줄어든다고 가정했지만,
        # 실제로는 잎벽이 측면 창(±90°±15°)에 들어오는 순간 ~2.1m → ~0.4m 로
        # 불연속 점프한다 (입구 밖 측면 = 외벽/트인 공간). 그 전환 구간에서 한쪽
        # 창만 잎벽을 보고 반대쪽은 아직 트여 있어 |좌-우| 가 1.0 캡까지 포화 →
        # 입구에서 w3·ζ·1.0 의 구조적 진입 벌점("입구 관문")이 발생했다 (sac_2
        # eval 에서 진입 순간 자세 탐색 거동으로 실측).
        #
        # yaw 정렬: 기존 d_side_min 램프 유지 (한쪽 벽만 보여도 정렬은 의미 있음)
        d_side_min = min(d_left, d_right)
        d_side_max = max(d_left, d_right)
        in_aisle = d_side_min < self.aisle_mode_th       # (로깅/진단용 불리언)
        yaw_ramp = min(max((self.aisle_mode_th - d_side_min) / self.aisle_ramp,
                           0.0), 1.0)
        # 중앙 유지: 양쪽 벽이 '모두' 가까울 때(=진짜 통로 내부)만 활성.
        # 입구는 한쪽만 트여 d_side_max ≫ 1 → 게이트 0 (입구 |Δ좌우| 관문 제거)
        center_ramp = min(max(
            (self.center_gate_th - d_side_max) / self.center_gate_ramp, 0.0), 1.0)

        r_yaw = -self.gamma_ * abs(yaw_err)
        # |좌-우| 상한 1.0m 클립(5차)은 게이트가 꺼지기 전 잔여 구간 방어용으로 유지
        r_center = -self.zeta * min(abs(d_left - d_right), 1.0)

        # --- R_smooth (5차 조정): 행동 변화 페널티 −k·‖a_t − a_{t−1}‖² ---
        # 잔떨림(스텝마다 행동 반전)에 비용을 부여해 매끄러운 주행 유도.
        # 에피소드 첫 스텝(직전 행동 없음)은 0.
        if action is None or self.a_prev is None:
            r_smooth = 0.0
        else:
            r_smooth = -self.k_smooth * sum(
                (a - b) ** 2 for a, b in zip(action, self.a_prev))
        if action is not None:
            self.a_prev = list(action)

        # --- R_gaze (6차 조정): 타겟 주시 −η·|θ_t| ---
        # lost 절벽(FOV 이탈 15프레임 → −50)으로 가기 전의 dense 선행 신호.
        # 타겟을 화면 중앙에 유지할수록 0에 가깝고, FOV 가장자리(±30°≈0.52rad)
        # 에서 −η·0.52. 비가시 시 θ_t 는 last-known(stale)이므로 0 처리.
        r_gaze = -self.eta * abs(theta_t) if visible else 0.0

        reward = (self.w1 * r_track + r_approach
                  + self.w2 * r_safety
                  + self.w3 * (yaw_ramp * r_yaw + center_ramp * r_center)
                  + r_smooth + r_gaze)

        # ===== 2) 종료 조건 판정 (§5) — 우선순위: 충돌 > 이탈 > 정체 > 성공 =====
        terminal = None

        # 타겟 미검출 카운트 갱신 (검출되면 리셋)
        self.lost_count = 0 if visible else self.lost_count + 1

        in_warmup = step_idx < self.warmup  # 리셋 직후 정체/이탈 판정 유예 구간

        # 충돌 유예는 별도(더 짧게): stale 프레임 방어만 하면 되고,
        # 길게 잡으면 시작 직후의 진짜 충돌(문에 후진 등)이 마스킹된다.
        if env_margin < self.collision_margin and step_idx >= self.collision_warmup:
            terminal = 'env_collision'       # 작물 벽/구조물 충돌 (외곽 여유 소진)
        elif d_t < self.d_tgt_collision and visible:
            terminal = 'target_collision'    # 작업자 안전 위협 (0.4m 미만)
        elif (d_t > self.d_lost or self.lost_count >= self.lost_frames_th) \
                and not in_warmup:
            terminal = 'lost'                # 거리 초과 이탈 또는 연속 미검출
        elif step_idx + 1 >= self.max_steps:
            terminal = 'success'             # 최대 스텝 생존 = 성공!

        # 정체 감지: 최근 100스텝 보상 평균이 음수 (벽 비비기 등 꼼수 상태)
        self.reward_history.append(reward)
        if terminal is None and not in_warmup \
                and len(self.reward_history) == self.stuck_window \
                and sum(self.reward_history) / self.stuck_window < 0.0:
            terminal = 'stuck'

        # ===== 3) 터미널 보상 가산 =====
        if terminal is not None:
            reward += self.term[terminal]

        info = {
            'terminal': terminal,
            'r_track': r_track,
            'r_approach': r_approach,
            'r_safety': r_safety,
            # 램프 반영 후 실효 기여분 (w3 가중 전) — 7차: 항별 게이트 분리 반영
            'r_pose': yaw_ramp * r_yaw + center_ramp * r_center,
            'r_smooth': r_smooth,
            'r_gaze': r_gaze,
            'in_aisle': in_aisle,
            'd_t': d_t,
            'd_front_merged': d_front_merged,
            # 디버깅/튜닝용 원시값 (docs/rl_code_guide.md 검증 절차에서 사용)
            'd_left': d_left,
            'd_right': d_right,
            'yaw_err': yaw_err,
            'env_margin': env_margin,
        }
        return reward, terminal is not None, info
