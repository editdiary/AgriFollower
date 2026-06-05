"""callbacks.py — 학습 분석용 커스텀 TensorBoard 로깅 콜백.

SB3 의 기본 로깅(rollout/ep_rew_mean 등)은 "얼마나 잘하는지"만 보여주고
"왜 못하는지/어떻게 변하는지"를 보여주지 않는다. 이 콜백은 env 의 info dict 와
행동 벡터에서 진단 정보를 모아 TensorBoard 에 추가 기록한다.

[ 기록 지표 ]  (모두 SB3 로그 주기에 맞춰 dump 됨)
- episode/  : 최근 100 에피소드의 종료 사유 비율
    success_rate · lost_rate · env_collision_rate · target_collision_rate · stuck_rate
    → "왜 죽는가"가 한눈에 보임. 예: lost_rate 만 높다 = 추적 신호 부족,
      env_collision_rate 상승 = 안전 가중치 점검.
- reward/   : 에피소드 평균 보상 컴포넌트 (r_track / r_approach / r_safety / r_pose)
    → 어느 항이 학습을 끌고/막고 있는지 분해 진단. 꼼수(한 항만 최적화) 탐지.
- action/   : 에피소드 평균 |ax|, |ay|, |aω|
    → 매카넘 게걸음(ay)을 실제로 쓰기 시작하는 시점 관찰 (proposal §6.3 평가 2지표).
- state/    : 에피소드 평균 타겟 거리 d_t, 통로 내부(in_aisle) 비율
    → 로봇이 통로에 진입하는지, 목표 거리(0.65m)에 수렴하는지 추적.

사용법: train_sac.py 에서 callback=[CheckpointCallback, RLMetricsCallback()] 로 전달.
"""

from collections import deque

import numpy as np

from stable_baselines3.common.callbacks import BaseCallback

# 종료 사유 종류 (reward.py 의 terminal 문자열과 일치해야 함)
TERMINAL_KINDS = ('success', 'lost', 'env_collision', 'target_collision', 'stuck')


class RLMetricsCallback(BaseCallback):
    """info dict·행동 벡터 기반 진단 지표를 TensorBoard 에 기록."""

    def __init__(self, window=100, verbose=0):
        """window: 종료 사유 비율을 계산할 최근 에피소드 수."""
        super().__init__(verbose)
        self.terminals = deque(maxlen=window)   # 최근 에피소드 종료 사유
        self._ep_acc = None                     # 진행 중 에피소드의 누적 버퍼

    def _reset_ep_acc(self):
        self._ep_acc = {
            'r_track': [], 'r_approach': [], 'r_safety': [], 'r_pose': [],
            'r_smooth': [], 'r_gaze': [],
            'abs_ax': [], 'abs_ay': [], 'abs_aw': [],
            'd_t': [], 'in_aisle': [],
        }

    def _on_training_start(self):
        self._reset_ep_acc()

    def _on_step(self) -> bool:
        # self.locals: SB3 가 학습 루프의 지역변수를 노출 — infos(env info), actions
        info = self.locals['infos'][0]          # 단일 env (index 0)
        action = np.asarray(self.locals['actions']).reshape(-1)

        # --- 스텝 단위 누적 ---
        acc = self._ep_acc
        for k in ('r_track', 'r_approach', 'r_safety', 'r_pose', 'r_smooth',
                  'r_gaze', 'd_t'):
            if k in info:
                acc[k].append(float(info[k]))
        if 'in_aisle' in info:
            acc['in_aisle'].append(1.0 if info['in_aisle'] else 0.0)
        acc['abs_ax'].append(abs(float(action[0])))
        acc['abs_ay'].append(abs(float(action[1])))
        acc['abs_aw'].append(abs(float(action[2])))

        # --- 에피소드 종료 시: 평균 내서 기록 + 종료 사유 누적 ---
        if self.locals['dones'][0]:
            term = info.get('terminal')
            if term is not None:
                self.terminals.append(term)

            # 종료 사유 비율 (최근 window 에피소드)
            n = max(len(self.terminals), 1)
            for kind in TERMINAL_KINDS:
                rate = sum(1 for t in self.terminals if t == kind) / n
                self.logger.record(f'episode/{kind}_rate', rate)

            # 보상 컴포넌트·행동·상태의 에피소드 평균
            names = {
                'reward/r_track_mean': 'r_track',
                'reward/r_approach_mean': 'r_approach',
                'reward/r_safety_mean': 'r_safety',
                'reward/r_pose_mean': 'r_pose',
                'reward/r_smooth_mean': 'r_smooth',
                'reward/r_gaze_mean': 'r_gaze',
                'action/abs_ax_mean': 'abs_ax',
                'action/abs_ay_mean': 'abs_ay',     # 매카넘 게걸음 사용량
                'action/abs_aw_mean': 'abs_aw',
                'state/d_t_mean': 'd_t',
                'state/in_aisle_ratio': 'in_aisle',
            }
            for tb_key, acc_key in names.items():
                if acc[acc_key]:
                    self.logger.record(tb_key, float(np.mean(acc[acc_key])))

            self._reset_ep_acc()    # 다음 에피소드 누적 시작

        return True  # False 를 반환하면 학습이 중단됨 — 항상 True
