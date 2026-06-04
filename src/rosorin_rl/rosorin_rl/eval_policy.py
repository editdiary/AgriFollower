"""eval_policy.py — 학습된 정책 평가 스크립트.

설계 출처: docs/rl_design/0_project_proposal.md §6.3 (평가 3지표)
  ① 학습 수렴 속도 → TensorBoard 곡선 (train_sac.py 쪽)
  ② 주행 부드러움  → 에피소드 중 ω(각속도) 변화량 통계 (여기서 측정)
  ③ 노이즈 강건성  → noise_pct 를 올린 config 로 재평가 시 실패율 변화 (여기서 측정)

[ 실행 예 ]
  ros2 run rosorin_rl eval_policy --model models/sac_follow_final.zip
  ros2 run rosorin_rl eval_policy --model ... --episodes 10
  # 노이즈 강건성 평가: noise_pct 를 0.3 으로 올린 yaml 을 만들어 --config 로 지정
"""

import argparse

import numpy as np
import rclpy

from stable_baselines3 import PPO, SAC

from rosorin_rl.follow_env import FollowTargetEnv


def main():
    parser = argparse.ArgumentParser(description='학습된 추종 정책 평가')
    parser.add_argument('--model', required=True, help='모델(.zip) 경로')
    parser.add_argument('--episodes', type=int, default=5, help='평가 에피소드 수')
    parser.add_argument('--config', default=None, help='rl_params.yaml 경로')
    parser.add_argument('--algo', choices=['sac', 'ppo'], default='sac')
    args = parser.parse_args()

    rclpy.init()
    env = FollowTargetEnv(config_path=args.config)
    model = (SAC if args.algo == 'sac' else PPO).load(args.model)

    results = []  # 에피소드별 (리턴, 길이, 종료사유, ω변화량 std)
    try:
        for ep in range(args.episodes):
            obs, _ = env.reset()
            ep_ret, ep_len = 0.0, 0
            omegas = []      # 각 스텝의 ω 명령 (부드러움 지표용)
            terminal = None

            while True:
                # deterministic=True: 탐색 노이즈 없이 정책 평균 행동 사용 (평가 표준)
                action, _ = model.predict(obs, deterministic=True)
                obs, r, terminated, truncated, info = env.step(action)
                ep_ret += r
                ep_len += 1
                omegas.append(float(action[2]))
                if terminated or truncated:
                    terminal = info.get('terminal')
                    break

            # 주행 부드러움: 연속 스텝 간 ω 변화량의 표준편차 (작을수록 부드러움)
            smooth = float(np.std(np.diff(omegas))) if len(omegas) > 1 else 0.0
            results.append((ep_ret, ep_len, terminal, smooth))
            print(f'[EP {ep+1}/{args.episodes}] 리턴 {ep_ret:8.1f} | '
                  f'길이 {ep_len:4d} | 종료 {terminal} | ω부드러움 {smooth:.3f}')
    finally:
        env.close()
        rclpy.try_shutdown()

    # ---------------- 요약 ----------------
    rets = [r[0] for r in results]
    lens = [r[1] for r in results]
    succ = sum(1 for r in results if r[2] == 'success')
    lost = sum(1 for r in results if r[2] == 'lost')
    coll = sum(1 for r in results if r[2] in ('env_collision', 'target_collision'))
    print('\n========== 평가 요약 ==========')
    print(f'평균 리턴   : {np.mean(rets):8.1f} ± {np.std(rets):.1f}')
    print(f'평균 길이   : {np.mean(lens):8.1f} 스텝')
    print(f'성공률      : {succ}/{len(results)}')
    print(f'타겟 이탈   : {lost}/{len(results)}  | 충돌: {coll}/{len(results)}')
    print(f'ω 부드러움  : {np.mean([r[3] for r in results]):.3f} (낮을수록 부드러움)')


if __name__ == '__main__':
    main()
