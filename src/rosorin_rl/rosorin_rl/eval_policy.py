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
import csv

import numpy as np
import rclpy

from stable_baselines3 import PPO, SAC

from rosorin_rl.follow_env import FollowTargetEnv


# --dump-csv: 스텝별 계측 — 보상 항/센서 raw 거리/action (임계값 보정·구간별 진단용)
CSV_FIELDS = ['episode', 'step', 'd_t', 'd_front_merged', 'd_left', 'd_right',
              'env_margin', 'yaw_err', 'in_aisle',
              'r_track', 'r_approach', 'r_safety', 'r_pose', 'r_smooth',
              'r_gaze', 'ax', 'ay', 'aw', 'reward', 'terminal']


def evaluate(env, model, episodes, csv_writer=None, seed_base=None):
    """모델 1개를 deterministic 으로 평가하고 에피소드별 결과 리스트를 반환.

    반환: [(리턴, 길이, 종료사유, ω변화량 std, 밴드점유, d_t std), ...]
    seed_base 가 주어지면 ep 번째 에피소드를 reset(seed=seed_base+ep) 으로 시작 —
    로봇 시작 지터를 모델 간 동일하게(best-effort 페어링) 맞춘다.
    """
    results = []
    for ep in range(episodes):
        if seed_base is not None:
            obs, _ = env.reset(seed=seed_base + ep)
        else:
            obs, _ = env.reset()
        ep_ret, ep_len = 0.0, 0
        omegas = []      # 각 스텝의 ω 명령 (부드러움 지표용)
        d_ts = []        # 각 스텝의 타겟 거리 (거리 유지 지표용 — 6차 추가)
        terminal = None

        while True:
            # deterministic=True: 탐색 노이즈 없이 정책 평균 행동 사용 (평가 표준)
            action, _ = model.predict(obs, deterministic=True)
            obs, r, terminated, truncated, info = env.step(action)
            ep_ret += r
            ep_len += 1
            omegas.append(float(action[2]))
            if 'd_t' in info:
                d_ts.append(float(info['d_t']))
            if csv_writer is not None:
                csv_writer.writerow({
                    'episode': ep + 1, 'step': ep_len,
                    'd_t': info.get('d_t'),
                    'd_front_merged': info.get('d_front_merged'),
                    'd_left': info.get('d_left'),
                    'd_right': info.get('d_right'),
                    'env_margin': info.get('env_margin'),
                    'yaw_err': info.get('yaw_err'),
                    'in_aisle': int(bool(info.get('in_aisle'))),
                    'r_track': info.get('r_track'),
                    'r_approach': info.get('r_approach'),
                    'r_safety': info.get('r_safety'),
                    'r_pose': info.get('r_pose'),
                    'r_smooth': info.get('r_smooth'),
                    'r_gaze': info.get('r_gaze'),
                    'ax': float(action[0]), 'ay': float(action[1]),
                    'aw': float(action[2]),
                    'reward': float(r), 'terminal': info.get('terminal'),
                })
            if terminated or truncated:
                terminal = info.get('terminal')
                break

        # 주행 부드러움: 연속 스텝 간 ω 변화량의 표준편차 (작을수록 부드러움)
        smooth = float(np.std(np.diff(omegas))) if len(omegas) > 1 else 0.0
        # 거리 유지 지표 (6차 추가): 밴드 점유율 = |d_t − d_opt| < 0.15m 비율
        d_opt = env.unwrapped.cfg['target']['opt_distance']
        band = (float(np.mean(np.abs(np.asarray(d_ts) - d_opt) < 0.15))
                if d_ts else 0.0)
        d_std = float(np.std(d_ts)) if d_ts else 0.0
        results.append((ep_ret, ep_len, terminal, smooth, band, d_std))
        print(f'[EP {ep+1}/{episodes}] 리턴 {ep_ret:8.1f} | '
              f'길이 {ep_len:4d} | 종료 {terminal} | ω부드러움 {smooth:.3f} | '
              f'밴드점유 {band:.0%} | d_t σ {d_std:.3f}')
    return results


def summarize(results):
    """evaluate() 결과 리스트를 집계해 요약 dict 반환."""
    rets = [r[0] for r in results]
    lens = [r[1] for r in results]
    return {
        'n': len(results),
        'mean_return': float(np.mean(rets)),
        'std_return': float(np.std(rets)),
        'mean_len': float(np.mean(lens)),
        'success': sum(1 for r in results if r[2] == 'success'),
        'lost': sum(1 for r in results if r[2] == 'lost'),
        'coll': sum(1 for r in results
                    if r[2] in ('env_collision', 'target_collision')),
        'omega_smooth': float(np.mean([r[3] for r in results])),
        'band': float(np.mean([r[4] for r in results])),
        'd_std': float(np.mean([r[5] for r in results])),
    }


def main():
    parser = argparse.ArgumentParser(description='학습된 추종 정책 평가')
    parser.add_argument('--model', required=True, help='모델(.zip) 경로')
    parser.add_argument('--episodes', type=int, default=5, help='평가 에피소드 수')
    parser.add_argument('--config', default=None, help='rl_params.yaml 경로')
    parser.add_argument('--algo', choices=['sac', 'ppo'], default='sac')
    parser.add_argument('--dump-csv', default=None, metavar='PATH',
                        help='스텝별 계측값(보상 항·센서 거리·action)을 CSV로 저장')
    args = parser.parse_args()

    rclpy.init()
    env = FollowTargetEnv(config_path=args.config)
    model = (SAC if args.algo == 'sac' else PPO).load(args.model)

    csv_file = csv_writer = None
    if args.dump_csv:
        csv_file = open(args.dump_csv, 'w', newline='')
        csv_writer = csv.DictWriter(csv_file, fieldnames=CSV_FIELDS)
        csv_writer.writeheader()

    try:
        results = evaluate(env, model, args.episodes, csv_writer=csv_writer)
    finally:
        if csv_file is not None:
            csv_file.close()
            print(f'[dump-csv] 스텝별 계측 저장: {args.dump_csv}')
        env.close()
        rclpy.try_shutdown()

    # ---------------- 요약 ----------------
    s = summarize(results)
    print('\n========== 평가 요약 ==========')
    print(f'평균 리턴   : {s["mean_return"]:8.1f} ± {s["std_return"]:.1f}')
    print(f'평균 길이   : {s["mean_len"]:8.1f} 스텝')
    print(f'성공률      : {s["success"]}/{s["n"]}')
    print(f'타겟 이탈   : {s["lost"]}/{s["n"]}  | 충돌: {s["coll"]}/{s["n"]}')
    print(f'ω 부드러움  : {s["omega_smooth"]:.3f} (낮을수록 부드러움)')
    print(f'밴드 점유율 : {s["band"]:.0%} '
          f'(|d_t−d_opt|<0.15m 인 스텝 비율 — 거리 유지 품질)')
    print(f'd_t 표준편차: {s["d_std"]:.3f} m (낮을수록 일정한 거리)')


if __name__ == '__main__':
    main()
