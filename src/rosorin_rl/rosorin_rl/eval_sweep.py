"""eval_sweep.py — 여러 체크포인트를 한 번에 평가·랭킹해 최적 모델을 고르는 스크립트.

eval_policy.evaluate/summarize 를 재사용해, env(=sim 연결)를 1회만 띄우고
후보 체크포인트를 순차 평가한다. 결과를 비교 CSV 로 저장하고 정렬된 표를 출력한다.

설계 출처: docs/rl_design/0_project_proposal.md §6.3 (평가 지표) · 선정은 deterministic 평가 기준.
학습 중 monitor 성공률은 탐색 노이즈·커리큘럼 때문에 떨어져 보일 수 있어 신뢰하지 말 것.

[ 실행 예 ] (먼저 다른 터미널에서 rl_sim.launch.py 로 sim 구동)
  ros2 run rosorin_rl eval_sweep \
    --glob "src/rosorin_rl/models/1_main-train_sac1/sac_follow_[4-8][05]0000_steps.zip" \
    --episodes 20 --out rl_logs/eval_sweep_sac1.csv
  ros2 run rosorin_rl eval_sweep --models a.zip b.zip --episodes 20
"""

import argparse
import csv
import os
from glob import glob

import rclpy

from stable_baselines3 import PPO, SAC

from rosorin_rl.eval_policy import evaluate, summarize
from rosorin_rl.follow_env import FollowTargetEnv

# 비교 CSV / 표 컬럼 (랭킹은 success_rate desc → mean_return desc 순)
SUMMARY_FIELDS = ['model', 'n', 'success', 'success_rate', 'lost', 'coll',
                  'mean_return', 'std_return', 'mean_len',
                  'band', 'd_std', 'omega_smooth']


def _ckpt_step(path):
    """파일명에서 스텝 수를 뽑아 정렬용 키로 사용 (없으면 0)."""
    name = os.path.basename(path)
    digits = ''.join(c if c.isdigit() else ' ' for c in name).split()
    return int(digits[-1]) if digits else 0


def main():
    parser = argparse.ArgumentParser(description='여러 체크포인트 일괄 평가·랭킹')
    g = parser.add_mutually_exclusive_group(required=True)
    g.add_argument('--models', nargs='+', help='평가할 모델(.zip) 경로들')
    g.add_argument('--glob', help='모델 경로 glob 패턴 (스텝 순 정렬)')
    parser.add_argument('--episodes', type=int, default=20,
                        help='모델당 평가 에피소드 수 (성공률 안정 위해 ≥20 권장)')
    parser.add_argument('--config', default=None, help='rl_params.yaml 경로 (기본=현재 clean)')
    parser.add_argument('--algo', choices=['sac', 'ppo'], default='sac')
    parser.add_argument('--seed-base', type=int, default=0,
                        help='모델 간 동일 시드열로 페어링 (로봇 시작 지터 재현)')
    parser.add_argument('--out', default='rl_logs/eval_sweep.csv',
                        help='비교 결과 CSV 저장 경로')
    args = parser.parse_args()

    models = sorted(glob(args.glob), key=_ckpt_step) if args.glob else args.models
    if not models:
        parser.error(f'평가할 모델이 없습니다 (glob={args.glob!r})')
    print(f'[sweep] 후보 {len(models)}개 × {args.episodes}ep — '
          f'{", ".join(os.path.basename(m) for m in models)}')

    Algo = SAC if args.algo == 'sac' else PPO
    rclpy.init()
    env = FollowTargetEnv(config_path=args.config)  # env(=sim 연결) 1회 생성, 모델 간 재사용

    rows = []
    try:
        for i, path in enumerate(models):
            print(f'\n===== [{i+1}/{len(models)}] {os.path.basename(path)} =====')
            model = Algo.load(path)
            res = evaluate(env, model, args.episodes, seed_base=args.seed_base)
            row = summarize(res)
            row['model'] = os.path.basename(path)
            row['success_rate'] = row['success'] / row['n'] if row['n'] else 0.0
            rows.append(row)
    finally:
        env.close()
        rclpy.try_shutdown()

    # 랭킹: 성공률 desc → 평균 리턴 desc (동률 시 밴드↑ / d_t σ↓ / ω↓)
    rows.sort(key=lambda r: (r['success_rate'], r['mean_return'],
                             r['band'], -r['d_std'], -r['omega_smooth']),
              reverse=True)

    os.makedirs(os.path.dirname(args.out) or '.', exist_ok=True)
    with open(args.out, 'w', newline='') as f:
        w = csv.DictWriter(f, fieldnames=SUMMARY_FIELDS)
        w.writeheader()
        for r in rows:
            w.writerow({k: r[k] for k in SUMMARY_FIELDS})

    # ---------------- 랭킹 표 ----------------
    print('\n========== 모델 랭킹 (성공률 → 평균 리턴) ==========')
    print(f'{"":2}{"model":<28}{"성공률":>8}{"리턴":>11}{"길이":>8}'
          f'{"밴드":>7}{"d_tσ":>8}{"ω":>8}')
    for rank, r in enumerate(rows):
        mark = '★' if rank == 0 else ' '
        print(f'{mark} {r["model"]:<28}'
              f'{r["success"]}/{r["n"]:>3}{r["mean_return"]:>11.1f}'
              f'{r["mean_len"]:>8.0f}{r["band"]:>7.0%}'
              f'{r["d_std"]:>8.3f}{r["omega_smooth"]:>8.3f}')
    print(f'\n[sweep] 비교 CSV 저장: {args.out}')
    print('주의: 표 상위 모델은 GUI 육안 검증(제자리 진동 등 reward hacking 배제) 후 확정할 것.')


if __name__ == '__main__':
    main()
