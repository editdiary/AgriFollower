"""analyze_log.py — 학습 로그(TensorBoard)로 평가 후보 체크포인트를 사전선별하는 스크립트.

목적: 체크포인트 수십 개를 전부 sim 평가하는 건 비싸다. 학습곡선으로 "잘 됐던 구간"을
먼저 추려 소수만 deterministic 평가(eval_sweep)하게 해준다. sim·ROS 불필요(오프라인).

[ 무엇을 보나 ]
  - step별 success_rate / ep_rew_mean / 실패율(lost·env/target collision·stuck) / d_t_mean 곡선
  - 정점 구간 탐지 + 후반 열화(late-training degradation) 경고
  - reward(1차)·success(2차)로 후보 step 추천 → models-dir 의 가장 가까운 체크포인트에 매핑
    → 바로 붙여넣을 eval_sweep 명령 출력

⚠️ 학습 로그는 탐색(stochastic)·rolling window 통계라 사전선별 용도일 뿐.
   최종 줄세우기는 eval_sweep 의 deterministic 평가가 결정한다(후반 ckpt도 포함해 교차검증).

[ 실행 예 ]
  ros2 run rosorin_rl analyze_log \
    --logdir rl_logs/sac_1 \
    --models-dir src/rosorin_rl/models/1_main-train_sac1 --top 4
"""

import argparse
import csv
import glob
import os
import statistics

from tensorboard.backend.event_processing.event_accumulator import EventAccumulator

# 곡선에 띄울 스칼라 (있는 것만 출력)
CURVE_TAGS = [
    ('episode/success_rate', 'succ', '{:4.0%}'),
    ('rollout/ep_rew_mean', 'rew', '{:7.0f}'),
    ('episode/lost_rate', 'lost', '{:4.0%}'),
    ('episode/env_collision_rate', 'envC', '{:4.0%}'),
    ('episode/target_collision_rate', 'tgtC', '{:4.0%}'),
    ('episode/stuck_rate', 'stuck', '{:4.0%}'),
    ('state/d_t_mean', 'd_t', '{:5.2f}'),
]


def load_merged(logdir):
    """logdir 의 모든 tfevents 를 step순으로 병합. {tag: {step: value}} 반환."""
    files = sorted(glob.glob(os.path.join(logdir, 'events.out.tfevents.*')))
    if not files:
        raise SystemExit(f'tfevents 없음: {logdir}')
    merged = {}
    for fp in files:
        ea = EventAccumulator(fp, size_guidance={'scalars': 0})
        ea.Reload()
        for tag in ea.Tags().get('scalars', []):
            d = merged.setdefault(tag, {})
            for e in ea.Scalars(tag):
                d[e.step] = e.value  # 세션 경계 겹침 없음 → 덮어써도 무방
    print(f'[analyze] tfevents {len(files)}개 병합: '
          f'{", ".join(os.path.basename(f) for f in files)}')
    return merged


def _smooth(series, step, steps, w=2):
    """step 주변 ±w 로그포인트 평균 (rolling 노이즈 완화)."""
    i = steps.index(step)
    lo, hi = max(0, i - w), min(len(steps), i + w + 1)
    return statistics.mean(series[steps[k]] for k in range(lo, hi))


def list_checkpoints(models_dir):
    """models_dir 의 *_steps.zip 을 {step: path} 로, final 도 별도로 반환."""
    ckpts = {}
    final = None
    for p in glob.glob(os.path.join(models_dir, '*.zip')):
        name = os.path.basename(p)
        if 'replay_buffer' in name:
            continue
        if '_final' in name:
            final = p
            continue
        digits = ''.join(c if c.isdigit() else ' ' for c in name).split()
        if digits:
            ckpts[int(digits[-1])] = p
    return ckpts, final


def main():
    ap = argparse.ArgumentParser(description='학습 로그로 평가 후보 체크포인트 사전선별')
    ap.add_argument('--logdir', required=True, help='TensorBoard 로그 폴더 (예: rl_logs/sac_1)')
    ap.add_argument('--models-dir', required=True, help='체크포인트(.zip) 폴더')
    ap.add_argument('--top', type=int, default=4, help='추천 후보 수 (최신 ckpt 1개 포함)')
    ap.add_argument('--min-step', type=int, default=0, help='이 step 미만은 무시(미수렴 초반 숨김)')
    ap.add_argument('--spacing', type=int, default=20000, help='후보 간 최소 step 간격')
    ap.add_argument('--episodes', type=int, default=30, help='출력할 eval_sweep 명령의 에피소드 수')
    ap.add_argument('--out', default=None, help='step별 곡선을 CSV로 저장(선택)')
    args = ap.parse_args()

    merged = load_merged(args.logdir)
    if 'rollout/ep_rew_mean' not in merged:
        raise SystemExit('rollout/ep_rew_mean 스칼라가 없음 — 로그 폴더를 확인하세요.')
    rew = merged['rollout/ep_rew_mean']
    succ = merged.get('episode/success_rate', {})
    steps = [s for s in sorted(rew) if s >= args.min_step]

    # ---------------- 곡선 출력 ----------------
    present = [(t, lbl, fmt) for t, lbl, fmt in CURVE_TAGS if t in merged]
    header = ' step    | ' + ' | '.join(f'{lbl:>5}' for _, lbl, _ in present)
    print('\n========== 학습곡선 (step별, 균등 샘플) ==========')
    print(header)
    sample = steps[::max(1, len(steps) // 30)]
    for st in sample:
        cells = []
        for tag, _, fmt in present:
            v = merged[tag].get(st)
            cells.append(fmt.format(v) if v is not None else '   - ')
        print(f' {st:8d}| ' + ' | '.join(f'{c:>5}' for c in cells))

    if args.out:
        os.makedirs(os.path.dirname(args.out) or '.', exist_ok=True)
        with open(args.out, 'w', newline='') as f:
            w = csv.writer(f)
            w.writerow(['step'] + [lbl for _, lbl, _ in present])
            for st in steps:
                w.writerow([st] + [merged[t].get(st, '') for t, _, _ in present])
        print(f'[analyze] 곡선 CSV 저장: {args.out}')

    # ---------------- 진단: 정점 + 후반 열화 ----------------
    peak = max(steps, key=lambda s: _smooth(rew, s, steps))
    peak_rew = _smooth(rew, peak, steps)
    last = steps[-1]
    last_rew = _smooth(rew, last, steps)
    print('\n========== 진단 ==========')
    print(f'reward 정점  : step ~{peak} (smoothed rew {peak_rew:.0f}'
          + (f', succ {succ.get(peak, 0):.0%}' if succ else '') + ')')
    print(f'최신 구간    : step ~{last} (smoothed rew {last_rew:.0f}'
          + (f', succ {succ.get(last, 0):.0%}' if succ else '') + ')')
    if last_rew < peak_rew * 0.95:
        print(f'⚠️ 후반 열화 감지: 최신 reward 가 정점 대비 {(1-last_rew/peak_rew):.0%} 낮음 '
              f'→ 최신/final 이 최적이 아닐 수 있음. 정점 구간 ckpt 를 평가할 것.')

    # ---------------- 후보 추천 (reward 1차 → success 2차, 간격 보장) ----------------
    ranked = sorted(steps, key=lambda s: (_smooth(rew, s, steps),
                                          _smooth(succ, s, steps) if succ else 0),
                    reverse=True)
    picks = []
    for s in ranked:
        if len(picks) >= max(1, args.top - 1):
            break
        if all(abs(s - p) >= args.spacing for p in picks):
            picks.append(s)

    ckpts, final = list_checkpoints(args.models_dir)
    if not ckpts:
        raise SystemExit(f'체크포인트(.zip) 없음: {args.models_dir}')
    latest_step = max(ckpts)  # 최신 ckpt = 후반 열화 교차확인용으로 항상 포함

    def nearest(step):
        k = min(ckpts, key=lambda c: abs(c - step))
        return k, ckpts[k]

    chosen = {}  # ckpt_step -> (path, 사유)
    for s in sorted(picks):
        k, path = nearest(s)
        chosen.setdefault(k, (path, f'정점 구간(log step ~{s})'))
    if latest_step not in chosen:
        chosen[latest_step] = (ckpts[latest_step], '최신(후반 열화 교차확인)')

    print('\n========== 추천 후보 (deterministic 평가 대상) ==========')
    for k in sorted(chosen):
        path, why = chosen[k]
        print(f'  {os.path.basename(path):<32} ← {why}')
    if final:
        print(f'  (참고) final = {os.path.basename(final)} (대개 최신 step과 동일)')

    paths = [chosen[k][0] for k in sorted(chosen)]
    print('\n다음 명령으로 이 후보들만 deterministic 평가하세요 (sim 구동 필요):')
    print('  ros2 run rosorin_rl eval_sweep \\')
    print('    --models ' + ' '.join(paths) + ' \\')
    print(f'    --episodes {args.episodes} --out rl_logs/eval_sweep_sac1.csv')
    print('\n⚠️ 로그는 사전선별일 뿐 — 최종 선정은 위 eval_sweep(deterministic) 결과로.')


if __name__ == '__main__':
    main()
