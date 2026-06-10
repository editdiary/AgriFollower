"""train_sac.py — SAC(1순위) / PPO(베이스라인) 학습 스크립트.

설계 출처: docs/rl_design/0_project_proposal.md §6 (알고리즘 선정·평가 계획)
하이퍼파라미터: config/rl_params.yaml 의 sac / ppo 섹션

[ 사전 조건 ]
  별도 터미널에서 시뮬이 떠 있어야 한다:
    ros2 launch rosorin_rl rl_sim.launch.py
  (학습 처리량을 높이려면 headless:=true 로 GUI 없이 실행)

[ 실행 예 ]
  ros2 run rosorin_rl train_sac                          # SAC, 10만 스텝
  ros2 run rosorin_rl train_sac --algo ppo               # PPO 베이스라인
  ros2 run rosorin_rl train_sac --timesteps 5000         # 짧은 검증 런
  ros2 run rosorin_rl train_sac --warm-start models/sac_2/sac_follow_final.zip
                                # 보상 변경 후 재학습: 가중치만 이식, 버퍼·런 초기화

[ 모니터링 ]
  tensorboard --logdir ~/rosorin_sim_ws/rl_logs
  → rollout/ep_rew_mean (에피소드 평균 보상) 이 핵심 학습 곡선.
    SAC vs PPO 수렴 속도 비교는 proposal §6.3 평가 1지표.
"""

import argparse
import os

import rclpy

from stable_baselines3 import PPO, SAC
from stable_baselines3.common.callbacks import CheckpointCallback
from stable_baselines3.common.monitor import Monitor
from stable_baselines3.common.utils import get_latest_run_id

from rosorin_rl.callbacks import RLMetricsCallback
from rosorin_rl.follow_env import FollowTargetEnv


def main():
    parser = argparse.ArgumentParser(description='작업자 추종 정책 학습 (SAC/PPO)')
    parser.add_argument('--algo', choices=['sac', 'ppo'], default='sac',
                        help='알고리즘 (기본 sac — proposal §6.1)')
    parser.add_argument('--timesteps', type=int, default=100_000,
                        help='총 학습 스텝 수')
    parser.add_argument('--config', default=None,
                        help='rl_params.yaml 경로 (기본: 패키지 share)')
    parser.add_argument('--logdir', default=os.path.expanduser('~/rosorin_sim_ws/rl_logs'),
                        help='TensorBoard 로그 디렉토리')
    parser.add_argument('--modeldir', default=os.path.expanduser('~/rosorin_sim_ws/src/rosorin_rl/models'),
                        help='체크포인트/최종 모델 저장 디렉토리')
    parser.add_argument('--ckpt-freq', type=int, default=10_000,
                        help='체크포인트 저장 주기 (스텝)')
    parser.add_argument('--resume', default=None,
                        help='이어서 학습할 모델(.zip) 경로')
    parser.add_argument('--warm-start', default=None, metavar='ZIP',
                        help='정책 가중치만 이식해 "새 런"으로 시작 (.zip 경로). '
                             'resume 과 달리 Replay Buffer·옵티마이저·ent_coef·스텝 '
                             '카운터·런 번호를 모두 초기화한다. 보상 설계를 바꾼 뒤 '
                             '재학습할 때 사용 — 이전 버퍼의 보상값은 무효이므로 '
                             'resume 하면 안 된다 (rl_code_guide.md §5 참조).')
    args = parser.parse_args()
    if args.resume and args.warm_start:
        parser.error('--resume 과 --warm-start 는 동시에 쓸 수 없습니다.')

    os.makedirs(args.logdir, exist_ok=True)
    os.makedirs(args.modeldir, exist_ok=True)

    # ---------------- 환경 생성 ----------------
    rclpy.init()
    env = FollowTargetEnv(config_path=args.config)
    cfg = env.cfg
    # Monitor: 에피소드 리턴/길이를 SB3 로거(ep_rew_mean 등)에 기록하는 래퍼.
    # - filename: 에피소드별 (리턴, 길이, 경과시각) + 종료사유를 CSV 로도 남김
    #   → 학습 후 pandas 로 자유 분석 (rl_logs/monitor_{algo}_{N}.monitor.csv)
    #   확장자 없이 넘긴다 — SB3 가 Monitor.EXT('monitor.csv')를 자동으로 붙인다.
    # - info_keywords: 마지막 info 에서 함께 기록할 키
    # 파일명은 TB 런 번호(sac_N)와 일치시킨다. 새 런이면 +1, resume(reset_num_timesteps=False)
    # 이면 기존 번호 재사용(SB3 configure_logger 와 동일 규칙).
    # override_existing: 새 런이면 True(새 CSV 생성), resume 이면 False(같은 파일에 헤더 없이
    # 이어붙임 — SB3 ResultsWriter append 모드). 과거엔 기본 truncate 로 열려 resume 시 이전 런
    # CSV 가 덮어써졌다.
    run_id = get_latest_run_id(args.logdir, args.algo) + (0 if args.resume else 1)
    monitor_path = os.path.join(args.logdir, f'monitor_{args.algo}_{run_id}')
    env = Monitor(env, filename=monitor_path,
                  info_keywords=('terminal',),
                  override_existing=args.resume is None)
    print(f'에피소드 CSV: {monitor_path}.{Monitor.EXT}')

    # ---------------- 모델 생성 ----------------
    # device: GPU(cuda) 사용 가능하면 자동 선택. 48차원 MLP 라 CPU 도 충분히 빠름.
    if args.algo == 'sac':
        hp = cfg['sac']
        if args.resume:
            model = SAC.load(args.resume, env=env, tensorboard_log=args.logdir)
            # Replay Buffer 복원: 체크포인트와 함께 저장된 경험 버퍼(.pkl)가 있으면
            # 불러온다. 없으면 빈 버퍼로 시작(재개 직후 잠시 출렁일 수 있음).
            # 파일명 규칙 후보 2가지:
            #  - CheckpointCallback: sac_follow_50000_steps.zip
            #      → sac_follow_replay_buffer_50000_steps.pkl
            #  - 최종 저장(아래 finally): sac_follow_final.zip
            #      → sac_follow_final_replay_buffer.pkl
            import re
            candidates = [
                re.sub(r'_(\d+)_steps\.zip$', r'_replay_buffer_\1_steps.pkl',
                       args.resume),
                args.resume.replace('.zip', '') + '_replay_buffer.pkl',
            ]
            for buf_path in candidates:
                if buf_path != args.resume and os.path.exists(buf_path):
                    model.load_replay_buffer(buf_path)
                    print(f'Replay Buffer 복원: {buf_path} '
                          f'({model.replay_buffer.size()}개 경험)')
                    break
            else:
                print('⚠️ Replay Buffer 파일 없음 — 빈 버퍼로 재개 '
                      '(이전 체크포인트가 버퍼 미포함 형식)')
        else:
            model = SAC(
                'MlpPolicy', env,
                learning_rate=hp['learning_rate'],
                buffer_size=hp['buffer_size'],      # Replay Buffer (off-policy 핵심)
                batch_size=hp['batch_size'],
                gamma=hp['gamma'],                   # 할인율
                tau=hp['tau'],                       # 타겟망 soft update
                learning_starts=hp['learning_starts'],  # 초기 랜덤 탐색 구간
                train_freq=hp['train_freq'],
                gradient_steps=hp['gradient_steps'],
                ent_coef=hp['ent_coef'],             # 'auto' = 엔트로피 자동 조정 (최대 엔트로피 RL)
                policy_kwargs=dict(net_arch=list(hp['net_arch'])),
                tensorboard_log=args.logdir,
                verbose=1,
            )
    else:  # ppo
        hp = cfg['ppo']
        if args.resume:
            model = PPO.load(args.resume, env=env, tensorboard_log=args.logdir)
        else:
            model = PPO(
                'MlpPolicy', env,
                learning_rate=hp['learning_rate'],
                n_steps=hp['n_steps'],               # rollout 길이 (on-policy)
                batch_size=hp['batch_size'],
                gamma=hp['gamma'],
                gae_lambda=hp['gae_lambda'],
                clip_range=hp['clip_range'],         # 신뢰 영역 클리핑 (PPO 핵심)
                policy_kwargs=dict(net_arch=list(hp['net_arch'])),
                tensorboard_log=args.logdir,
                verbose=1,
            )

    # ---------------- 웜스타트 (7차): 정책 가중치만 이식 ----------------
    # SB3 SAC/PPO 의 policy 객체는 actor·critic·critic_target 을 모두 포함하므로
    # load_state_dict 한 번이면 네트워크 전체가 이식된다. 버퍼/옵티마이저/ent_coef
    # 는 위에서 새로 만든 것을 유지 → 바뀐 보상 체계에서 깨끗하게 재적응.
    if args.warm_start:
        algo_cls = SAC if args.algo == 'sac' else PPO
        warm = algo_cls.load(args.warm_start, device=model.device)
        model.policy.load_state_dict(warm.policy.state_dict())
        del warm
        print(f'웜스타트: {args.warm_start} 의 정책 가중치 이식 '
              f'(버퍼/옵티마이저/스텝 카운터는 초기화, 새 런 {args.algo}_{run_id})')

    # ---------------- 학습 ----------------
    ckpt_cb = CheckpointCallback(
        save_freq=args.ckpt_freq,
        save_path=args.modeldir,
        name_prefix=f'{args.algo}_follow',
        save_replay_buffer=(args.algo == 'sac'),  # 버퍼(~120MB)도 저장 → resume 시 복원
    )
    # 진단 지표 (종료사유 분포·보상 컴포넌트·행동 사용량·상태 통계 → TensorBoard)
    metrics_cb = RLMetricsCallback(window=100)

    try:
        model.learn(
            total_timesteps=args.timesteps,
            callback=[ckpt_cb, metrics_cb],
            tb_log_name=args.algo,
            progress_bar=True,
            reset_num_timesteps=args.resume is None,
        )
    except KeyboardInterrupt:
        print('\n학습 중단 (Ctrl+C) — 현재까지의 모델을 저장합니다.')
    finally:
        final_path = os.path.join(args.modeldir, f'{args.algo}_follow_final')
        model.save(final_path)
        if args.algo == 'sac':
            model.save_replay_buffer(final_path + '_replay_buffer.pkl')
        print(f'모델 저장 완료: {final_path}.zip')
        env.close()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
