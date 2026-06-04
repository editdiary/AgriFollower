# 📖 [코드 가이드] rosorin_rl — 강화학습 파이프라인 이해·검증·튜닝

> 대상: `src/rosorin_rl/` (2026-06 작성). 설계의 단일 출처는 `docs/rl_design/` —
> 이 문서는 "그 설계가 코드 어디에 어떻게 구현됐고, 무엇을 확인하고, 어떤 값을 만져야 하는지"를 다룬다.

## 1. 아키텍처 맵 — 데이터가 흐르는 길

```
[Ignition Gazebo Fortress]
  ├─ /scan (LiDAR 360°) ─────────────┐
  ├─ /depth_cam/depth_image ─────────┤  ┌──────────────────────────────┐
  ├─ /odom (속도) ───────────────────┼─▶│ follow_env.py  FollowTargetEnv│
  ├─ /world/.../dynamic_pose/info ───┤  │  _collect_obs():             │
  │    (로봇 ground-truth pose)      │  │   obs_pipeline.py 로 16D 정제 │
  │                                  │  │   → 3프레임 스택 = 48D       │◀── SB3 (SAC/PPO)
  │  [target_controller_node]        │  │  step(action):               │     train_sac.py
  │   set_pose 로 원기둥 이동(20Hz)  │  │   [-1,1]³ → Twist 발행      │
  │   └─ /worker/pose ──┐            │  │  reward.py 로 보상·종료 판정 │
  │                     ▼            │  └──────┬───────────────────────┘
  │  [target_feature_node]           │         │ /controller/cmd_vel
  │   ground truth → 마커 특징 역산  │         ▼
  │   + 가우시안 노이즈(3%)          │  [MecanumDrive 플러그인 → 로봇 구동]
  │   └─ /target/features ───────────┘
  └─ /world/.../set_pose (서비스) ◀── reset() 텔레포트 + 작업자 이동
```

**코드 읽는 순서 (추천):**

| 순서 | 파일 | 설계 문서 대응 | 핵심 |
|---|---|---|---|
| ① | `config/rl_params.yaml` | 전체 수치 | 모든 튜너블이 여기 모임 |
| ② | `geometry_utils.py` | `rl_state_space.md` §2.1·§2.5 | ground truth → 마커 특징 역산 (핀홀 투영) |
| ③ | `obs_pipeline.py` | `rl_state_space.md` §2.2~§3 | 16D 정제 + 3프레임 스택 = 48D |
| ④ | `reward.py` | `rl_reward_function.md` §2~§5 | 보상 3항 + 모드 전환 + 종료 5종 |
| ⑤ | `follow_env.py` | `0_project_proposal.md` §4.3 | Gym ⇄ ROS 브리지 (reset/step) |
| ⑥ | `target_controller_node.py` | `rl_train_senarioes.md` §2 | 작업자 걸음 (전략 패턴 — 시나리오 확장점) |
| ⑦ | `target_feature_node.py` | roadmap 2단계 (b) | Sim-to-Real 교체 지점 |
| ⑧ | `train_sac.py` / `eval_policy.py` | proposal §6 | SB3 학습/평가 |

**Sim-to-Real 때 바뀌는 것은 ⑦ 하나뿐이다** — `/target/features` 발행자를
실물 비전(AprilTag 검출) 노드로 갈아끼우면 환경·정책 코드는 불변 (roadmap 5단계).

## 2. 검증 체크리스트 — 직접 눈으로 확인할 것

> 아래 1~6은 코드 작성 시점에 자동 검증을 통과했지만, **환경을 다시 만지면(월드 재생성,
> 파라미터 변경, 패키지 재빌드) 다시 돌려보는 게 안전하다.** 각 단계는 독립 실행 가능.

모든 터미널에서 먼저: `source install/setup.bash && source .typerc`

| # | 확인 항목 | 명령 | 기대 결과 | 실패 시 의심 지점 |
|---|---|---|---|---|
| 1 | 시뮬+타겟 기동 | `ros2 launch rosorin_rl rl_sim.launch.py` | noVNC에서 주황 원기둥이 통로를 0.25m/s로 왕복. **서 있어야 함**(누우면 비정상) | 런치 로그의 `WorkerController 시작` 누락 → 빌드/소싱 |
| 2 | 타겟 특징 | `ros2 topic echo /target/features` | `data: [x_norm, y_norm, d_t, θ_t, visible]`. 작업자가 멀어지면 d_t 증가, ±3% 지터 | d_t가 음수/고정 → `/worker/pose`·`dynamic_pose/info` 브리지 확인 |
| 3 | 리셋(텔레포트) | `ros2 service call /world/greenhouse_world/set_pose ros_gz_interfaces/srv/SetEntityPose "{entity: {name: robot, type: 2}, pose: {position: {x: 0.0, y: 0.0, z: 0.05}, orientation: {w: 1.0}}}"` | `success: true` + GUI에서 로봇 순간이동 | rl_bridge 노드 미기동 |
| 4 | Env 롤아웃 | 아래 "최소 롤아웃 스크립트" | obs shape (48,), 유한값, 보상 출력 | 센서 타임아웃 에러 메시지에 누락 토픽 명시됨 |
| 5 | 종료 조건 | 롤아웃에서 정지([0,0,0]) 유지 | ~50스텝 후 `lost` 종료 (d_t>3.0, r≈-50) | `episode.target_lost_frames`·`target.lost_distance` |
| 6 | 충돌 종료 | 롤아웃에서 [-0.6,0,0] (후진) 유지 | 문 접촉 부근에서 `env_collision` (r≈-30) | `thresholds.collision_margin`·`robot.footprint` (§5 함정 9 참조) |
| 7 | 학습 sanity | `ros2 run rosorin_rl train_sac --timesteps 1500 --ckpt-freq 500` | 진행바, `models/sac_follow_*_steps.zip` 생성, 무크래시 | torch/CUDA — §5 함정 참조 |
| 8 | TensorBoard | `tensorboard --logdir ~/rosorin_sim_ws/rl_logs` | `rollout/ep_rew_mean` 곡선 표시 | logdir 경로 |

**최소 롤아웃 스크립트** (시뮬이 떠 있는 상태에서):
```python
import rclpy, numpy as np
from rosorin_rl.follow_env import FollowTargetEnv
rclpy.init()
env = FollowTargetEnv()
obs, _ = env.reset()
print(obs.shape, np.isfinite(obs).all())          # (48,) True
for i in range(50):
    obs, r, term, trunc, info = env.step(env.action_space.sample() * 0.5)
    print(f'{i}: r={r:+.2f} d_t={info["d_t"]:.2f} terminal={info["terminal"]}')
    if term: break
env.close()
```

**관측 벡터 48D 의 인덱스 맵** (디버깅 시 필수 — 첫 16개가 최신 프레임 t):
```
[0:4]   타겟: x_norm, y_norm, d_t, θ_t
[4:7]   뎁스범퍼: 좌, 중, 우 (바닥 차감 후 최단거리)
[7:13]  LiDAR: 좌전방, 정면, 우전방, 우후방, 후면, 좌후방
[13:16] 속도: vx, vy, ω
[16:32] 같은 구성의 t-1 프레임, [32:48] t-2 프레임
```

## 3. 튜닝 파라미터 표 (`config/rl_params.yaml`)

> `--symlink-install` 빌드라 **yaml 수정은 재빌드 없이 다음 실행부터 반영**된다.

### 보상 계수 — "이 증상이면 이 값을"

> [2차 조정 이력] 14k 스텝 정체 진단(ep_len~50 고정 = lost 반복) 후
> "추적↑ / 벽 페널티↓ / 사람 충돌 유지" 방향으로 재균형: w1 1.0→2.0, w2 1.0→0.5,
> alpha 3.0→1.5, k_approach 신규(5.0), terminal.env_collision -100→-30,
> target.speed 0.25→0.05(커리큘럼 0단계).
>
> [4차 조정 이력] 18k 스텝 진단(d_t_mean 1.4 수렴·in_aisle_ratio 0.29 급락 =
> "입구 배회" 국소최적): α 1.5→3.0 복원 — α=1.5는 1.4m 배회로도 +0.86/step 을 줘서
> 통로 진입(충돌 위험)보다 대기가 유리했음. 2차 때와 달리 원거리 기울기는
> k_approach 가 담당하므로 신호 소실은 재발하지 않는다. 같은 라운드에 타겟
> 차폐(occlusion) 판정도 추가(함정 #12).

| 키 | 현재값 | 의미 | 증상 → 조정 |
|---|---|---|---|
| `w1` | 2.0 | 추종 보상 비중 | 로봇이 추종을 포기하고 안전 거리만 유지 → w1↑ |
| `w2` | 0.5 | 안전 페널티 비중 | 벽을 자주 긁음 → w2↑ / 벽이 무서워 위축(통로 진입 회피) → w2↓ |
| `w3` | 0.5 | 자세·중앙 유지 비중 | 통로에서 비스듬히 주행 → w3↑ / 코너에서 굳음 → w3↓ |
| `alpha` | 3.0 | 추종 가우시안 폭(클수록 뾰족) | 0.65m 근처에서 출렁임 → α↓ / 멀리서 배회(원거리에서도 보상 후함) → α↑ |
| `k_approach` | 5.0 | **접근 shaping** `k·(d_prev−d_t)` — 거리 불문 "가까워지면 +" 즉각 신호. d<d_opt(과근접)에선 0 처리 | 멀리서 타겟을 아예 못 찾아감 → ↑ / 과속 돌진 → ↓ |
| `beta_f` | 30.0 | 전방 페널티 강도 | 전방 추돌 잦음 → ↑ / 타겟 근접(0.65m)을 페널티로 회피 → ↓ |
| `beta_s` | 80.0 | 측면 페널티 강도 | 측면 긁힘 잦음 → ↑ |
| `gamma` | 0.5 | 헤딩 정렬 페널티 | U턴 학습(시나리오5) 때 회전을 못 함 → ↓ |
| `zeta` | 0.5 | 중앙 유지 페널티 | 한쪽 벽에 붙어 다님 → ↑ |
| `terminal.env_collision` | -30 | 벽 충돌 1회성 페널티 | 벽을 무시하고 긁고 다님 → 절대값↑ |
| `terminal.target_collision` | -100 | **사람 충돌 — 안전상 크게 유지** | 낮추지 말 것 |

**스케일 균형 원리:** 정상 주행 시 스텝 보상은 `R_track`(0~1)이 지배하는 +0.6~1.0,
페널티는 임계 침범 시에만 O(0.1~3), 터미널은 ±50~100. 한 항이 다른 항을 10배 이상
압도하면 그 항만 최적화하는 꼼수가 나온다 — 컴포넌트별 값은 `info['r_track']`,
`info['r_safety']`, `info['r_pose']` 로 항상 확인 가능.

### 임계값·에피소드

| 키 | 초기값 | 비고 |
|---|---|---|
| `thresholds.front` / `side` | 0.3 / 0.05 | 페널티 시작 거리 (설계 문서 §3.2 고정값) |
| `thresholds.collision_margin` | **0.05** | 환경 충돌 판정: 로봇 **외곽** 기준 여유(`env_margin`)가 이 미만이면 종료. 노이즈 퍼센타일 꼬리(~2cm) 때문에 실효 발화는 외곽 5~8cm 전 (보수적·안전 측) |
| `robot.footprint` | front 0.23 / rear 0.27 / half_width 0.19 | **직사각형 풋프린트** (LiDAR 원점 기준 외곽 거리, 4방향 텔레포트 실측 검증). 로봇 형상 바꾸면 함께 갱신 |
| `thresholds.aisle_mode` | 1.5 | 통로/교차로 모드 전환. 교차로에서도 `in_aisle=True`면 ↑ |
| `episode.max_steps` | 1200 | 성공 기준 생존 시간 (10Hz×120s) |
| `episode.warmup_steps` | 20 | 리셋 직후 종료 판정 유예 (오판 방지) |
| `target.speed` | 0.05 | 작업자 속도. **커리큘럼 난이도 조절의 1순위 손잡이** — 0.05(현재, 0단계) → 0.1 → 0.25(시나리오 1 본래 값)로 수렴 확인 후 단계 상향. ⚠️ launch 의 controller `speed` 파라미터와 함께 수정 |
| `obs.noise_pct` | 0.03 | 도메인 랜덤화. **강건성 평가(proposal §6.3) 때 0.3까지 올려 실험** |
| `obs.ema_alpha` | 0.4 | LiDAR 평활화. ↓하면 부드럽지만 반응 느려 충돌 판정도 늦어짐 |
| `obs.floor_margin` | 0.9 | 뎁스범퍼 바닥 차감 여유. 바닥을 장애물로 오인하면 ↓ (§5 참조) |

### SAC 하이퍼파라미터 (막히면 이 순서로)

1. `learning_starts`(1000): 초기 랜덤 수집 — 에피소드가 너무 일찍 끝나면 ↑
2. `buffer_size`(30만): 메모리 부족하면 ↓ (10만도 동작)
3. `learning_rate`(3e-4): 학습 곡선이 발산하면 1e-4 로
4. `net_arch`([256,256]): 48차원 입력엔 충분 — 건드릴 일 적음

## 4. 학습 모니터링 — TensorBoard 읽는 법

```bash
tensorboard --logdir ~/rosorin_sim_ws/rl_logs   # http://localhost:6006
```

| 그래프 | 정상 패턴 | 이상 패턴 → 대응 |
|---|---|---|
| `rollout/ep_rew_mean` | 초기 음수(-50~-100, 조기 종료) → 점진 상승 → 성공 시 +1000 이상 (1200스텝×~1 + 100) | **장기 정체**: 보상 컴포넌트 로깅으로 어느 항이 0인지 확인. **하락 후 붕괴**: learning_rate↓ |
| `rollout/ep_len_mean` | 상승 (오래 생존) | 100 부근 고정 → stuck 종료 남발: `episode.stuck_window` 또는 보상 균형 점검 |
| `train/ent_coef` (SAC) | 서서히 하강 (탐색→수렴) | 0 으로 급락 → 조기 수렴(꼼수 가능성), 거동을 영상으로 확인 |
| `train/critic_loss` | 진동하며 완만 | 폭발적 증가 → 보상 스케일 문제 (터미널 값이 너무 큼 등) |

### 진단 지표 (RLMetricsCallback — 2차 추가, `rosorin_rl/callbacks.py`)

| 지표 | 의미 | 읽는 법 |
|---|---|---|
| `episode/success_rate` 등 5종 | 최근 100 에피소드의 종료 사유 비율 | **"왜 죽는가"의 답.** lost_rate↑ = 추적 신호 부족 / env_collision_rate↑ = 안전 항 점검 / success_rate 우상향 = 승기 |
| `reward/r_track_mean` `r_approach_mean` `r_safety_mean` `r_pose_mean` | 에피소드 평균 보상 컴포넌트 | 어느 항이 끌고/막는지 분해. 한 항만 비대 = 꼼수 의심 |
| `action/abs_ax(ay,aw)_mean` | 평균 행동 사용량 | `abs_ay`(게걸음) 상승 시점 = 매카넘 활용 시작 (proposal §6.3-②) |
| `state/d_t_mean` | 평균 타겟 거리 | 0.65 로 수렴하면 이상적 |
| `state/in_aisle_ratio` | 통로 내부 비율 | 0 에 머물면 "통로 진입 회피" 학습 의심 → w2↓ 또는 env_collision 완화 |

### 에피소드 CSV (`rl_logs/monitor.csv`)
`Monitor(filename=...)` 가 에피소드별 `(r=리턴, l=길이, t=경과초, terminal=종료사유)` 를 기록.
pandas 자유 분석: `pd.read_csv('rl_logs/monitor.csv', skiprows=1)`

**꼼수(Reward Hacking) 의심 거동** — 곡선만 좋고 실제 거동이 이상할 때:
- 제자리 진동으로 추종 보상만 수집 → `stuck` 종료가 잡아주는지 확인 (`terminal` 분포 로깅)
- 타겟을 시야에 두고 후진만 반복 → 시나리오 1에선 d_t>3.0 lost 로 종료됨 ✓
- 평가는 항상 `eval_policy` 의 **deterministic 모드 + 거동 육안 확인**과 병행할 것.

## 5. 주의사항·함정 (검증 중 실제로 발견된 것 포함)

1. **매카넘 y축 반전 (실측 버그 보정):** 벤더 MecanumDrive 플러그인이 `linear.y` 를
   REP-103(+y=좌측)과 **반대로** 해석한다 (vx·ω 는 정상). `follow_env.step()` 이 발행
   직전 부호를 뒤집어 보정한다. **실물 로봇 포팅 시 이 보정(부호 반전)을 제거할 것.**
   → `follow_env.py` step() 의 `cmd.linear.y = -a[1]·v_max` 주석 참조.
2. **뎁스범퍼 바닥 차감:** 수평 카메라의 하반 ROI에는 바닥이 항상 보여(맨 아랫행 ~0.47m)
   min 이 상수가 되는 문제가 있었다. 행별 기대 바닥 깊이보다 `floor_margin`(0.9)배
   가까운 픽셀만 장애물로 취급한다. 로봇이 기울거나(피치) 바닥이 평평하지 않으면
   바닥을 장애물로 오인할 수 있음 → `floor_margin` 을 0.8 로 낮춰 둔감하게.
3. **타겟은 set_pose 키네마틱 구동:** velocity-control 플러그인은 Fortress 에서 link
   gravity off 가 무시돼 원기둥이 넘어지는 문제로 폐기. 현재는 static 모델을
   20Hz set_pose 로 움직인다(틱당 1.25cm — LiDAR 에 연속으로 보임). 작업자의 위치
   단일 출처는 **컨트롤러의 내부 적분값**(`/worker/pose`)이다.
4. **sim time 기준 동작:** env 의 step 대기·노드 타이머 모두 sim time. RTF(실시간 배율)가
   0.5 면 학습 wall-clock 도 2배 — 거동은 동일하다. 처리량이 필요하면
   `ros2 launch rosorin_rl rl_sim.launch.py headless:=true` (GUI 없이 센서 렌더 유지).
5. **재빌드가 필요한 경우:** `--symlink-install` 이라 **파이썬 코드·yaml 수정은 재빌드
   불필요.** `setup.py`(엔트리포인트)·launch 파일 추가·새 파일 생성 시에만
   `colcon build --symlink-install --packages-select rosorin_rl`.
6. **torch 버전:** 이 컨테이너 드라이버는 CUDA 12.8 — 기본 pip torch(cu13x)는 GPU 인식
   실패. `torch==2.8.0+cu128` (`--index-url https://download.pytorch.org/whl/cu128`) 사용.
   numpy 는 cv_bridge 호환을 위해 `<2` 유지.
7. **리셋 직후 1프레임:** 텔레포트 직전에 렌더된 stale 센서 프레임 방지를 위해 reset()이
   "0.3s 안정화 → 버퍼 비움 → 신규 수신 대기" 순서로 동작한다. 그래도 작업자는 리셋
   즉시 걷기 시작하므로 첫 관측의 d_t 는 1.4m 정각이 아닐 수 있다 (정상).
8. **학습 중 시뮬 GUI 를 닫지 말 것:** env 는 토픽이 끊기면 step 타임아웃 경고를 내며
   멈춘다. 시뮬 재시작 시 학습도 재시작해야 한다 (`--resume models/...zip` 으로 이어가기).
   2차 수정부터 체크포인트에 **Replay Buffer(.pkl, ~120MB)** 도 함께 저장되어 resume 시
   자동 복원된다(경험 손실 없음). 단, **보상 설계를 바꾼 뒤에는 resume 하지 말 것** —
   버퍼·가치함수가 옛 보상 스케일로 학습된 상태라 오염된다. 설계 변경 후엔 새로 시작.
9. **충돌 판정은 직사각형 풋프린트 마진 방식 (3차 수정):** 처음엔 "min(전구역) <
   단일 반경" 방식이었는데, 로봇이 직사각형(반길이 0.27 ≠ 반폭 0.19)이라 **전/후면
   접촉(중심에서 0.23~0.29m)이 단일 임계 0.20에 절대 도달하지 못해 미감지**되는
   버그가 있었다(사용자가 GUI에서 "문에 닿는데 인식 안 됨"으로 발견). 현재는 빔
   각도별 풋프린트 반경 r(θ)를 빼서 **외곽 기준 여유**(`env_margin`)로 판정 —
   4방향 근접 텔레포트로 실측 검증 완료. 참고: **입구 문은 visual 전용**(collision
   없음)이라 물리적으론 통과되지만 gpu_lidar(렌더 기반)는 문짝을 보므로 판정엔 잡힘.
   충돌 처리 설계: **즉시 리셋 + R_safety 의 단계적 사전 페널티** 채택 (접촉 허용·
   누적 임계 방식은 벽 긁기 학습/실물 위험 때문에 기각 — 사용자 확정).
   충돌 워밍업은 `episode.collision_warmup: 3`(stale 프레임 방어용)으로 lost/stuck
   의 20스텝과 분리 — 길면 시작 직후 문에 박는 동작이 마스킹된다.
10. **스폰/리셋 좌표 (3차 확정):** 로봇 x=-0.40(입구 문 앞 — LiDAR 노이즈 꼬리까지
   감안한 후방 시작 마진 ~0.15m 실측 확보), 작업자 x=0.6(통로 입구). 초기 d_t≈1.1m.
   좌표를 바꾸면 config·rl_sim.launch.py·spawn_scaled_model.launch.py 3곳을 함께 수정.
11. **AprilTag 시각 패널 (2차 추가):** 원기둥 ±x 면, 월드 z=0.35m(= GT marker_height와
   동일)에 공식 tag36h11 ID 0 텍스처 부착 — `worlds_models/tag36_11_00000.png`.
   **시각 전용**이며 검출은 여전히 GT 역산. 실물 비전 검증 시 이 태그를 그대로 쓰면 된다.
12. **타겟 가시성에 차폐(occlusion) 판정 포함 (4차 수정):** 원래 가시성은 "카메라
   전방 + 화각 ±30°"만 검사해서 **작물 벽 너머의 타겟도 visible=1 로 발행되는
   "벽 투시" 버그**가 있었다(사용자가 GUI에서 "다른 통로에서도 타겟 위치를 아는 듯한
   행동"으로 발견). 실물 AprilTag 검출에선 불가능한 정보라 Sim-to-Real 갭이기도 했다.
   현재는 `geometry_utils.OCCLUDERS`(작물 줄 3개의 2D AABB — `gen_greenhouse_world.py`
   레이아웃에서 도출)와 카메라→타겟 선분의 교차로 차폐를 판정한다(잎 높이 1.4m >
   마커 0.35m 라 2D 로 충분). **온실 레이아웃을 바꾸면 OCCLUDERS 도 함께 갱신할 것.**
   미검출 프레임 처리는 last-known 유지 + env 의 "연속 15스텝 → lost" — d_t 를 0 으로
   리셋하면 사람 충돌(d_t<0.4) 종료가 오발하므로 0 리셋 방식은 쓰지 않는다(설계 결정).
   검증: 다른 통로 텔레포트 → visible=0 → step 21 lost 발화 / 정상 추종 50스텝 오발 0.
13. **학습 중 자잘한 떨림은 SAC 탐색 샘플링이 주원인:** 학습 중에는 정책이 확률적으로
   행동을 샘플링하므로(엔트로피 탐색) 미세 진동이 정상이다. 정책의 실제 매끄러움은
   `eval_policy`(deterministic=True) 로 평가할 것. 평가에서도 떨면 액션 변화율 페널티
   `−c·|a_t−a_{t−1}|²` 추가를 검토(이번 라운드 보류 — 사용자 확정).

## 6. 다음 단계 (roadmap 연동)

- **시나리오 2~5 추가:** `target_controller_node.py` 의 `GaitStrategy` 상속 + `STRATEGIES`
  등록만으로 확장. 커리큘럼 비율은 `rl_train_senarioes.md` §3.
- **본 학습:** `ros2 run rosorin_rl train_sac --timesteps 200000` (헤드리스 권장).
  PPO 비교군: `--algo ppo`. 평가: `eval_policy` 3지표 (proposal §6.3).
- **랜덤 시작 위치(도메인 랜덤화 확장):** 현재 리셋은 고정 위치 — `follow_env.reset()` 의
  `options`/`config` 로 로봇·타겟 시작점을 랜덤화하면 일반화에 유리.
