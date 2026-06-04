# 로드맵 — ROSOrin 시뮬 기반 강화학습 (작업자 추종)

목표: Ignition Gazebo에서 ROSOrin(매카넘)을 온실에 띄우고, **수확 작업자를 일정 거리로 추종하면서
주변 장애물에 맞춰 최적 자세를 잡는 주행 정책**을 강화학습으로 학습한다.
단순 통로 주행/내비게이션이 아니라 **타겟(작업자) 추종**이 핵심 과제다.

> RL 설계: 개요·알고리즘·Sim-to-Real은 **`rl_design/0_project_proposal.md`**, 구체 수치·수식의 단일 출처는
> **세부 노트**(`rl_design/rl_state_space.md`·`rl_reward_function.md`·`rl_train_senarioes.md`).
> 이 로드맵은 그 설계를 **현재 ws(Ignition Fortress + 기존 토픽/온실)에서 실행하는 단계**로 풀어 쓴 것이다.
> 토픽/제어 인터페이스는 `README.md`·`docs/environment.md`의 표를 단일 출처로 사용.

## 현재 상태 (2026-06)
- ✅ 제조사 공식 절차로 Ignition Gazebo 시뮬 세팅 성공 (`docs/setup_process.md`).
  - `ros2 launch robot_gazebo worlds.launch.py` / `room_worlds.launch.py` 정상 동작, teleop 확인, GPU 렌더 활용.
- ✅ 커스텀 토마토 온실 world를 Ignition용으로 이식 (`greenhouse_sim`).
  - 옛 ws(Gazebo Classic)의 절차적 생성기를 Ignition SDF(인라인 PBR 머티리얼) 출력으로 개조.
- ✅ **1단계 환경/토픽 검증 완료.**
  - `ros2 launch greenhouse_sim greenhouse.launch.py`로 온실 + ROSOrin 스폰을 GUI/noVNC에서 시각 확인.
  - RViz2로 `/scan`·`/scan/points`·`/imu`·`/odom`·`/joint_states`·`/depth_cam/*` 발행 및 `/controller/cmd_vel` 구독 확인.
  - teleop(매카넘 vx/vy/ω)로 통로 주행 거동 확인.
- ✅ 로봇 몸체 스케일업(균일 `S=1.83` 포크 `greenhouse_sim/urdf/`)을 온실 런치에 적용.
  - 이 과정에서 만난 **depth_cam 대각선(45/135/225/315°) 회색 렌더 버그**(센서 링크 mesh까지 스케일한 것이 원인)를 해결 → `docs/troubleshooting.md`.
- ✅ **RGB-D 통합 + LiDAR 360° 확장.** 카메라를 `rgbd_camera`로 교체(`/depth_cam/{image,depth_image,points,camera_info}`),
  LiDAR를 실물 MS200과 동일한 360°(450 samples)로 확장. 카메라 intrinsic·Cam–LiDAR extrinsic은 `docs/environment.md` 참조.
- ✅ RL 상태/보상 설계를 RGB-D 기반으로 개정(16차원×3프레임=48차원, 뎁스 범퍼·전방 센서 퓨전) → `rl_design/` 세부 노트.

## MDP 요약 (수치 출처: `rl_design/rl_state_space.md`·`rl_reward_function.md` — 상세는 그쪽 참조)
- **상태(48차원):** 단일 프레임 16차원 = RGB-D 마커 타겟 특징 `[x_norm, y_norm, d_t, θ_t]`(4) + RGB-D 하단 뎁스 범퍼 `[d_depth_{left,center,right}_low]`(3) + LiDAR 6구역 최솟값(6) + 로봇 속도 `[vx, vy, ω]`(3). 최근 3프레임(`t, t-1, t-2`) 스택 → 48차원 1D 벡터.
- **행동(3-DOF 연속):** `[ax, ay, aω] ∈ [-1,1]³` → `vx=ax·Vmax`, `vy=ay·Vmax`(Vmax≈0.5 m/s), `ω=aω·Wmax`(Wmax≈1.0 rad/s). 매카넘 홀로노믹 활용. (출처: proposal §5.2)
- **보상:** `R = w1·R_tracking + w2·R_safety + w3·R_pose_center` (목표거리 d_opt=0.65m 가우시안 / 전방 LiDAR+뎁스범퍼 min 퓨전·측면 이원화, 클리핑 2차 페널티 / 통로 정렬·중앙 유지) + 통로 내·외부 모드 전환.
- **종료:** 성공(최대스텝 생존, +100) / 환경충돌(-100) / 타겟충돌(d_t<0.4m, -100) / 타겟이탈(d_t>3.0m 또는 시야 밖, -50) / 정체감지(-50).

## 1단계 — 온실 환경 검증 (완료)
- [x] 온실 + 로봇 스폰 시각 확인, 전 토픽 RViz2 검증, teleop 주행 확인 (위 "현재 상태" 참조).
- [ ] (필요 시) 스폰 위치·작물 간격이 로봇 폭/타겟 추종에 적절한지 점검, `gen_greenhouse_world.py` 파라미터(`AISLE_WIDTH` 등) 조정 후 재생성.

## 2단계 — 시뮬 선결 과제 (RL 학습에 필요한 환경/센서 보강)
현재 sim에는 RL에 필요한 요소 일부가 없다. **제조사 `src/simulations/`는 수정하지 않고**, 우리 패키지(`greenhouse_sim`/신규 `rosorin_rl`)로 보강한다.
- [x] **(a) 이동 타겟(작업자) 추가:** 완료. 원기둥(`rosorin_rl/worlds_models/worker_target.sdf`, static) + 컨트롤러 노드(`target_controller_node.py`, 시나리오 1=정속 왕복, 전략 패턴으로 2~5 확장 가능). ⚠️ velocity-control 플러그인은 Fortress에서 link gravity off 미지원으로 원기둥이 넘어져 폐기 → **20Hz set_pose 키네마틱 구동**으로 구현 (`docs/rl_code_guide.md` §5).
- [x] **(b) 타겟 특징 토픽:** 완료. `target_feature_node.py` — 로봇 ground-truth(`dynamic_pose/info`) + 작업자 pose(`/worker/pose`)에서 `[x_norm, y_norm, d_t, θ_t, visible]` 역산, 가우시안 노이즈 3% 주입 → `/target/features` 발행. Sim-to-Real 때 이 노드만 실물 비전으로 교체.
- [x] **(c) RGB-D 깊이 센서 보강:** 완료. greenhouse 스택 카메라를 `rgbd_camera`로 교체(`greenhouse_sim/urdf/depth_cam_scaled.gazebo.xacro`; `robot_scaled.gazebo.xacro`가 벤더 RGB 카메라 대신 include). `/depth_cam/{image,depth_image,points,camera_info}` 발행, frame_id `camera_link0`(브리지 포크: `greenhouse_sim/launch/ros_ign_bridge.launch.py`). RViz2 PointCloud2/Image 확인.
- [x] **(d) 환경 리셋 경로 확정:** 완료. **`/world/greenhouse_world/set_pose`** 를 ros_gz 브리지(`ros_gz_interfaces/srv/SetEntityPose` — Humble 브리지가 지원함을 확인)로 노출, `follow_env.reset()`이 rclpy 서비스 클라이언트로 호출해 로봇 텔레포트 + `/worker/reset`으로 작업자 초기화. (WorldControl 전체 리셋은 sim time까지 리셋되어 불채택.)
- [ ] **(e) 경량 학습 world 옵션:** 온실 잎은 이미 box collision이라 LiDAR엔 동일하고 텍스처는 렌더 비용만 추가. 학습 처리량을 위해 `gen_greenhouse_world.py`에 **텍스처 off / primitive-only** 변형 옵션 추가 권장. (현행 상태공간은 뎁스 범퍼용 카메라 렌더가 필수라 카메라 자체는 끄지 않음 — 텍스처 off는 렌더 비용 절감 목적. 정성평가용으로는 텍스처 온실 유지.) → 우선 **`headless:=true`**(GUI 없이 서버+센서 렌더만)는 `greenhouse.launch.py`/`rl_sim.launch.py`에 추가 완료.

## 3단계 — RL 환경 패키지 (`rosorin_rl`) ✅ (2026-06)
- [x] 새 패키지 `src/rosorin_rl/` 생성 (제조사 monorepo와 분리). 구조·코드 가이드는 **`docs/rl_code_guide.md`**.
- [x] RL 라이브러리 설치: gymnasium 1.2.3 / SB3 2.8.0 / **torch 2.8.0+cu128** (⚠️ 컨테이너 드라이버=CUDA 12.8이라 기본 pip torch(cu13x)는 GPU 인식 실패 — cu128 빌드 필수) / numpy<2 핀(cv_bridge 보호).
- [x] **커스텀 Gymnasium Env** 작성 (`follow_env.py` — 내부 rclpy 노드 + 백그라운드 executor):
  - `_collect_obs()`: `/scan` 6구역(5퍼센타일+EMA) + `/target/features` + `/depth_cam/depth_image` 뎁스범퍼(**바닥 차감** 보정 — `rl_code_guide.md` §5) + `/odom` 속도 → 16차원 → 3프레임 스택 48차원 (`obs_pipeline.py`).
  - `step(action)`: `[ax,ay,aω]` 스케일링 → `/controller/cmd_vel` 발행 → **sim time 0.1s** 대기 → 보상. ⚠️ 벤더 MecanumDrive의 **linear.y 부호 반전** 보정 포함 (실물 포팅 시 제거 — `rl_code_guide.md` §5).
  - `reset()`: set_pose 텔레포트 + 작업자 리셋 + stale 프레임 방지 순서 적용.
- [x] 보상 함수·모드 전환·종료조건 구현 (`reward.py`). 계수 초기값·튜닝 가이드는 `config/rl_params.yaml` + `rl_code_guide.md` §3. (검증 중 환경충돌 임계 0.12→**0.20** 보정 — LiDAR가 로봇 중심 기준이라 반폭 반영.)
- [x] 학습/평가 스크립트 (`train_sac.py` SAC/PPO, `eval_policy.py` 3지표) + 통합 런치 (`rl_sim.launch.py`).

## 4단계 — 학습 / 평가 (진행 중, 2026-06)
- [x] 1차 200k 학습 시도 → 14k 시점 정체 진단(ep_len~50 고정 = lost 반복, 추적 기울기 소멸)
  → **2차 조정**: 스폰 위치(로봇 -0.5 / 작업자 0.6 통로 입구), 작업자 0.05m/s(커리큘럼 0단계),
  보상 재균형(w1↑·w2↓·α↓·접근 shaping 신규·env_collision -30), AprilTag 시각 패널,
  진단 로깅(종료사유 분포·보상 분해·행동 통계 → TB + monitor.csv), resume 시 Replay Buffer 복원.
  상세·튜닝 가이드는 `docs/rl_code_guide.md` §3~§5.
- [x] **3차 수정 — 충돌 감지 버그 해결**: 단일 반경 임계로는 직사각형 로봇의 전/후면 접촉이
  미감지되던 문제(사용자 GUI 관찰로 발견)를 **풋프린트 마진 판정**(`obs_pipeline.env_margin`)으로
  교체, 4방향 실측 검증. 시작 위치 -0.40 확정, 충돌 워밍업 분리(3스텝). 충돌 처리 설계는
  "즉시 리셋 + R_safety 사전 페널티" 확정 — `docs/rl_code_guide.md` §5-9·10.
- [x] **4차 수정 — 타겟 차폐 판정 + 배회 국소최적 해소**: 가시성이 화각만 검사해 작물 벽
  너머도 visible=1 이던 "벽 투시" 버그(사용자 GUI 관찰로 발견)를 작물 줄 AABB 선분 교차
  판정으로 수정(`geometry_utils.OCCLUDERS`), 다른 통로 텔레포트 → lost 발화 실측 검증.
  18k 진단(d_t_mean 1.4·in_aisle 0.29 = 입구 배회)으로 α 1.5→3.0 복원 —
  `docs/rl_code_guide.md` §3·§5-12.
- [ ] 커리큘럼 0단계(0.05m/s) 수렴 확인 → 속도 0.1 → 0.25 단계 상향 (`target.speed` + launch `speed`).
- [ ] **SAC(1순위)** 로 학습, **PPO(베이스라인)** 와 비교 (둘 다 SB3).
- [ ] 헤드리스(가능하면 다중 인스턴스)로 학습 처리량 확보. (Gazebo는 real-time 대비 ~2–5x 한계 — `docs/hardware_requirements.md`.)
- [ ] 평가 3지표 (proposal §6.3): 학습 수렴 속도(Ep_rew_mean, TensorBoard) / 주행 부드러움(ω 변화량) / 센서 노이즈 강건성(노이즈 30% 주입 시 실패율).

## 5단계 — Sim-to-Real (현실 이식)
- [ ] sim의 "타겟 좌표 토픽" 인터페이스를 **실물 RGB-D 객체검출**(AprilTag/색상 작업복 → bbox 중앙 depth → 상대좌표)로 교체. 출력 인터페이스 동일 유지.
- [ ] 학습 정책을 실물 ROSOrin(Jetson Orin Nano)에 포팅, 기초 추종 성능 검증.

## 메모
- 토픽/제어 인터페이스: `README.md`·`docs/environment.md` 표 단일 출처.
- RL 설계: 개요·알고리즘은 `rl_design/0_project_proposal.md`, 수치·수식은 세부 노트(`rl_state_space.md` 등) 단일 출처.
- 온실 레이아웃 변경·재현: `gen_greenhouse_world.py` 파라미터 + `RANDOM_SEED`.
- Ignition 전용 — 리셋/서비스/리소스경로에 Gazebo Classic 문법(`/reset_world`, `gazebo_ros`, `GAZEBO_RESOURCE_PATH`) 혼용 금지.
