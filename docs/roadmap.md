# 로드맵 — ROSOrin 시뮬 기반 강화학습 (작업자 추종)

목표: Ignition Gazebo에서 ROSOrin(매카넘)을 온실에 띄우고, **수확 작업자를 일정 거리로 추종하면서
주변 장애물에 맞춰 최적 자세를 잡는 주행 정책**을 강화학습으로 학습한다.
단순 통로 주행/내비게이션이 아니라 **타겟(작업자) 추종**이 핵심 과제다.

> RL 설계 단일 출처: **`rl_design/0_project_proposal.md`** (MDP·보상·알고리즘·Sim-to-Real 상세).
> 이 로드맵은 그 설계를 **현재 ws(Ignition Fortress + 기존 토픽/온실)에서 실행하는 단계**로 풀어 쓴 것이다.
> 토픽/제어 인터페이스는 `CLAUDE.md`의 표를 단일 출처로 사용.

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

## MDP 요약 (출처: `rl_design/0_project_proposal.md` §5 — 상세는 그쪽 참조)
- **상태(30차원):** 단일 프레임 10차원 = 타겟 상대좌표 `[ΔX, ΔZ, d]`(3) + LiDAR 기하 특징 `[d_left_min, d_right_min, d_obs_front, θ_obs_front]`(4) + 로봇 속도 `[vx, vy, ω]`(3). 최근 3프레임(`t, t-1, t-2`) 스택 → 30차원 1D 벡터.
- **행동(3-DOF 연속):** `[ax, ay, aω] ∈ [-1,1]³` → `vx=ax·Vmax`, `vy=ay·Vmax`(Vmax≈0.5 m/s), `ω=aω·Wmax`(Wmax≈1.0 rad/s). 매카넘 홀로노믹 활용.
- **보상:** `R = w1·R_tracking + w2·R_safety + w3·R_pose` (목표거리 d_opt≈1.5m 유지 / 충돌 근접 페널티 / 불필요 회전 페널티).
- **종료:** 환경충돌(d_min<0.15m), 타겟충돌(d<0.5m), 타겟이탈(d>4m 또는 시야 밖), 성공(최대스텝 유지).

## 1단계 — 온실 환경 검증 (완료)
- [x] 온실 + 로봇 스폰 시각 확인, 전 토픽 RViz2 검증, teleop 주행 확인 (위 "현재 상태" 참조).
- [ ] (필요 시) 스폰 위치·작물 간격이 로봇 폭/타겟 추종에 적절한지 점검, `gen_greenhouse_world.py` 파라미터(`AISLE_WIDTH` 등) 조정 후 재생성.

## 2단계 — 시뮬 선결 과제 (RL 학습에 필요한 환경/센서 보강)
현재 sim에는 RL에 필요한 요소 일부가 없다. **제조사 `src/simulations/`는 수정하지 않고**, 우리 패키지(`greenhouse_sim`/신규 `rosorin_rl`)로 보강한다.
- [ ] **(a) 이동 타겟(작업자) 추가:** 온실에 이동 원기둥(또는 actor) 1개 + 컨트롤러 노드(직진/지그재그·무작위 속도). 통로 사이를 돌아다니게 함.
- [ ] **(b) 타겟 상대좌표 토픽:** sim에선 객체검출 대신 타겟 엔티티 ground-truth pose에서 로봇 기준 상대좌표 `[ΔX, ΔZ, d]` 산출 → 토픽 발행. **가우시안 노이즈(±2~5%) 주입**으로 도메인 랜덤화(Sim-to-Real 대비).
- [x] **(c) RGB-D 깊이 센서 보강:** 완료. greenhouse 스택 카메라를 `rgbd_camera`로 교체(`greenhouse_sim/urdf/depth_cam_scaled.gazebo.xacro`; `robot_scaled.gazebo.xacro`가 벤더 RGB 카메라 대신 include). `/depth_cam/{image,depth_image,points,camera_info}` 발행, frame_id `camera_link0`(브리지 포크: `greenhouse_sim/launch/ros_ign_bridge.launch.py`). RViz2 PointCloud2/Image 확인.
- [ ] **(d) 환경 리셋 경로 확정:** ⚠️ proposal §4.3의 `/reset_world`는 **Gazebo Classic 문법**. 본 ws는 Ignition Fortress이므로 **`/world/greenhouse_world/control`(WorldControl reset)** 또는 엔티티 재배치용 **`/world/greenhouse_world/set_pose`** 를 사용(ros_gz 브리지 또는 ign transport 직접 호출). 로봇·타겟 위치 초기화 방식 결정.
- [ ] **(e) 경량 학습 world 옵션:** 온실 잎은 이미 box collision이라 LiDAR엔 동일하고 텍스처는 렌더 비용만 추가. 헤드리스 LiDAR 학습 처리량을 위해 `gen_greenhouse_world.py`에 **텍스처 off / primitive-only** 변형 옵션 추가 권장. (카메라/정성평가용으로는 텍스처 온실 유지.)

## 3단계 — RL 환경 패키지 (`rosorin_rl`)
- [ ] 새 패키지 `src/rosorin_rl/` 생성 (제조사 monorepo와 분리).
- [ ] RL 라이브러리 설치: `stable-baselines3` / `gymnasium` / `torch` (컨테이너에 미설치 상태).
- [ ] **커스텀 Gymnasium Env(= ROS2 노드)** 작성:
  - `_get_obs()`: `/scan`(→ LiDAR 기하 특징 4개로 압축) + 타겟 좌표 토픽 + `/imu`·`/odom`(로봇 속도) 구독 → 10차원 정제 후 **3프레임 스택 → 30차원**. 노이즈 주입.
  - `step(action)`: `[ax,ay,aω]`를 물리값으로 스케일링 → `/controller/cmd_vel`(Twist, vx/vy/ω) 발행 → 일정 시간(ROS rate) 진행 후 보상 계산.
  - `reset()`: 2단계 (d)의 Ignition 서비스로 로봇·타겟 초기화, 상태 버퍼 비움.
- [ ] 보상 함수(`R_tracking`/`R_safety`/`R_pose`)·종료조건 구현 (proposal §5.3~5.4).

## 4단계 — 학습 / 평가
- [ ] **SAC(1순위)** 로 학습, **PPO(베이스라인)** 와 비교 (둘 다 SB3).
- [ ] 헤드리스(가능하면 다중 인스턴스)로 학습 처리량 확보. (Gazebo는 real-time 대비 ~2–5x 한계 — `docs/hardware_requirements.md`.)
- [ ] 평가 3지표 (proposal §6.3): 학습 수렴 속도(Ep_rew_mean, TensorBoard) / 주행 부드러움(ω 변화량) / 센서 노이즈 강건성(노이즈 30% 주입 시 실패율).

## 5단계 — Sim-to-Real (현실 이식)
- [ ] sim의 "타겟 좌표 토픽" 인터페이스를 **실물 RGB-D 객체검출**(AprilTag/색상 작업복 → bbox 중앙 depth → 상대좌표)로 교체. 출력 인터페이스 동일 유지.
- [ ] 학습 정책을 실물 ROSOrin(Jetson Orin Nano)에 포팅, 기초 추종 성능 검증.

## 메모
- 토픽/제어 인터페이스: `CLAUDE.md` 표 단일 출처.
- RL 설계(MDP/보상/알고리즘): `rl_design/0_project_proposal.md` 단일 출처.
- 온실 레이아웃 변경·재현: `gen_greenhouse_world.py` 파라미터 + `RANDOM_SEED`.
- Ignition 전용 — 리셋/서비스/리소스경로에 Gazebo Classic 문법(`/reset_world`, `gazebo_ros`, `GAZEBO_RESOURCE_PATH`) 혼용 금지.
