# CLAUDE.md — rosorin_sim_ws (AgriFollower)

ROSOrin(매카넘) **Ignition Gazebo 시뮬 + 강화학습** 워크스페이스. 목표는 온실에서 **작업자(타겟) 추종** 주행 정책 학습.
프로젝트 개요·환경·실행법은 `README.md`, RL 설계는 `docs/rl_design/0_project_proposal.md`가 단일 출처다.
(이 파일은 가볍게 유지 — 상세 절차/배경은 아래 "어디에 뭐가 있나"의 위치를 그때그때 참조할 것.)

## 개발 환경 & 버전 (코드/명령 생성 전 기준값)
| 항목 | 값 |
|------|-----|
| 시뮬레이터 | **Ignition Gazebo Fortress 6.x** (Gazebo Classic 아님) · 렌더 `ogre2`(GPU) |
| OS / 미들웨어 | Ubuntu 22.04 · **ROS 2 Humble** · Python 3.10 |
| 빌드 | `colcon build --symlink-install` |
| 실행 환경 | Docker 컨테이너 `nvidia-egl-desktop-ros2:humble` · GPU 가속 · noVNC(`:6080`) |
| 로봇 | ROSOrin Mecanum (4 매카넘 휠, 3-DOF 홀로노믹) |
| 센서 | 2D LiDAR `MS200` · RGB 카메라 `aurora` · IMU (구성은 루트 `.typerc`로 결정) |
| RL 스택(예정) | Gymnasium + Stable-Baselines3 (SAC 1순위 / PPO 베이스라인) |

## 반드시 지킬 것 (invariants)
- **Ignition Gazebo Fortress 전용.** Gazebo Classic 문법(`gazebo_ros`, `spawn_entity.py`, OGRE `.material`, `GAZEBO_RESOURCE_PATH`, `/reset_world`)을 섞지 말 것.
- **`src/simulations/`(제조사 monorepo)는 수정 금지·repo 미포함** — 라이선스 미선언 제3자 코드라 push하지 않음(zip에서 복원). 우리 코드는 `greenhouse_sim`/`rosorin_rl`처럼 별도 패키지로.
- 새 터미널마다 소싱(`install/setup.bash` + `.typerc`), 코드 변경 후 `colcon build --symlink-install`. (빌드/소싱 누락이 가장 흔한 오류.)
- 로봇/센서 구성은 루트 `.typerc`로 결정 (`LIDAR_TYPE=MS200`, `DEPTH_CAMERA_TYPE=aurora`, `MACHINE_TYPE=ROSOrin_Mecanum`).
- 스케일 포크(`greenhouse_sim/urdf/`, `S=1.83`)에서 **센서 링크 mesh `scale`은 1 유지**(몸체·바퀴만 `${S}`, 마운트 origin은 `×S`). 센서 mesh를 키우면 대각선 yaw에서 `/depth_cam` 회색 회귀 → 상세 `docs/troubleshooting.md`.
- 라이선스: 우리 코드는 Apache-2.0(`LICENSE`/`NOTICE`). 제조사 코드는 제외 — 상세는 `README.md` "제조사 코드 & 에셋"·"라이선스".

## 어디에 뭐가 있나 (필요할 때 참조)
- 프로젝트 개요·실행 환경·빌드/실행·온실 재생성·토픽표 → `README.md`
- RL 설계 (MDP·보상·SAC/PPO·Sim-to-Real) → `docs/rl_design/` (단일 출처 `0_project_proposal.md` + 상태/보상/시나리오 세부 노트)
- 진행 상황·단계별 실행 계획 → `docs/roadmap.md`
- Ignition 스택·`.typerc`·토픽 인터페이스·센서 스펙 → `docs/environment.md`
- Docker/GPU 환경·학습 처리량 → `docs/hardware_requirements.md`
- 세팅 성공 기록(설치 절차) → `docs/setup_process.md`
- 치명적 오류 기록(원인·해결) → `docs/troubleshooting.md`
