# CLAUDE.md — rosorin_sim_ws (AgriFollower)

ROSOrin(매카넘) **Ignition Gazebo 시뮬 + 강화학습** 워크스페이스. 목표는 온실에서 **작업자(타겟) 추종** 주행 정책 학습.
프로젝트 개요·환경·실행법은 `README.md`, RL 설계는 `docs/0_project_proposal.md`가 단일 출처다.
(이 파일은 가볍게 유지 — 상세는 아래 위치를 그때그때 참조할 것.)

## 반드시 지킬 것 (invariants)
- **Ignition Gazebo Fortress 전용.** Gazebo Classic 문법(`gazebo_ros`, `spawn_entity.py`, OGRE `.material`, `GAZEBO_RESOURCE_PATH`, `/reset_world`)을 섞지 말 것.
- **`src/simulations/`(제조사 monorepo)는 수정 금지·repo 미포함** — 라이선스 미선언 제3자 코드라 push하지 않음(zip에서 복원). 우리 코드는 `greenhouse_sim`/`rosorin_rl`처럼 별도 패키지로.
- 새 터미널마다 소싱(`install/setup.bash` + `.typerc`), 코드 변경 후 `colcon build --symlink-install`. (빌드/소싱 누락이 가장 흔한 오류.)
- 로봇/센서 구성은 루트 `.typerc`로 결정 (`LIDAR_TYPE=MS200`, `DEPTH_CAMERA_TYPE=aurora`, `MACHINE_TYPE=ROSOrin_Mecanum`).
- 라이선스: 우리 코드는 Apache-2.0(`LICENSE`/`NOTICE`). 제조사 코드는 제외 — 상세는 `README.md` "제조사 코드 & 에셋"·"라이선스".

## 어디에 뭐가 있나 (필요할 때 참조)
- 프로젝트 개요·실행 환경·빌드/실행·온실 재생성·토픽표 → `README.md`
- RL 설계 (MDP·보상·SAC/PPO·Sim-to-Real) → `docs/0_project_proposal.md`
- 진행 상황·단계별 실행 계획 → `docs/roadmap.md`
- Ignition 스택·`.typerc`·토픽 인터페이스·센서 스펙 → `docs/environment.md`
- Docker/GPU 환경·학습 처리량 → `docs/hardware_requirements.md`
- 세팅 성공 기록·트러블슈팅 → `docs/setup_process.md`
