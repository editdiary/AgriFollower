# 하드웨어 / 렌더링 환경

## 핵심
이 ws는 **Docker + NVIDIA EGL로 GPU 가속 렌더(ogre2)를 사용**한다.
> ⚠️ 옛 ws(`rosorin_sim_ws_old`) 문서에는 "Gazebo는 CPU(llvmpipe) 소프트웨어 렌더, GPU는 RL 연산 전용"이라고 적혀 있었으나, **이 환경에는 해당하지 않는다.** 현재 컨테이너는 GPU로 Gazebo를 직접 가속한다.

## 실행 환경
- 컨테이너 이미지: `nvidia-egl-desktop-ros2:humble` (https://github.com/atinfinity/nvidia-egl-desktop-ros2 기반)
- 접속: noVNC `http://<host>:6080` (컨테이너 포트 8080 → 호스트 6080)
- GPU 패스스루: `--gpus all`, `NVIDIA_DRIVER_CAPABILITIES=all`, `--shm-size=16g`
- 마운트: 호스트 `~/rosorin_sim_ws` ↔ 컨테이너 `~/rosorin_sim_ws`

## 소프트웨어 스택
- OS: Ubuntu 22.04 / ROS2 **Humble** / Python 3.10 / colcon(ament)
- 시뮬레이터: **Ignition Gazebo Fortress 6.x** (`ign gazebo`, 렌더 엔진 `ogre2`)
- 브리지/제어: `ros_gz_bridge`, `ros_ign_gazebo`, `ign_ros2_control`

## 부하 메모
- depth/RGB 카메라 시뮬이 가장 무겁다. 학습 시에는 **헤드리스(`-s`/GUI off)** 와 해상도/주기 튜닝 권장.
- RL 학습 처리량이 필요하면 헤드리스 + 다중 인스턴스 병렬 실행을 고려.
- 단, **카메라 센서는 헤드리스에서도 GPU 렌더가 필요**하다(Ignition `Sensors` 시스템이 ogre2로 이미지 생성). 카메라 기반(RGB-D) 관측은 처리량을 크게 떨어뜨린다 — 현행 상태공간(`rl_design/rl_state_space.md`)은 뎁스 범퍼·마커 특징에 카메라가 필수라 비활성은 불가하고, 해상도/주기 튜닝 + 경량 world(텍스처 off)로 비용을 줄인다. → `docs/roadmap.md` 2단계.
- Gazebo는 물리·렌더가 무거워 학습 속도가 real-time 대비 대략 ~2–5x 한계(`rl_design/0_project_proposal.md` §4.1). 가벼운 PyBullet 류 대비 느리지만, 실 센서와 동일한 토픽 구조가 장점이라 채택.

## 컨테이너 실행 예시
```bash
docker run -d \
  --name ros2_gpu_vnc \
  --gpus all \
  -e NVIDIA_DRIVER_CAPABILITIES=all \
  --shm-size=16g \
  --pid=host \
  -e SIZEW=1920 -e SIZEH=1080 \
  -e PASSWD=<YOUR_VNC_PASSWORD> -e BASIC_AUTH_PASSWORD=<YOUR_BASIC_AUTH_PASSWORD> \
  -e NOVNC_ENABLE=true \
  -p 6080:8080 \
  -v ~/rosorin_sim_ws:/home/user/rosorin_sim_ws \
  nvidia-egl-desktop-ros2:humble
```
