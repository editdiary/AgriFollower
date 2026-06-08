# 환경 구성 (Ignition 스택 · `.typerc` · 토픽 · 하드웨어)

## Ignition 스택 요약
| 레이어 | 사용 기술 |
|--------|-----------|
| 시뮬레이터 | Ignition Gazebo Fortress 6.x (`ign gazebo`) |
| world 포맷 | SDF 1.6 + `ignition-gazebo-*-system` 플러그인 |
| 렌더 | `ogre2` (GPU, Docker EGL) |
| ROS↔IGN 브리지 | `ros_gz_bridge`의 `parameter_bridge` |
| 제어 | `ign_ros2_control` + `ros2_control` (`controller_manager`) |
| 센서 | Ignition 내장 (gpu_lidar, imu, rgbd_camera) |

설치했던 런타임 패키지(`docs/setup_process.md` 참고):
`ros-humble-ros-ign-gazebo`, `ros-humble-ign-ros2-control`, `ros-humble-joint-state-publisher`, `ros-humble-ros-gz-bridge`.

## `.typerc` (루트, `~/.bashrc`에서 source)
로봇/센서 구성을 환경변수로 결정한다. 현재 값:
```bash
LIDAR_TYPE=MS200            # A1/A2/C1/G4/S2L/LD14P/MS200
DEPTH_CAMERA_TYPE=aurora    # ascamera/aurora/usb_cam
MACHINE_TYPE=ROSOrin_Mecanum # ROSOrin_Mecanum/ROSOrin_Acker
ASR_LANGUAGE=English        # 음성/마이크 모듈용 — 시뮬 미사용
MIC_TYPE=xf                 # 〃 (스케일 포크는 제조사 mic_link 제거, 아래 "로봇 스케일 포크" 참조)
ROS_DOMAIN_ID=0
CYCLONEDDS_URI=file:///etc/cyclonedds/config.xml
HOST=/  MASTER=/           # 멀티로봇(robot_xxx)용, 현재 비활성
```
- `MACHINE_TYPE`/`LIDAR_TYPE`/`DEPTH_CAMERA_TYPE`를 바꾸면 URDF/센서 구성이 달라진다(xacro 조건부).

## 토픽 인터페이스 (`robot_gazebo/launch/ros_ign_bridge.launch.py`)
| 토픽 | 타입 | 방향 |
|------|------|------|
| `/controller/cmd_vel` | geometry_msgs/Twist | ROS→IGN |
| `/odom` | nav_msgs/Odometry | IGN→ROS |
| `/odom/tf` (→ `tf`) | tf2_msgs/TFMessage | IGN→ROS |
| `/clock` | rosgraph_msgs/Clock | IGN→ROS |
| `/joint_states` | sensor_msgs/JointState | IGN→ROS |
| `/scan` | sensor_msgs/LaserScan | IGN→ROS |
| `/scan/points` | sensor_msgs/PointCloud2 | IGN→ROS |
| `/imu` | sensor_msgs/Imu | IGN→ROS |
| `/depth_cam/image` | sensor_msgs/Image | IGN→ROS |
| `/depth_cam/depth_image` | sensor_msgs/Image (32FC1) | IGN→ROS |
| `/depth_cam/points` | sensor_msgs/PointCloud2 | IGN→ROS |
| `/depth_cam/camera_info` | sensor_msgs/CameraInfo | IGN→ROS |
- `nav:=true` → `/controller/cmd_vel`가 `/cmd_vel`로 리매핑.
- `map`→`odom` static TF가 브리지 런치에서 함께 발행됨.
- ⚠️ 위 `/depth_cam/*` 4토픽은 **greenhouse 스택**(`greenhouse_sim/launch/ros_ign_bridge.launch.py`, RGB-D `rgbd_camera`) 기준. 벤더 `robot_gazebo` 브리지는 여전히 RGB 전용(`/depth_cam/depth_cam` Image)이다.

## 센서 스펙 (URDF/xacro 검증값)
RViz2로 발행 확인된 센서 구성. 상세 정의는 IMU는 `robot_gazebo/urdf/imu.gazebo.xacro`, LiDAR/카메라는 `greenhouse_sim/urdf/{lidar_scaled,depth_cam_scaled}.gazebo.xacro`.
| 센서 | Ignition 타입 | 주요 스펙 | 비고 |
|------|---------------|-----------|------|
| LiDAR (MS200) | `gpu_lidar` | 450 samples, 360°(±π, 0.8°/sample), 10 Hz, 0.15–12 m | `/scan`(+`/scan/points`) — greenhouse 포크. 벤더 파일은 ~156°(±1.36 rad)·270 samples 로 제한돼 있었음(실물 MS200 은 360°) |
| IMU | `imu` | 50 Hz | `/imu` |
| 카메라 (aurora) | `rgbd_camera` | 640×400, FOV 60°(1.047 rad), 30 Hz, clip 0.1–10 m | **RGB-D** (greenhouse 스택) — `/depth_cam/{image,depth_image,points,camera_info}`, frame_id `camera_link0`. 정의: `greenhouse_sim/urdf/depth_cam_scaled.gazebo.xacro` |
- 구동: `MecanumDrive` 플러그인 — `/controller/cmd_vel`(Twist)의 `linear.x/linear.y/angular.z`로 홀로노믹 제어.
- depth_image는 32FC1(미터). RViz Image로는 어둡게 보이는 게 정상 → PointCloud2(`/depth_cam/points`)로 보는 게 정석.

## 카메라 Intrinsic & Cam–LiDAR Extrinsic
시뮬은 캘리브레이션이 필요 없다 — 파라미터를 SDF/URDF에 직접 정의하므로 그 값이 곧 ground truth(오차 0)다.
현실 캘리브레이션은 미지 파라미터를 추정하는 역문제지만, 시뮬에서는 추정할 게 없다.

### Intrinsic (핀홀, 무왜곡)
SDF에 명시적 intrinsic 태그는 없고, Ignition이 FOV·해상도에서 자동 산출해 `/depth_cam/camera_info`로 발행한다(브리지 등록됨).
유도식: `fx = fy = (W/2) / tan(hfov/2)`.
| 파라미터 | 값 | 비고 |
|----------|-----|------|
| fx = fy | ≈ 554.26 px | `320 / tan(1.047/2)` |
| cx, cy | 320, 200 | 영상 중심 (640×400) |
| 왜곡계수 | 0 | 시뮬 핀홀은 무왜곡 |
- 확인: `ros2 topic echo /depth_cam/camera_info --once` → K 행렬.
- 출처: `greenhouse_sim/urdf/depth_cam_scaled.gazebo.xacro:20-27` (hfov 1.047 rad, 640×400).

### Extrinsic (Cam–LiDAR)
두 센서 모두 `base_link`에 **회전 없는(rpy=0) fixed joint**로 마운트 → 상대 변환은 origin 차이 그 자체.
| 링크 | base_link 기준 xyz [m] | 출처 |
|------|------------------------|------|
| `camera_link0` | [0.10499, 0.00014, 0.16811] | `greenhouse_sim/urdf/ascamera_scaled.xacro:55-67` |
| `lidar_frame` | [0.02100, 0.00014, 0.22736] | `greenhouse_sim/urdf/rosorin_scaled.xacro:114-126` |

→ **T(camera_link0 → lidar_frame): 평행이동 [-0.0840, ≈0, +0.0592] m, 회전 = I** (LiDAR가 카메라보다 8.4 cm 뒤·5.9 cm 위).
- 런타임 획득: `robot_state_publisher`가 TF 발행 → `lookup_transform('camera_link0', 'lidar_frame', ...)`. fixed joint라 1회 조회 후 캐시하면 된다.
- 위 값은 스케일 포크(`S=1.83`)가 반영된 값. 실기 공칭값은 ÷1.83이 아니라 **벤더 원본 URDF의 origin**이다.

### ⚠️ 주의
- **optical frame 링크 없음** — `camera_link0`은 X-forward body 좌표계 그대로다(`*_optical_frame` 미정의). 픽셀↔3D 투영(센서 퓨전 등) 시 ROS 광학 관례(Z-forward) 회전 `rpy="-π/2 0 -π/2"`를 끼워야 한다.
- Sim-to-Real: 실제 aurora의 렌즈 왜곡·마운트 공차는 시뮬과 다르므로, 실기에서는 별도 캘리브레이션 값으로 교체해야 한다(시뮬 값 = 노이즈 없는 공칭값).
- RL 파이프라인에서의 활용(마커 역투영, 가상 뎁스 범퍼 ROI)은 `docs/rl_design/rl_state_space.md` §2.5 참조.

## 로봇 스케일 포크 (`greenhouse_sim/urdf/`)
온실 런치(`greenhouse.launch.py`)는 제조사 `robot.gazebo.xacro` 대신 **균일 확대 포크**
`urdf/robot_scaled.gazebo.xacro`(`S=1.83`)로 스폰한다(몸체·바퀴 mesh·마운트 origin·질량 S³·관성 S⁵).
- 제조사 대비 변경점: ① 음성 모듈 `mic_link` 제거(시뮬 미사용), ② 카메라를 RGB-D `rgbd_camera`로 교체(`urdf/depth_cam_scaled.gazebo.xacro` → `/depth_cam/*`, 벤더 RGB 전용 `camera.gazebo.xacro` 대신 include).
- ⚠️ **불변식: 센서 링크(`camera_link0`, `lidar_frame`)의 visual/collision mesh `scale`은 1로 유지**(몸체·바퀴만 `${S}`),
  마운트 joint origin은 `×S` 유지(큰 몸체 표면에 장착 = 비가림). 센서 mesh를 `${S}`로 키우면 ogre2 카메라 frustum이
  대각선 yaw에서 degenerate되어 `/depth_cam`이 회색이 된다 — 상세·재현·해결은 `docs/troubleshooting.md`.

## 리소스 경로 (텍스처)
Ignition은 `IGN_GAZEBO_RESOURCE_PATH`(Fortress) / `GZ_SIM_RESOURCE_PATH`로 상대경로 리소스를 찾는다.
`greenhouse_sim`의 잎 텍스처(`media/materials/textures/*.jpg`)는 `greenhouse.launch.py`가 패키지 share를
이 변수들 앞에 추가해 해결한다. (Gazebo Classic의 `GAZEBO_RESOURCE_PATH`와는 다른 변수이므로 혼동 주의.)

## 하드웨어 / 렌더링 부하
이 ws는 **Docker + NVIDIA EGL로 GPU 가속 렌더(ogre2)를 사용**한다 (컨테이너 스펙·실행 예시는 `README.md` "실행 환경" 참조).
> ⚠️ 옛 ws(`rosorin_sim_ws_old`) 문서에는 "Gazebo는 CPU(llvmpipe) 소프트웨어 렌더"라고 적혀 있었으나, **이 환경에는 해당하지 않는다.**

- depth/RGB 카메라 시뮬이 가장 무겁다. 학습 시에는 **헤드리스(`headless:=true`)** 와 해상도/주기 튜닝 권장.
- 단, **카메라 센서는 헤드리스에서도 GPU 렌더가 필요**하다(Ignition `Sensors` 시스템이 ogre2로 이미지 생성). 현행 상태공간(`rl_design/rl_state_space.md`)은 뎁스 범퍼·마커 특징에 카메라가 필수라 비활성은 불가.
- Gazebo는 물리·렌더가 무거워 학습 속도가 real-time 대비 대략 ~2–5x 한계. 가벼운 PyBullet 류 대비 느리지만, 실 센서와 동일한 토픽 구조가 장점이라 채택 (`rl_design/0_project_proposal.md` §4.1).
- 위 ~2–5x 를 내려면 world physics 의 `<real_time_factor>` 를 **`0`(무제한)** 으로 둬야 한다(`greenhouse.sdf`·생성기 `gen_greenhouse_world.py`). 기본값 `1.0` 이면 자원이 남아도 실시간(1x)에 묶인다. Fortress 는 `real_time_factor` 만 유효하고 Classic 의 `real_time_update_rate` 는 무시 — 함정·근거는 `troubleshooting.md`.
