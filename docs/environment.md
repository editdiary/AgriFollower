# 환경 구성 (Ignition 스택 · `.typerc` · 토픽)

## Ignition 스택 요약
| 레이어 | 사용 기술 |
|--------|-----------|
| 시뮬레이터 | Ignition Gazebo Fortress 6.x (`ign gazebo`) |
| world 포맷 | SDF 1.6 + `ignition-gazebo-*-system` 플러그인 |
| 렌더 | `ogre2` (GPU, Docker EGL) |
| ROS↔IGN 브리지 | `ros_gz_bridge`의 `parameter_bridge` |
| 제어 | `ign_ros2_control` + `ros2_control` (`controller_manager`) |
| 센서 | Ignition 내장 (gpu_lidar, imu, depth camera) |

설치했던 런타임 패키지(`docs/setup_process.md` 참고):
`ros-humble-ros-ign-gazebo`, `ros-humble-ign-ros2-control`, `ros-humble-joint-state-publisher`, `ros-humble-ros-gz-bridge`.

## `.typerc` (루트, `~/.bashrc`에서 source)
로봇/센서 구성을 환경변수로 결정한다. 현재 값:
```bash
LIDAR_TYPE=MS200            # A1/A2/C1/G4/S2L/LD14P/MS200
DEPTH_CAMERA_TYPE=aurora    # ascamera/aurora/usb_cam
MACHINE_TYPE=ROSOrin_Mecanum # ROSOrin_Mecanum/ROSOrin_Acker
ASR_LANGUAGE=English        # 음성 부분 (시뮬엔 영향 적음)
MIC_TYPE=xf
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
| `/depth_cam/depth_cam` | sensor_msgs/Image | IGN→ROS |
| `/depth_cam/rgb/camera_info` | sensor_msgs/CameraInfo | IGN→ROS |
- `nav:=true` → `/controller/cmd_vel`가 `/cmd_vel`로 리매핑.
- `map`→`odom` static TF가 브리지 런치에서 함께 발행됨.

## 센서 스펙 (URDF/xacro 검증값)
RViz2로 발행 확인된 센서 구성. 상세 정의는 `robot_gazebo/urdf/{lidar,imu,camera}.gazebo.xacro`.
| 센서 | Ignition 타입 | 주요 스펙 | 비고 |
|------|---------------|-----------|------|
| LiDAR (MS200) | `gpu_lidar` | 270 samples, ±~78°(±1.36 rad), 10 Hz, 0.15–12 m | `/scan`(+`/scan/points`) |
| IMU | `imu` | 50 Hz | `/imu` |
| 카메라 (aurora) | `camera` | 640×400, FOV 60°(1.047 rad), 30 Hz, clip 0.1–10 m | **RGB 전용** — `/depth_cam/depth_cam`는 Image, **depth 미발행** |
- 구동: `MecanumDrive` 플러그인 — `/controller/cmd_vel`(Twist)의 `linear.x/linear.y/angular.z`로 홀로노믹 제어.
- ⚠️ RL(`rl_design/0_project_proposal.md`)의 RGB-D 깊이 추종을 sim에서 쓰려면 `depth_camera`/`rgbd_camera` 센서 보강이 필요(현재 RGB만). → `docs/roadmap.md` 2단계.

## 로봇 스케일 포크 (`greenhouse_sim/urdf/`)
온실 런치(`greenhouse.launch.py`)는 제조사 `robot.gazebo.xacro` 대신 **균일 확대 포크**
`urdf/robot_scaled.gazebo.xacro`(`S=1.83`)로 스폰한다(몸체·바퀴 mesh·마운트 origin·질량 S³·관성 S⁵).
- ⚠️ **불변식: 센서 링크(`camera_link0`, `lidar_frame`)의 visual/collision mesh `scale`은 1로 유지**(몸체·바퀴만 `${S}`),
  마운트 joint origin은 `×S` 유지(큰 몸체 표면에 장착 = 비가림). 센서 mesh를 `${S}`로 키우면 ogre2 카메라 frustum이
  대각선 yaw에서 degenerate되어 `/depth_cam`이 회색이 된다 — 상세·재현·해결은 `docs/troubleshooting.md`.

## 리소스 경로 (텍스처)
Ignition은 `IGN_GAZEBO_RESOURCE_PATH`(Fortress) / `GZ_SIM_RESOURCE_PATH`로 상대경로 리소스를 찾는다.
`greenhouse_sim`의 잎 텍스처(`media/materials/textures/*.jpg`)는 `greenhouse.launch.py`가 패키지 share를
이 변수들 앞에 추가해 해결한다. (Gazebo Classic의 `GAZEBO_RESOURCE_PATH`와는 다른 변수이므로 혼동 주의.)
