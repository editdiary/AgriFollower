"""rl_sim.launch.py — RL 학습용 시뮬레이션 전체 스택 런치.

기존 greenhouse.launch.py(온실 + 로봇 + 센서 브리지)에 RL 에 필요한 요소를 얹는다:
  ① 작업자(타겟) 원기둥 스폰 (worlds_models/worker_target.sdf — static 모델)
  ② RL 전용 추가 브리지 (기존 ros_ign_bridge.launch.py 는 건드리지 않음 — 관심사 분리)
     - /world/.../dynamic_pose/info : 동적 모델 ground-truth pose (IGN → ROS)
     - /world/.../set_pose          : 엔티티 텔레포트 서비스 (작업자 이동 + 에피소드 리셋)
  ③ target_controller : 작업자 걸음 컨트롤러 노드 (시나리오 1: 정속 왕복,
                         set_pose 키네마틱 구동 + /worker/pose 발행)
  ④ target_feature    : 타겟 특징 [x_norm,y_norm,d_t,θ_t] 발행 노드 (+노이즈)

[ 실행 ]
  ros2 launch rosorin_rl rl_sim.launch.py                # GUI 포함 (검증용)
  ros2 launch rosorin_rl rl_sim.launch.py headless:=true # 서버만 (학습용, 처리량↑)

이후 별도 터미널에서 학습 시작: ros2 run rosorin_rl train_sac
"""

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

WORLD = 'greenhouse_world'


def generate_launch_description():
    rl_share = get_package_share_directory('rosorin_rl')
    greenhouse_share = get_package_share_directory('greenhouse_sim')

    headless = LaunchConfiguration('headless', default='false')
    scenario = LaunchConfiguration('scenario', default='1')

    # ---------------- ① 기존 온실 스택 (world + 로봇 + 센서 브리지) ----------------
    greenhouse_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(greenhouse_share, 'launch', 'greenhouse.launch.py')),
        launch_arguments={'headless': headless}.items(),
    )

    # ---------------- ② 작업자(타겟) 원기둥 스폰 ----------------
    # z=0.8: 원기둥(높이 1.6m)의 중심 — 바닥에 딱 맞게 서 있는 높이.
    # x=0.6: 통로 입구(잎 시작점). config 의 target.reset_x/y 와 일치시킬 것.
    spawn_worker = Node(
        package='ros_ign_gazebo',
        executable='create',
        output='screen',
        arguments=[
            '-file', os.path.join(rl_share, 'worlds_models', 'worker_target.sdf'),
            '-name', 'worker_target',
            '-x', '0.6', '-y', '0.0', '-z', '0.8',
        ],
        parameters=[{'use_sim_time': True}],
    )

    # ---------------- ③ RL 전용 추가 브리지 ----------------
    # 문법: <topic>@<ROS타입><방향><IGN타입>  (] = ROS→IGN, [ = IGN→ROS)
    # 서비스: <service>@<ROS srv 타입> (Ignition 서비스를 ROS 서비스로 노출)
    rl_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='rl_bridge',
        output='screen',
        arguments=[
            # 모든 동적 모델의 ground-truth 월드 pose (IGN → ROS) — 로봇 yaw 등
            f'/world/{WORLD}/dynamic_pose/info@tf2_msgs/msg/TFMessage[ignition.msgs.Pose_V',
            # 엔티티 텔레포트 서비스 (작업자 이동 + 에피소드 리셋 — Ignition Fortress 방식)
            f'/world/{WORLD}/set_pose@ros_gz_interfaces/srv/SetEntityPose',
        ],
        parameters=[{'use_sim_time': True}],
    )

    # ---------------- ④ 작업자 컨트롤러 + 타겟 특징 노드 ----------------
    target_controller = Node(
        package='rosorin_rl',
        executable='target_controller',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            # LaunchConfiguration 은 문자열이므로 int 로 명시 변환
            'scenario': ParameterValue(scenario, value_type=int),
            # 보행 속도·왕복 구간·시작 위치는 config/rl_params.yaml 의 target 섹션과 일치시킬 것
            'speed': 0.1,         # 커리큘럼 1단계 (7차, config 주석 참조)
            'aisle_x_min': 0.9,
            'aisle_x_max': 6.3,
            'reset_x': 0.6,       # 통로 입구
            'reset_y': 0.0,
        }],
    )

    target_feature = Node(
        package='rosorin_rl',
        executable='target_feature',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'noise_pct': 0.03,        # 도메인 랜덤화 노이즈 (proposal §4.4)
            'marker_height': 0.20,    # 마커 가정 높이 [m] (0.35→0.20: 추종 거리에서
                                      # 화면 상단 잘림 보정 — worker_target.sdf 주석 참조)
        }],
    )

    return LaunchDescription([
        DeclareLaunchArgument('headless', default_value='false',
                              description='true 면 GUI 없이 서버만 (학습용)'),
        DeclareLaunchArgument('scenario', default_value='1',
                              description='학습 시나리오 번호 (현재 1만 구현)'),
        greenhouse_launch,
        spawn_worker,
        rl_bridge,
        target_controller,
        target_feature,
    ])
