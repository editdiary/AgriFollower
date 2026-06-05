"""스케일된 ROSOrin 을 스폰하는 런치.

robot_gazebo/launch/spwan_model.launch.py 를 본떠 작성했으며 세 가지가 다르다:
1) robot_description 을 생성하는 xacro 를 제조사 robot.gazebo.xacro 대신 이 패키지의
   urdf/robot_scaled.gazebo.xacro (균일 확대 포크) 로 가리킨다.
2) URDF 에 <ros2_control> 태그가 없어 ros2_control_node/spawner 가 즉시 죽고 로그를
   폭주시키므로 제거했다. 로봇은 MecanumDrive 플러그인으로 구동되고, /joint_states 는
   gz JointStatePublisher 플러그인→브리지로 공급되므로 휠 TF·표시에 영향 없다.
3) joint_state_publisher 노드를 제거했다. source_list 가 자기 발행 토픽과 같은
   /joint_states 라 브리지 공급분을 받아 같은 토픽에 되쏘는 중복 발행이었다
   (브리지 공급만으로 충분. 참고: "Moved backwards in time" 경고 폭주의 원인은
   이 노드가 아니라 이전 세션에서 살아남은 좀비 parameter_bridge 가 /clock 을
   이중 발행한 것 — docs/troubleshooting.md 참조).
"""

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription, LaunchService
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration, Command

from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def launch_setup(context):
    use_sim_time = LaunchConfiguration('use_sim_time', default='true').perform(context)
    moveit_unite = LaunchConfiguration('moveit_unite', default='false').perform(context)

    sim_ign = 'false' if moveit_unite == 'true' else 'true'

    use_sim_time_arg = DeclareLaunchArgument('use_sim_time', default_value=use_sim_time)
    use_sim_time = True if use_sim_time == 'true' else False

    greenhouse_share = get_package_share_directory('greenhouse_sim')

    # 스케일된 xacro (이 패키지) — robot_description 생성
    xacro_file = os.path.join(greenhouse_share, 'urdf', 'robot_scaled.gazebo.xacro')

    # ParameterValue(value_type=str) 로 감싸지 않으면 launch_ros 가 이 대용량 URDF
    # 문자열을 YAML 로 파싱하려다 실패한다.
    robot_description_content = ParameterValue(
        Command(['xacro ', xacro_file, ' sim_ign:=', sim_ign]),
        value_type=str,
    )

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[
            {
                'robot_description': robot_description_content,
                'use_sim_time': use_sim_time
            }
        ],
    )

    ignition_spawn_entity = Node(
        package='ros_ign_gazebo',
        executable='create',
        output='screen',
        arguments=['-topic', 'robot_description',
                   '-name', 'robot',
                   '-allow_renaming', 'true',
                   # x=-0.40: 입구 문(서쪽 벽 x=-0.90) 앞 — RL 리셋 위치
                   # (rosorin_rl config robot.reset_x)와 일치시켜 GUI 일관성 유지.
                   # (풋프린트 충돌 판정 + LiDAR 노이즈 마진 실측으로 확정한 위치)
                   '-x', '-0.40',
                   '-y', '0',
                   '-z', '0.02'
                   ],
        parameters=[
            {"use_sim_time": True}],
    )

    return [
        use_sim_time_arg,

        robot_state_publisher_node,
        ignition_spawn_entity,
    ]


def generate_launch_description():
    return LaunchDescription([
        OpaqueFunction(function=launch_setup)
    ])


if __name__ == '__main__':
    ld = generate_launch_description()
    ls = LaunchService()
    ls.include_launch_description(ld)
    ls.run()
