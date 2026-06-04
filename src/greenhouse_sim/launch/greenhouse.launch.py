"""온실(greenhouse) world 에 ROSOrin 을 스폰하는 런치.

robot_gazebo/launch/worlds.launch.py 를 본떠 작성했으며, world 만 이 패키지의
greenhouse.sdf 로 교체하고 로봇 스폰/브리지는 robot_gazebo 런치를 그대로 재사용한다.

- 텍스처(잎 사진)는 greenhouse.sdf 안에서 상대경로(media/materials/textures/*.jpg)로
  참조하므로, IGN_GAZEBO_RESOURCE_PATH / GZ_SIM_RESOURCE_PATH 에 이 패키지 share 를 추가한다.
- greenhouse.sdf 는 통로 입구가 원점(0,0)에 오도록 생성되어 있어,
  robot_gazebo 의 spwan_model.launch.py 가 (-x 0 -y 0) 에 스폰하면 통로 입구·+x 방향과 일치.
"""

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription, LaunchService
from launch.actions import (DeclareLaunchArgument, IncludeLaunchDescription,
                            OpaqueFunction, SetEnvironmentVariable)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def launch_setup(context):
    use_sim_time = LaunchConfiguration('use_sim_time', default='true').perform(context)
    moveit_unite = LaunchConfiguration('moveit_unite', default='false').perform(context)
    # headless:=true 면 GUI 없이 서버만 실행 (RL 학습 처리량 확보용).
    # --headless-rendering: GUI 없이도 카메라 센서 렌더링(EGL)은 유지.
    headless = LaunchConfiguration('headless', default='false').perform(context)

    use_sim_time_arg = DeclareLaunchArgument('use_sim_time', default_value=use_sim_time)
    moveit_unite_arg = DeclareLaunchArgument('moveit_unite', default_value=moveit_unite)
    headless_arg = DeclareLaunchArgument('headless', default_value=headless)

    greenhouse_share = get_package_share_directory('greenhouse_sim')

    # world 파일
    world = os.path.join(greenhouse_share, 'worlds', 'greenhouse.sdf')

    # 텍스처 해결용 리소스 경로: 이 패키지 share 를 앞에 추가 (기존 값 보존)
    def prepend(var):
        prev = os.environ.get(var, '')
        return SetEnvironmentVariable(
            var, os.pathsep.join(p for p in (greenhouse_share, prev) if p))

    ign_resource_env = prepend('IGN_GAZEBO_RESOURCE_PATH')
    gz_resource_env = prepend('GZ_SIM_RESOURCE_PATH')

    # Ignition Gazebo
    server_flags = ' -s --headless-rendering' if headless == 'true' else ''
    ign_gz = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('ros_ign_gazebo'),
                        'launch', 'ign_gazebo.launch.py')),
        launch_arguments=[('ign_args', [' -r' + server_flags + ' ' + world])])

    # 로봇 스폰 (스케일된 xacro 사용 — 이 패키지 런치)
    spawn_model_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(greenhouse_share, 'launch', 'spawn_scaled_model.launch.py')),
        launch_arguments={
            'moveit_unite': moveit_unite,
            'use_sim_time': use_sim_time,
        }.items(),
    )

    # ROS <-> Ignition 브리지 (greenhouse 포크 — RGB-D /depth_cam/* 포함)
    ros_ign_bridge_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(greenhouse_share, 'launch', 'ros_ign_bridge.launch.py')),
        launch_arguments={
            'use_sim_time': use_sim_time,
        }.items(),
    )

    return [
        use_sim_time_arg,
        moveit_unite_arg,
        headless_arg,
        ign_resource_env,
        gz_resource_env,
        ign_gz,
        spawn_model_launch,
        ros_ign_bridge_launch,
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
