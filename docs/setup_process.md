# 제조사 파일 따라함

> 메모: 제조사 가이드는 워크스페이스 이름을 `ros2_ws`로 안내하지만, 이 ws는 **`rosorin_sim_ws`**로 이름을 바꿔 쓴다.
> 아래 명령은 모두 `rosorin_sim_ws` 기준으로 정규화해 두었다.
> (소싱: `source ~/rosorin_sim_ws/install/setup.bash`, `source ~/rosorin_sim_ws/.typerc`)

## 1. 새롭게 docker 이미지 환경 생성

https://github.com/atinfinity/nvidia-egl-desktop-ros2
위의 파일을 기반으로 새롭게 이미지 빌드 및 컨테이너 생성

컨테이너 실행 시 아래 명령어 사용:

docker run -d \
  --name ros2_gpu_vnc_temp \
  --gpus all \
  -e NVIDIA_DRIVER_CAPABILITIES=all \
  --shm-size=16g \
  --pid=host \
  -e SIZEW=1920 \
  -e SIZEH=1080 \
  -e PASSWD=<YOUR_VNC_PASSWORD> \
  -e BASIC_AUTH_PASSWORD=<YOUR_BASIC_AUTH_PASSWORD> \
  -e NOVNC_ENABLE=true \
  -p 6080:8080 \
  -v ~/rosorin_sim_ws:/home/user/rosorin_sim_ws \
  nvidia-egl-desktop-ros2:humble


## 2. Feature Package 옮기기

제조사에서 제공하는 파일을 옮겨서 가이드 문서를 따라서 명령어 입력

파일 위치:
/PPJ/ROSOrin_Tutorials/09_Gazebo Simulation/Virtual Machine Software & Image File/Resources/3_Feature Package"

여기서 simulations.zip이랑 .typerc 파일 home으로 옮기기

이후 아래 명령어를 통해 ws랑 압축 해제 및 파일 위치 변경 후 패키지 빌드

mkdir -p ~/rosorin_sim_ws/src

unzip ~/simulations.zip
mv ~/simulations ~/rosorin_sim_ws/src/simulations

cd ~/rosorin_sim_ws && colcon build --symlink-install

`.typerc`도 옮기고 bashrc 등록한다.
(`.typerc`는 "로봇이 어떤 하드웨어 부품들로 조립되어 있는지 명시해 둔 제조사 전용 로봇 명세서(환경 설정 파일)"다. 상세는 `docs/environment.md` 참조.)

mv /home/user/.typerc ~/rosorin_sim_ws/.typerc

cd ~/rosorin_sim_ws/ && ls -a
(이걸로 .typerc 파일이 확인되어야 함)

echo "source ~/rosorin_sim_ws/install/setup.bash">>~/.bashrc
echo "source ~/rosorin_sim_ws/.typerc">>~/.bashrc

source ~/.bashrc

URDF와 Xacro 패키지도 설치를 해줘야 한다.

sudo apt update

sudo apt-get install ros-humble-urdf
sudo apt-get install ros-humble-xacro

## 3. Gazebo 실행

문서에 따라 Gazebo를 실행하려는데 여러 오류를 맞이함

ros2 launch robot_gazebo worlds.launch.py

### 오류 1: holonomin_sim 패키지 없음

[ERROR] [launch]: Caught exception in launch (see debug for traceback): "package 'holonomic_sim' not found, searchiung: ['/home/user/rosorin_sim_ws/install'/rosorin_description', '/home/user/rosorin_sim_ws/install/robot_gazebo', '/opt/ros/humble']"

아래 명령어를 통해 어떤 파일에서 해당 패키지를 쓰는지 분석

grep -rn "holonomic_sim" ~/rosorin_sim_ws/src/

알고보니 주석으로 처리된 패키지이고 소스코드에서 쓰이지도 않는데 변수 선언으로 남아있어서 오류를 일으키고 있었음

=> 주석 처리로 해결

### 오류 2: ros_ign_gazebo 없음

오류 1과 동일한 오류이지만, 다른 패키지를 언급

심지어 동일하게 아래 명령어로 관련 파일들을 찾아봤으나, 실제 코드에도 가져다 쓰이고 있는 것으로 보임

grep -rn "ros_ign_gazebo" ~/rosorin_sim_ws/src/

알고 보니 ros_ign_gazebo는 Ignition Gazebo(신세대 Gazebo)를 의미하는데, 이를 토대로 생각했을 때 알고보니 제조사 로봇은 Gazebo Classic을 쓰는 게 아니라 Ignition을 기준으로 작성된 것으로 보임

따라서 아래 명령어로 gazebo ignition을 설치

sudo apt update && sudo apt install ros-humble-ros-ign-gazebo -y

다만, 위의 명령어를 수행 도중 네트워크 환경 이슈가 발생해서 https로 강제 전환 후 설치 진행

```
# 1. ROS 소스 리스트 파일 안의 http를 https로 일괄 변경
sudo sed -i 's/http:\/\/packages.ros.org/https:\/\/packages.ros.org/g' /etc/apt/sources.list.d/*.list

# 2. 다시 업데이트 및 설치 시도
sudo apt update && sudo apt install ros-humble-ros-ign-gazebo -y
```

### 오류 3: 여전히 관련 패키지 없음

동일한 종류의 오류가 몇 가지 종류 계속해서 일어났음

- ign_ros2_control 패키지 없음 => 아래 명령어로 설치
	sudo apt update && sudo apt install ros-humble-ign-ros2-control -y

- joint_state_publisher 패키지 없음 => 아래 명령어로 설치
	sudo apt update && sudo apt install ros-humble-joint-state-publisher -y

- ros_gz_bridge 패키지 없음 => 아래 명령어로 설치
	sudo apt update && sudo apt install ros-humble-ros-gz-bridge -y

## 4. 드디어 성공!

이제 아래 명령어를 실행하니까 정상적으로 로봇과 world가 불러와진다!

ros2 launch robot_gazebo worlds.launch.py

다른 예제 파일도 실행해보면 잘 spawn 된다.

ros2 launch robot_gazebo room_worlds.launch.py

심지어 GPU도 엄청 활용을 잘 하는 걸 볼 수 있었다.

아래 명령어를 입력하면 직접 teleop 명령으로 움직여볼 수도 있다.

ros2 run robot_gazebo teleop_key_control