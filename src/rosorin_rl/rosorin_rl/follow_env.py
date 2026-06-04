"""follow_env.py — 작업자 추종 커스텀 Gymnasium 환경 (ROS 2 ⇄ Ignition 브리지).

설계 출처: docs/rl_design/0_project_proposal.md §4.3 (The Bridge), §5 (MDP)
           docs/roadmap.md 3단계

[ 구조 개요 ]
  "Gazebo 시뮬레이터"와 "Stable-Baselines3(파이썬)"는 서로 다른 프로그램이다.
  이 클래스가 둘 사이의 다리(Bridge) 역할을 한다:

    SB3 에이전트 ──action──▶ FollowTargetEnv.step()
                              │  ① [-1,1]³ 행동 → Twist 스케일링 → /controller/cmd_vel 발행
                              │  ② 0.1초(sim time) 대기 — 로봇이 실제로 움직이는 시간
                              │  ③ 최신 센서값 수집 → 16차원 정제 → 3프레임 스택(48차원)
                              │  ④ 보상·종료 판정 (reward.py)
    SB3 에이전트 ◀─obs,reward─┘

  ROS 통신은 내부에 생성한 rclpy 노드가 담당하고, 그 노드는 백그라운드 스레드의
  Executor 에서 spin 된다. 센서 콜백은 최신 메시지를 Lock 으로 보호하며 저장만 하고,
  step() 이 필요할 때 스냅샷을 떠서 사용한다 (멀티스레드 안전).

[ 사용 예 ]
    rclpy.init()
    env = FollowTargetEnv()          # rl_sim.launch.py 가 떠 있는 상태에서
    obs, info = env.reset()
    obs, r, term, trunc, info = env.step(env.action_space.sample())
"""

import math
import os
import threading
import time

import numpy as np
import yaml

import gymnasium as gym
from gymnasium import spaces

import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor

from ament_index_python.packages import get_package_share_directory

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Image, LaserScan
from std_msgs.msg import Float32MultiArray
from std_srvs.srv import Empty
from tf2_msgs.msg import TFMessage
from ros_gz_interfaces.srv import SetEntityPose
from ros_gz_interfaces.msg import Entity

from rosorin_rl.geometry_utils import AISLE_HEADING, extract_pose
from rosorin_rl.obs_pipeline import ObsBuilder
from rosorin_rl.reward import RewardCalculator


def _default_config_path():
    """설치된 패키지 share 디렉토리의 기본 설정 파일 경로."""
    return os.path.join(get_package_share_directory('rosorin_rl'),
                        'config', 'rl_params.yaml')


class FollowTargetEnv(gym.Env):
    """작업자(타겟) 추종 강화학습 환경.

    - 관측: 48차원 (16차원 × 3프레임 스택) — rl_state_space.md
    - 행동: [ax, ay, aω] ∈ [-1,1]³ → 매카넘 vx/vy/ω — proposal §5.2
    - 보상/종료: reward.py — rl_reward_function.md
    """

    metadata = {'render_modes': []}

    WORLD = 'greenhouse_world'

    def __init__(self, config_path=None):
        super().__init__()

        # ---------------- 설정 로드 ----------------
        with open(config_path or _default_config_path(), 'r') as f:
            self.cfg = yaml.safe_load(f)

        self.v_max = self.cfg['action']['v_max']
        self.w_max = self.cfg['action']['w_max']
        self.dt = 1.0 / self.cfg['control']['rate_hz']           # 제어 주기 [s]
        self.step_timeout = self.cfg['control']['step_timeout_sec']

        # footprint: 직사각형 로봇의 방향별 외곽 거리 — 충돌 판정(env_margin)에 필요
        self.obs_builder = ObsBuilder(self.cfg['obs'],
                                      footprint=self.cfg['robot']['footprint'])
        self.reward_calc = RewardCalculator(self.cfg)

        # ---------------- Gym 공간 정의 ----------------
        # 행동: 정규화된 3자유도 연속 (proposal §5.2)
        self.action_space = spaces.Box(low=-1.0, high=1.0, shape=(3,), dtype=np.float32)
        # 관측: 48차원. 거리류는 0~12m, 정규화 픽셀은 ±1.5, 속도는 ±1.5 정도라
        # 보수적으로 [-15, 15] 범위로 잡는다 (SB3 MlpPolicy 는 범위를 학습에 쓰진 않음).
        self.observation_space = spaces.Box(low=-15.0, high=15.0, shape=(48,),
                                            dtype=np.float32)

        # ---------------- ROS 노드/통신 구성 ----------------
        if not rclpy.ok():
            rclpy.init()
        self.node = Node('follow_env',
                         parameter_overrides=[
                             rclpy.parameter.Parameter('use_sim_time', value=True)])

        self._lock = threading.Lock()   # 콜백(수신 스레드) ↔ step(메인 스레드) 보호
        self._scan = None               # 최신 LaserScan
        self._depth = None              # 최신 depth Image
        self._odom = None               # 최신 Odometry
        self._target_feat = None        # 최신 /target/features (list 5개)
        self._robot_yaw = 0.0           # ground-truth 로봇 yaw (dynamic_pose 에서)

        self.node.create_subscription(LaserScan, '/scan', self._on_scan, 10)
        self.node.create_subscription(Image, '/depth_cam/depth_image',
                                      self._on_depth, 10)
        self.node.create_subscription(Odometry, '/odom', self._on_odom, 10)
        self.node.create_subscription(Float32MultiArray, '/target/features',
                                      self._on_target, 10)
        self.node.create_subscription(TFMessage,
                                      f'/world/{self.WORLD}/dynamic_pose/info',
                                      self._on_world_pose, 10)

        self.cmd_pub = self.node.create_publisher(Twist, '/controller/cmd_vel', 10)

        # Ignition 엔티티 텔레포트 서비스 (브리지: ros_gz_interfaces/SetEntityPose)
        # ⚠️ Gazebo Classic 의 /reset_world 가 아님 — Ignition Fortress 전용 경로.
        self.set_pose_cli = self.node.create_client(
            SetEntityPose, f'/world/{self.WORLD}/set_pose')
        # 작업자 걸음 상태 리셋 (target_controller_node.py)
        self.worker_reset_cli = self.node.create_client(Empty, '/worker/reset')

        # ---------------- Executor 백그라운드 스레드 ----------------
        self.executor = MultiThreadedExecutor()
        self.executor.add_node(self.node)
        self._spin_thread = threading.Thread(target=self.executor.spin, daemon=True)
        self._spin_thread.start()

        self.step_idx = 0

    # ==================================================================
    # 센서 콜백 — 최신 메시지를 저장만 한다 (가공은 step 에서)
    # ==================================================================
    def _on_scan(self, msg):
        with self._lock:
            self._scan = msg

    def _on_depth(self, msg):
        with self._lock:
            self._depth = msg

    def _on_odom(self, msg):
        with self._lock:
            self._odom = msg

    def _on_target(self, msg):
        with self._lock:
            self._target_feat = list(msg.data)

    def _on_world_pose(self, msg):
        p = extract_pose(msg, 'robot')
        if p is not None:
            with self._lock:
                self._robot_yaw = p[3]

    # ==================================================================
    # 시간 유틸 — sim time 기준 대기
    # ==================================================================
    def _sim_now(self):
        """현재 sim time [s] (/clock 기반 — use_sim_time=True)."""
        return self.node.get_clock().now().nanoseconds * 1e-9

    def _sleep_sim(self, duration):
        """sim time 으로 duration 초 대기.

        wall time 의 time.sleep 을 쓰면 시뮬이 느려질 때(RTF<1) 로봇이 의도보다
        조금만 움직인 채 관측하게 되어 transition 이 비일관해진다.
        sim clock 이 duration 만큼 흐를 때까지 폴링하되, sim 이 멈춘 경우를 대비해
        wall time 안전 타임아웃을 둔다.
        """
        t_start_sim = self._sim_now()
        t_start_wall = time.monotonic()
        while self._sim_now() - t_start_sim < duration:
            time.sleep(0.002)
            if time.monotonic() - t_start_wall > self.step_timeout:
                self.node.get_logger().warn(
                    f'sim time 이 {self.step_timeout}s(wall) 동안 진행되지 않음 — '
                    'Gazebo 가 일시정지 상태인지 확인하세요.')
                break

    def _wait_for_sensors(self, timeout=30.0):
        """모든 필수 토픽의 첫 메시지가 도착할 때까지 블록 (기동/리셋 직후)."""
        t0 = time.monotonic()
        while True:
            with self._lock:
                ready = (self._scan is not None and self._depth is not None
                         and self._odom is not None and self._target_feat is not None)
            if ready:
                return
            if time.monotonic() - t0 > timeout:
                missing = []
                with self._lock:
                    if self._scan is None:
                        missing.append('/scan')
                    if self._depth is None:
                        missing.append('/depth_cam/depth_image')
                    if self._odom is None:
                        missing.append('/odom')
                    if self._target_feat is None:
                        missing.append('/target/features')
                raise TimeoutError(
                    f'센서 토픽 수신 실패: {missing} — rl_sim.launch.py 가 떠 있고 '
                    '브리지가 정상인지 확인하세요.')
            time.sleep(0.1)

    # ==================================================================
    # 리셋 보조 — Ignition set_pose 텔레포트
    # ==================================================================
    def _teleport(self, entity_name, x, y, z, yaw=0.0):
        """엔티티를 지정 pose 로 순간이동 (Ignition set_pose 서비스)."""
        if not self.set_pose_cli.wait_for_service(timeout_sec=5.0):
            raise RuntimeError(f'/world/{self.WORLD}/set_pose 서비스 없음 — '
                               'RL 브리지(rl_sim.launch.py)가 떠 있는지 확인.')
        req = SetEntityPose.Request()
        req.entity = Entity(name=entity_name, type=Entity.MODEL)
        req.pose.position.x = float(x)
        req.pose.position.y = float(y)
        req.pose.position.z = float(z)
        req.pose.orientation.z = math.sin(yaw / 2.0)
        req.pose.orientation.w = math.cos(yaw / 2.0)
        future = self.set_pose_cli.call_async(req)
        # executor 스레드가 spin 중이므로 future 완료를 폴링으로 대기
        t0 = time.monotonic()
        while not future.done():
            time.sleep(0.01)
            if time.monotonic() - t0 > 5.0:
                raise TimeoutError(f'set_pose({entity_name}) 응답 없음')
        if not future.result().success:
            self.node.get_logger().warn(f'set_pose({entity_name}) 실패 응답')

    # ==================================================================
    # Gymnasium API
    # ==================================================================
    def reset(self, *, seed=None, options=None):
        """에피소드 초기화: 로봇·타겟 위치 복원 + 내부 버퍼 비우기."""
        super().reset(seed=seed)

        # ① 로봇 정지 (잔여 속도 제거)
        self.cmd_pub.publish(Twist())

        # ② 작업자 리셋: 컨트롤러(target_controller_node)가 내부 키네마틱 pose 를
        #    시작 위치로 되돌리고 다음 틱(50ms 내)에 set_pose 로 반영한다.
        if self.worker_reset_cli.wait_for_service(timeout_sec=2.0):
            self.worker_reset_cli.call_async(Empty.Request())
        else:
            self.node.get_logger().warn('/worker/reset 서비스 없음 — '
                                        'target_controller 가 떠 있는지 확인.')

        # ③ 로봇 텔레포트 (Ignition Fortress set_pose — roadmap 2단계 (d))
        rb = self.cfg['robot']
        self._teleport('robot', rb['reset_x'], rb['reset_y'], rb['reset_z'],
                       rb['reset_yaw'])

        # ④ 내부 상태 초기화
        self.obs_builder.reset()
        self.reward_calc.reset()
        self.step_idx = 0

        # ⑤ 텔레포트 이후의 '신선한' 센서값을 기다림.
        #    순서 중요: 먼저 안정화 시간을 주고 → 버퍼를 비우고 → 새 수신을 기다린다.
        #    (텔레포트 직전에 렌더된 영상/스캔이 브리지를 타고 늦게 도착하는
        #     stale 프레임이 첫 관측에 섞이는 것을 방지)
        self._sleep_sim(0.3)          # 물리/센서 안정화 시간
        with self._lock:
            self._scan = self._depth = self._odom = self._target_feat = None
        self._wait_for_sensors()

        obs, _ = self._collect_obs()
        return obs, {}

    def step(self, action):
        """행동 1스텝 실행 (proposal §4.3 의 step 흐름 그대로)."""
        # ① 정규화 행동 → 물리 속도 스케일링 → 발행 (proposal §5.2)
        a = np.clip(np.asarray(action, dtype=np.float32), -1.0, 1.0)
        cmd = Twist()
        cmd.linear.x = float(a[0] * self.v_max)    # 전/후진
        # ⚠️ 시뮬 보정: 벤더 MecanumDrive 플러그인 구성이 linear.y 부호를 REP-103
        #    (+y=좌측)과 반대로 해석하는 것을 검증으로 확인(+명령→우측 이동).
        #    vx·ω 는 정상. 에이전트 행동 ay 의 의미(+1=좌측)를 지키기 위해 발행
        #    직전에 부호를 뒤집는다. 실물 로봇은 REP-103 을 따르므로 Sim-to-Real
        #    포팅 시 이 보정만 제거하면 된다. (docs/rl_code_guide.md 참조)
        cmd.linear.y = float(-a[1] * self.v_max)   # 좌/우 게걸음 (매카넘)
        cmd.angular.z = float(a[2] * self.w_max)   # 제자리 회전
        self.cmd_pub.publish(cmd)

        # ② 로봇이 움직일 시간을 줌 (sim time 0.1초)
        self._sleep_sim(self.dt)

        # ③ 관측 수집·가공
        obs, raw = self._collect_obs()

        # ④ 보상·종료 판정
        reward, terminated, info = self.reward_calc.compute(
            d_t=raw['d_t'],
            visible=raw['visible'],
            d_front_merged=raw['d_front_merged'],
            d_left=raw['d_left'],
            d_right=raw['d_right'],
            yaw_err=raw['yaw_err'],
            env_margin=raw['env_margin'],
            step_idx=self.step_idx,
        )
        self.step_idx += 1

        if terminated:
            # 에피소드 종료 시 로봇 정지 (다음 reset 까지 관성 주행 방지)
            self.cmd_pub.publish(Twist())
            self.node.get_logger().info(
                f"에피소드 종료: {info['terminal']} (step {self.step_idx})")

        # truncated 는 사용하지 않음 — '최대 스텝 생존'을 success 종료로 다루기 때문
        return obs, float(reward), terminated, False, info

    def close(self):
        """학습 종료 시 자원 정리."""
        self.cmd_pub.publish(Twist())
        self.executor.shutdown()
        self.node.destroy_node()

    # ==================================================================
    # 관측 수집: 최신 센서 스냅샷 → 16차원 → 48차원 + 보상용 원시값
    # ==================================================================
    def _collect_obs(self):
        with self._lock:
            scan = self._scan
            depth = self._depth
            odom = self._odom
            tfeat = self._target_feat
            robot_yaw = self._robot_yaw

        # --- 타겟 특징 4D + 가시성 (target_feature_node 발행) ---
        x_norm, y_norm, d_t, theta_t, vis = tfeat
        visible = vis > 0.5

        # --- LiDAR 6구역 + 측면 거리 (obs_pipeline) ---
        lidar6 = self.obs_builder.sector_minima(scan.ranges)
        d_left, d_right = self.obs_builder.side_distances(scan.ranges)

        # --- 하단 뎁스 범퍼 3D ---
        depth3 = self.obs_builder.depth_bumper(depth)

        # --- 로봇 속도 3D (/odom twist — 적분 드리프트와 무관한 순간 속도) ---
        vel3 = [odom.twist.twist.linear.x,
                odom.twist.twist.linear.y,
                odom.twist.twist.angular.z]

        # --- 16차원 프레임 → 3프레임 스택(48차원) ---
        frame = self.obs_builder.build_frame(
            [x_norm, y_norm, d_t, theta_t], depth3, lidar6, vel3)
        obs = self.obs_builder.stack(frame).astype(np.float32)

        # --- 보상 계산용 원시값 ---
        # 전방 센서 퓨전 (rl_reward_function.md §3.2):
        #   LiDAR 정면 구역(인덱스 1)과 뎁스범퍼 중앙(인덱스 1)의 min
        d_front_merged = float(min(lidar6[1], depth3[1]))
        # 헤딩 오차: 통로 방향(월드 +x) 대비. [-π, π] 로 정규화.
        yaw_err = (robot_yaw - AISLE_HEADING + math.pi) % (2 * math.pi) - math.pi
        # 환경 충돌 판정 (3차 수정): 직사각형 풋프린트 기준 '외곽 여유'.
        #   LiDAR 는 빔별 풋프린트 반경을 빼서 계산하고, 하단 뎁스범퍼(전방)는
        #   카메라가 전방 외곽 근처에 있으므로 그대로 외곽 여유로 취급해 min 융합
        #   (LiDAR 평면 아래 사각지대 장애물도 충돌 판정에 포함).
        env_margin = min(self.obs_builder.env_margin(scan.ranges),
                         float(depth3.min()))

        raw = {'d_t': d_t, 'visible': visible,
               'd_front_merged': d_front_merged,
               'd_left': d_left, 'd_right': d_right,
               'yaw_err': yaw_err, 'env_margin': env_margin}
        return obs, raw
