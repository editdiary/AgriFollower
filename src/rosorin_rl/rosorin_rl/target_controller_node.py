"""target_controller_node.py — 작업자(타겟) 원기둥을 움직이는 컨트롤러 노드.

설계 출처: docs/rl_design/rl_train_senarioes.md §2 (학습 시나리오)
roadmap 2단계 (a): "이동 원기둥 + 컨트롤러 노드(직진/지그재그·무작위 속도)"

[ 구동 방식: set_pose 키네마틱 ]
- 노드가 작업자의 pose 를 자체 적분(x += v·dt)으로 관리하고, 20Hz 로
  /world/greenhouse_world/set_pose (브리지된 Ignition 서비스)에 보내 갱신한다.
- worker_target 은 static 모델이라 물리의 영향을 받지 않는다 — 넘어짐·밀림 없음.
  0.25 m/s ÷ 20Hz = 틱당 1.25cm 라 LiDAR(10Hz)에는 연속 이동으로 보인다.
- ⚠️ 처음에는 velocity-control 플러그인으로 시도했으나 Fortress 에서 link 레벨
  gravity off 가 무시되어 스폰 직후 원기둥이 넘어짐 → 이 방식으로 전환.

[ 시나리오 확장 구조 (전략 패턴) ]
- GaitStrategy 를 상속한 클래스가 "이번 틱의 속도"를 결정한다.
- 현재는 시나리오 1(ConstantWalk: 정속 왕복)만 구현.
- 시나리오 2(Stop&Go)·3(후진 접근)·4(지그재그)·5(U턴)는 같은 인터페이스로
  클래스를 추가하고 STRATEGIES 딕셔너리에 등록하면 된다.

[ 에피소드 리셋 연동 ]
- RL 환경(follow_env.py)이 /worker/reset (std_srvs/Empty) 서비스를 호출하면
  내부 pose 를 시작 위치(reset_x/y)로 되돌리고 걸음 상태를 초기화한다.
"""

import math

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import PoseStamped
from std_srvs.srv import Empty
from ros_gz_interfaces.srv import SetEntityPose
from ros_gz_interfaces.msg import Entity


class GaitStrategy:
    """타겟 걸음새(시나리오)의 공통 인터페이스.

    compute(x, y, dt) -> (vx, vy)
      현재 위치를 받아 이번 틱의 월드 좌표계 속도를 반환한다.
    reset()
      에피소드 시작 시 내부 상태(진행 방향 등)를 초기화한다.
    """

    def compute(self, x, y, dt):
        raise NotImplementedError

    def reset(self):
        pass


class ConstantWalk(GaitStrategy):
    """시나리오 1 — 정속 주행 (rl_train_senarioes.md §2 시나리오 1).

    통로(+x 방향)를 일정 속도로 걷다가, 통로 끝(x_max)에 도달하면
    반대 방향으로 돌아 걸어온다(왕복).
    """

    def __init__(self, speed, x_min, x_max):
        self.speed = speed       # [m/s] 보행 속도
        self.x_min = x_min       # [m] 왕복 구간 시작
        self.x_max = x_max       # [m] 왕복 구간 끝
        self.direction = +1.0    # +1: x 증가 방향, -1: 감소 방향

    def reset(self):
        self.direction = +1.0    # 에피소드 시작은 항상 전진 방향부터

    def compute(self, x, y, dt):
        # 통로 끝에 도달하면 방향 반전
        if self.direction > 0 and x >= self.x_max:
            self.direction = -1.0
        elif self.direction < 0 and x <= self.x_min:
            self.direction = +1.0
        return (self.direction * self.speed, 0.0)


# 시나리오 번호 → 전략 클래스 매핑. 새 시나리오는 여기에 등록.
#   2: StopGo, 3: Backtrack, 4: Zigzag, 5: UTurn (추후 구현)
STRATEGIES = {
    1: ConstantWalk,
}


class WorkerController(Node):
    """작업자 원기둥 키네마틱 컨트롤러 노드 (set_pose 기반)."""

    RATE_HZ = 20.0       # pose 갱신 주기 (RL 제어 10Hz 보다 촘촘하게)
    WORLD = 'greenhouse_world'
    BODY_Z = 0.8         # 원기둥(높이 1.6m) 중심 높이 — 바닥에 서 있는 자세

    def __init__(self):
        super().__init__('worker_controller')

        # --- 파라미터 (launch 또는 CLI 에서 덮어쓰기 가능) ---
        self.declare_parameter('scenario', 1)
        self.declare_parameter('speed', 0.25)        # [m/s]
        self.declare_parameter('aisle_x_min', 0.9)   # [m]
        self.declare_parameter('aisle_x_max', 6.3)   # [m]
        self.declare_parameter('reset_x', 1.5)       # [m] 에피소드 시작 위치
        self.declare_parameter('reset_y', 0.0)

        scenario = self.get_parameter('scenario').value
        speed = self.get_parameter('speed').value
        x_min = self.get_parameter('aisle_x_min').value
        x_max = self.get_parameter('aisle_x_max').value
        self.reset_x = self.get_parameter('reset_x').value
        self.reset_y = self.get_parameter('reset_y').value

        if scenario not in STRATEGIES:
            raise ValueError(f'시나리오 {scenario} 은(는) 아직 구현되지 않음. '
                             f'가능: {list(STRATEGIES)}')
        self.gait = STRATEGIES[scenario](speed, x_min, x_max)

        # 내부 키네마틱 상태 — 이 값이 작업자 위치의 단일 출처
        self.x = self.reset_x
        self.y = self.reset_y

        # --- 통신 설정 ---
        # Ignition 엔티티 텔레포트 서비스 (브리지: rl_sim.launch.py 의 rl_bridge)
        self.set_pose_cli = self.create_client(
            SetEntityPose, f'/world/{self.WORLD}/set_pose')

        # 작업자 ground-truth pose 발행 — 이 노드의 내부 적분값이 단일 출처.
        # (static 모델은 /world/.../dynamic_pose/info 에 안 나오므로 직접 발행한다.
        #  target_feature_node 가 이 토픽을 구독해 타겟 특징을 계산.)
        self.pose_pub = self.create_publisher(PoseStamped, '/worker/pose', 10)

        # 에피소드 리셋 서비스 (RL 환경이 호출)
        self.create_service(Empty, '/worker/reset', self._on_reset)

        # 제어 타이머 (sim time 기준 — use_sim_time=True 를 launch 에서 지정)
        self.create_timer(1.0 / self.RATE_HZ, self._on_tick)

        self.get_logger().info(
            f'WorkerController 시작 — 시나리오 {scenario}, 속도 {speed} m/s, '
            f'왕복 구간 x∈[{x_min}, {x_max}], 시작 ({self.reset_x}, {self.reset_y})')

    def _on_reset(self, request, response):
        """RL 환경의 에피소드 리셋 → 위치·걸음 상태 초기화."""
        self.gait.reset()
        self.x = self.reset_x
        self.y = self.reset_y
        self.get_logger().info('작업자 위치·걸음 상태 리셋')
        return response

    def _on_tick(self):
        """주기적으로: 속도 적분 → set_pose 로 위치 갱신."""
        if not self.set_pose_cli.service_is_ready():
            return  # 브리지가 아직 안 떠 있음 (기동 직후)

        # ① 현재 전략이 정한 속도로 내부 pose 적분
        dt = 1.0 / self.RATE_HZ
        vx, vy = self.gait.compute(self.x, self.y, dt)
        self.x += vx * dt
        self.y += vy * dt

        # ② Ignition 에 텔레포트 요청 (응답은 기다리지 않음 — 20Hz 유지)
        req = SetEntityPose.Request()
        req.entity = Entity(name='worker_target', type=Entity.MODEL)
        req.pose.position.x = self.x
        req.pose.position.y = self.y
        req.pose.position.z = self.BODY_Z
        req.pose.orientation.w = 1.0   # 항상 직립 자세 유지
        self.set_pose_cli.call_async(req)

        # ③ ground-truth pose 토픽 발행 (target_feature_node 용)
        ps = PoseStamped()
        ps.header.stamp = self.get_clock().now().to_msg()
        ps.header.frame_id = 'world'
        ps.pose.position.x = self.x
        ps.pose.position.y = self.y
        ps.pose.position.z = self.BODY_Z
        ps.pose.orientation.w = 1.0
        self.pose_pub.publish(ps)


def main(args=None):
    rclpy.init(args=args)
    node = WorkerController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
