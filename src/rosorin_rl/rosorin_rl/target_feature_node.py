"""target_feature_node.py — 타겟(작업자) 특징 토픽 발행 노드.

설계 출처: docs/rl_design/rl_state_space.md §2.1 (RGB-D 타겟 특징 4차원)
roadmap 2단계 (b): "ground-truth pose 에서 타겟 특징 산출 + 가우시안 노이즈 주입"

[ Sim-to-Real 핵심 설계 ]
실물 로봇에서는 RGB-D 영상에서 마커(AprilTag)를 검출하는 비전 노드가
이 토픽(/target/features)을 발행하게 된다. 시뮬에서는 무거운 비전 연산 대신
ground-truth pose 로 같은 값을 역산해 발행한다.
→ RL 환경(follow_env.py)은 이 토픽만 구독하므로,
  나중에 발행자만 비전 노드로 교체하면 학습 코드는 한 줄도 안 바뀐다 (roadmap 5단계).

발행 메시지 (std_msgs/Float32MultiArray, /target/features, 15Hz):
  data = [x_norm, y_norm, d_t, theta_t, visible]
   - x_norm, y_norm: 화면 중심 기준 정규화 픽셀 위치 [-1.5, 1.5]
   - d_t:     카메라→마커 3D 실측 거리 [m]
   - theta_t: 수평 상대 각도 [rad]
   - visible: 1.0(검출) / 0.0(미검출 — FOV 밖)
  미검출 프레임에는 마지막으로 본 값(last-known)을 visible=0 과 함께 보낸다.
  (rl_state_space.md §4 우려1 의 '추적 유지' 아이디어의 단순화 버전 —
   칼만 필터 대신 last-known 유지. 에이전트는 visible 신호 자체는 받지 않고
   env 가 '연속 미검출 스텝 수'로 타겟 이탈을 판정하는 데 쓴다.)

도메인 랜덤화 (proposal §4.4):
  발행 직전 가우시안 노이즈를 섞는다 (기본 3%):
   - d_t: 곱셈 노이즈 d_t·(1+ε), ε~N(0, σ)  → 거리가 멀수록 오차 커짐 (실제 depth 특성)
   - x_norm, y_norm, theta_t: 덧셈 노이즈 (스케일 맞춰 σ 조정)
"""

import numpy as np

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Float32MultiArray
from tf2_msgs.msg import TFMessage

from rosorin_rl.geometry_utils import extract_pose, world_to_target_features


class TargetFeatureNode(Node):
    """ground-truth pose → 타겟 특징 [x_norm, y_norm, d_t, θ_t, visible] 발행."""

    RATE_HZ = 15.0  # 발행 주기 (RL 제어 루프 10Hz 보다 약간 촘촘하게)

    def __init__(self):
        super().__init__('target_feature_node')

        # --- 파라미터 ---
        self.declare_parameter('noise_pct', 0.03)      # 가우시안 노이즈 표준편차 (3%)
        self.declare_parameter('marker_height', 0.20)  # 마커 가정 높이 [m]
        self.declare_parameter('robot_entity', 'robot')         # ignition 모델 이름

        self.noise_pct = self.get_parameter('noise_pct').value
        self.marker_z = self.get_parameter('marker_height').value
        self.robot_name = self.get_parameter('robot_entity').value

        self.rng = np.random.default_rng()

        # --- 상태 ---
        self.robot_pose = None    # (x, y, z, yaw)
        self.target_pose = None
        # 마지막으로 '검출'됐을 때의 특징 (미검출 프레임에 재사용)
        self.last_known = [0.0, 0.0, 1.0, 0.0]  # [x_norm, y_norm, d_t, theta_t]

        # --- 통신 ---
        # 로봇 pose: dynamic_pose/info (동적 모델의 ground truth)
        self.create_subscription(
            TFMessage, '/world/greenhouse_world/dynamic_pose/info',
            self._on_pose, 10)
        # 타겟 pose: 컨트롤러의 키네마틱 적분값 (static 모델은 dynamic_pose 에 없음)
        self.create_subscription(PoseStamped, '/worker/pose', self._on_worker, 10)
        self.pub = self.create_publisher(Float32MultiArray, '/target/features', 10)
        self.create_timer(1.0 / self.RATE_HZ, self._on_tick)

        self.get_logger().info(
            f'TargetFeatureNode 시작 — 노이즈 {self.noise_pct*100:.0f}%, '
            f'마커 높이 {self.marker_z}m')

    def _on_pose(self, msg):
        """dynamic_pose/info 에서 로봇 pose 를 갱신."""
        rp = extract_pose(msg, self.robot_name)
        if rp is not None:
            self.robot_pose = rp

    def _on_worker(self, msg):
        """컨트롤러가 발행하는 작업자 키네마틱 pose 갱신."""
        p = msg.pose.position
        self.target_pose = (p.x, p.y, p.z, 0.0)

    def _on_tick(self):
        """특징 계산 → 노이즈 주입 → 발행."""
        if self.robot_pose is None or self.target_pose is None:
            return  # sim 기동 직후 pose 미수신

        rx, ry, _, ryaw = self.robot_pose
        tx, ty, _, _ = self.target_pose

        feat = world_to_target_features(rx, ry, ryaw, tx, ty, marker_z=self.marker_z)

        if feat['visible']:
            # --- 도메인 랜덤화: 가우시안 노이즈 주입 ---
            s = self.noise_pct
            d_t = feat['d_t'] * (1.0 + self.rng.normal(0.0, s))      # 곱셈 노이즈
            x_norm = feat['x_norm'] + self.rng.normal(0.0, s)        # 덧셈 노이즈
            y_norm = feat['y_norm'] + self.rng.normal(0.0, s)
            theta_t = feat['theta_t'] + self.rng.normal(0.0, s * 0.5)
            self.last_known = [x_norm, y_norm, max(0.05, d_t), theta_t]
            visible = 1.0
        else:
            # 미검출: 마지막으로 본 값 유지 + visible=0
            visible = 0.0

        msg = Float32MultiArray()
        msg.data = [float(v) for v in self.last_known] + [visible]
        self.pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = TargetFeatureNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
