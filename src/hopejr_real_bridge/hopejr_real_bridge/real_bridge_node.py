import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray
import math
from .config import ARM_MOTORS_LIMITS, HAND_MOTORS_LIMITS

# calculate_jm.py에서 함수 임포트
from .calculate_jm import (
    thumb_joint_to_motor,
    pip_dip_joint_to_motor,
    radial_ulnar_joint_to_motor
)

class RealBridgeNode(Node):
    def __init__(self):
        super().__init__('hopejr_real_bridge')

        # 1. 구독/발행 설정
        self.joint_sub = self.create_subscription(JointState, '/joint_states', self.joint_callback, 10)
        self.arm_pub = self.create_publisher(Float64MultiArray, '/arm_goals', 10)
        self.hand_pub = self.create_publisher(Float64MultiArray, '/hand_goals', 10)

        # 조인트 데이터 저장소
        self.joints = {}
        
        # 실제 로봇 모터 순서 (teleop.py와 동일하게 유지)
        self.HAND_ORDER = [
            "thumb_cmc", "thumb_mcp", "thumb_pip", "thumb_dip",
            "index_radial_flexor", "index_ulnar_flexor", "index_pip_dip",
            "middle_radial_flexor", "middle_ulnar_flexor", "middle_pip_dip",
            "ring_radial_flexor", "ring_ulnar_flexor", "ring_pip_dip",
            "pinky_radial_flexor", "pinky_ulnar_flexor", "pinky_pip_dip"
        ]

        self.get_logger().info("HopeJr Real Bridge Node Started")
    
    def normalize_arm_angle(self, motor_name, angle):
        """각도를 min_limit도에서 max_limit도 사이로 정규화"""
        min_limit, max_limit = None, None
        if motor_name in ARM_MOTORS_LIMITS:
            min_limit, max_limit = ARM_MOTORS_LIMITS[motor_name][0], ARM_MOTORS_LIMITS[motor_name][1]
        normalized_angle = (angle - min_limit) / (max_limit - min_limit) * 200.0 - 100.0
        if normalized_angle < -100.0:
            normalized_angle = -100.0
        elif normalized_angle > 100.0:
            normalized_angle = 100.0
        return normalized_angle
    
    def normalize_hand_angle(self, motor_name, angle):
        """각도를 min_limit도에서 max_limit도 사이로 정규화"""
        min_limit, max_limit = None, None
        if motor_name in HAND_MOTORS_LIMITS:
            min_limit, max_limit = HAND_MOTORS_LIMITS[motor_name][0], HAND_MOTORS_LIMITS[motor_name][1]
        normalized_angle = (angle - min_limit) / (max_limit - min_limit) * 100.0
        if normalized_angle < 0.0:
            normalized_angle = 0.0
        elif normalized_angle > 100.0:
            normalized_angle = 100.0
        return normalized_angle

    def joint_callback(self, msg):
        # 1. 수신된 JointStates 저장
        for name, pos in zip(msg.name, msg.position):
            self.joints[name] = pos
        
        if not self.joints: return

        # 2. [ARM] 계산: 7개 조인트 (각도 그대로 사용)
        arm_msg = Float64MultiArray()
        arm_names = ['shoulder_pitch', 'shoulder_yaw', 'shoulder_roll', 'elbow_flex', 'wrist_roll', 'wrist_yaw', 'wrist_pitch']
        # 시뮬레이션(Radian) -> 실제 로봇(Degree)
        arm_msg.data = [float(math.degrees(self.joints.get(j, 0.0))) for j in arm_names]
        for i, name in enumerate(arm_names):
            arm_msg.data[i] = self.normalize_arm_angle(name, arm_msg.data[i])
        self.arm_pub.publish(arm_msg)

        # 3. [HAND] 계산: 16개 모터 (calculate_jm 사용)
        hand_results = {}

        # (A) 엄지: 4개 관절 -> 4개 모터
        thumb_list = ['thumb_mcp_2', 'thumb_pip_1', 'thumb_pip_2', 'thumb_dip']
        thumb_motor_list = ['thumb_cmc', 'thumb_mcp', 'thumb_pip', 'thumb_dip']
        t_pos = [self.joints.get(joint, 0.0) for joint in thumb_list]
        hand_results[thumb_motor_list[0]] = self.normalize_hand_angle(thumb_motor_list[0], t_pos[0])
        for i in range(1, len(t_pos)):
            hand_results[thumb_motor_list[i]] = self.normalize_hand_angle(thumb_motor_list[i], thumb_joint_to_motor(t_pos[i]))

        # (B) 나머지 4개 손가락
        for f in ['index', 'middle', 'ring', 'pinky']:
            # i. Radial/Ulnar: mcp_2(alpha)와 pip(beta) 기반 실 계산
            alpha = self.joints.get(f'{f}_mcp_2', 0.0)
            beta = self.joints.get(f'{f}_pip', 0.0)
            tr, tu = radial_ulnar_joint_to_motor(alpha, beta)
            hand_results[f'{f}_radial_flexor'] = self.normalize_hand_angle(f'{f}_radial_flexor', tr)
            hand_results[f'{f}_ulnar_flexor'] = self.normalize_hand_angle(f'{f}_ulnar_flexor', tu)

            # ii. PIP-DIP: dip_1 기반 모터 계산
            dip_val = self.joints.get(f'{f}_dip_1', 0.0)
            hand_results[f'{f}_pip_dip'] = self.normalize_hand_angle(f'{f}_pip_dip', pip_dip_joint_to_motor(dip_val))

        # 4. 정해진 순서대로 메시지 구성 및 발행
        hand_msg = Float64MultiArray()
        hand_msg.data = [float(hand_results.get(name, 0.0)) for name in self.HAND_ORDER]
        self.hand_pub.publish(hand_msg)

def main():
    rclpy.init()
    node = RealBridgeNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()