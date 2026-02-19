import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray, Float64MultiArray
import numpy as np

class RobotHandDriver(Node):
    def __init__(self):
        super().__init__('ah_bot_driver')
        
        # Vision에서 0~1 값을 받음
        self.sub = self.create_subscription(Float32MultiArray, '/hand_control', self.callback, 10)
        # MuJoCo로 라디안 값을 보냄
        self.pub = self.create_publisher(Float64MultiArray, '/joint_commands', 10)

        # [전문가 설정] 시뮬레이션 모드 여부
        # True: 시뮬레이션용 (안전장치 해제, 부드러운 움직임 위주)
        # False: 실물 로봇용 (와이어 끊어짐 방지 안전장치 활성화)
        self.SIM_MODE = True 

        # [설정] AmazingHand 하드웨어 스펙
        # MuJoCo 모델에 맞춰 범위 조정 (Degree)
        self.RANGE_BEND = [0.0, 90.0]   # [펴짐, 굽힘] 
        self.RANGE_SIDE = [-30.0, 30.0] # [좌, 우]

        # 모터 오프셋 (순서: 검지M1,M2, 중지M1,M2, 약지M1,M2, 엄지M1,M2)
        # 필요하다면 나중에 이 값을 튜닝해서 초기 자세를 잡으세요.
        self.MOTOR_OFFSETS = [
            0.0, 0.0,    # 검지
            0.0, 0.0,    # 중지
            0.0, 0.0,    # 약지
            0.0, 0.0     # 엄지
        ]

        # 스무딩(LPF)을 위한 이전 값 저장
        self.prev_motor = np.zeros(8) 
        self.FILTER_ALPHA = 0.2  # 0.0 ~ 1.0 (클수록 빠르지만 떨림 심함)

        self.get_logger().info('🧠 Driver Node Ready: Physics Logic Loaded')

    def callback(self, msg):
        inputs = msg.data # [B0, S0, B1, S1, B2, S2, B3, S3] (0.0 ~ 1.0)
        if len(inputs) != 8: return

        joint_commands = [] # MuJoCo로 보낼 8개 모터 값 (radian)

        for i in range(4):
            # 1. 입력값 파싱
            in_bend_ratio = inputs[i*2]   # 0.0 ~ 1.0
            in_side_ratio = inputs[i*2+1] # -1.0 ~ 1.0 (엄지는 0~1)

            # 2. 각도 변환 (Mapping)
            # 0~1 입력을 실제 로봇 각도(Degree)로 변환
            target_bend = np.interp(in_bend_ratio, [0.0, 1.0], self.RANGE_BEND)
            
            # 좌우 각도 변환
            if i == 3: # 엄지
                target_side_raw = np.interp(in_side_ratio, [0.0, 1.0], [0, 60])
            else:      # 검지, 중지, 약지
                target_side_raw = np.interp(in_side_ratio, [-1.0, 1.0], self.RANGE_SIDE)

            # 3. 로직 적용 (안전장치 및 방향 보정)
            target_side = target_side_raw

            # 엄지가 아닌 손가락들(검지, 중지, 약지)에 대한 처리
            if i != 3:
                # (1) 방향 보정 (하드웨어 특성에 따라 반전)
                if i == 0: target_side = -target_side # 검지 반전
                if i == 1: target_side = -target_side # 중지 반전
                if i == 2: target_side = -target_side # 약지 반전

                # (2) 안전장치 (SIM_MODE가 아닐 때만 작동)
                if not self.SIM_MODE:
                    # 굽힘이 심하면 좌우 움직임 제한 (와이어 이탈 방지)
                    if in_bend_ratio > 0.4:
                        target_side = 0.0

            # 4. 모터 믹싱 (Mixing) - AmazingHand의 차동 구동 핵심
            # M1 = Side + Bend
            # M2 = Side - Bend
            # (움직임이 반대라면 여기서 부호를 바꾸면 됩니다)
            m1_val = target_side - target_bend
            m2_val = target_side + target_bend

            # 5. 오프셋 적용
            m1_val += self.MOTOR_OFFSETS[i*2]
            m2_val += self.MOTOR_OFFSETS[i*2+1]

            # 6. 스무딩 및 최종 저장 (LPF)
            # 이전 모터 값 가져오기
            idx_m1 = i * 2
            idx_m2 = i * 2 + 1
            
            smooth_m1 = (self.prev_motor[idx_m1] * (1 - self.FILTER_ALPHA)) + (m1_val * self.FILTER_ALPHA)
            smooth_m2 = (self.prev_motor[idx_m2] * (1 - self.FILTER_ALPHA)) + (m2_val * self.FILTER_ALPHA)
            
            self.prev_motor[idx_m1] = smooth_m1
            self.prev_motor[idx_m2] = smooth_m2

            # 7. 최종 변환 (Degree -> Radian for MuJoCo)
            joint_commands.append(np.deg2rad(smooth_m1))
            joint_commands.append(np.deg2rad(smooth_m2))

        # MuJoCo로 전송
        out_msg = Float64MultiArray()
        out_msg.data = joint_commands
        self.pub.publish(out_msg)

def main():
    rclpy.init()
    try:
        node = RobotHandDriver()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if 'node' in locals():
            node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
