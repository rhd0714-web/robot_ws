import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray, Float64MultiArray
import numpy as np

class RobotHandDriver(Node):
    def __init__(self):
        super().__init__('ah_bot_driver')
        
        self.sub = self.create_subscription(Float32MultiArray, '/hand_control', self.callback, 10)
        self.pub = self.create_publisher(Float64MultiArray, '/joint_commands', 10)

        # -------------------------------------------------------------------
        # [최적화 1] 시뮬레이션 전용 설정 (Simulation Mode)
        # -------------------------------------------------------------------
        self.SIM_MODE = True 

        # [최적화 2] 매핑 범위 수정 (Vision 데이터: 0=주먹, 1=펴짐)
        # AmazingHand 시뮬레이션 상:
        # - 약 90도 ~ 100도: 주먹 쥔 상태 (Bend)
        # - 약 0도: 쫙 편 상태 (Flat)
        # 따라서 입력 0(주먹) -> 100도 / 입력 1(펴짐) -> 0도로 매핑합니다.
        self.RANGE_BEND = [100.0, 0.0]   
        
        # 좌우 벌림 범위 (Degree)
        self.RANGE_SIDE = [-20.0, 20.0] 

        # [최적화 3] 오프셋 제거
        # MuJoCo는 조립 오차가 없는 완벽한 환경이므로 실물 오프셋을 0으로 둡니다.
        self.MOTOR_OFFSETS = [0.0] * 8

        # 스무딩 필터 (0.0 ~ 1.0) - 떨림 방지
        self.FILTER_ALPHA = 0.15
        
        # 초기값 설정 (로봇이 켜질 때 확 튀는 것 방지)
        # 시작 시 '펴짐(1.0)' 상태인 0도로 초기화
        self.prev_motor = np.zeros(8)

        self.get_logger().info('🧠 Driver Node Optimized for MuJoCo')

    def callback(self, msg):
        inputs = msg.data # [B, S, B, S, ...]
        if len(inputs) != 8: return

        joint_commands = [] 

        for i in range(4):
            # 1. Vision 데이터 수신 (0=주먹, 1=펴짐)
            in_bend = inputs[i*2]
            in_side = inputs[i*2+1]

            # 2. 각도 변환 (Mapping)
            # 0(주먹) -> 100도, 1(펴짐) -> 0도
            target_bend = np.interp(in_bend, [0.0, 1.0], self.RANGE_BEND)
            
            # Side 매핑 (엄지와 나머지 구분)
            if i == 3: # 엄지
                target_side = np.interp(in_side, [0.0, 1.0], [0, 45])
            else:      # 검지, 중지, 약지
                target_side = np.interp(in_side, [-1.0, 1.0], self.RANGE_SIDE)

            # 3. 방향 보정 (하드웨어/시뮬레이션 축 일치)
            # 검지(0), 중지(1), 약지(2)는 좌우 대칭 구조일 수 있음.
            # 움직임을 보고 반대면 부호를 바꾸세요. (현재: 기본)
            #if i == 0: target_side = target_side 
            #if i == 2: target_side = target_side 

            # 4. [핵심] 차동 구동 믹싱 (Differential Drive Mixing)
            # AmazingHand 공식: 
            # Motor1 = Side - Bend
            # Motor2 = Side + Bend
            # (또는 반대일 수 있음. 만약 굽혔는데 벌어지면 부호를 반대로!)
            
            # 시뮬레이션 테스트 기반 추천 수식:
            # 굽힘(Bend)이 주된 동작이므로 Bend가 양쪽 모터에 서로 반대로 작용해야 함.
            m1_val = target_side + target_bend
            m2_val = target_side - target_bend

            # 5. 스무딩 (LPF)
            idx_m1, idx_m2 = i*2, i*2+1
            
            smooth_m1 = (self.prev_motor[idx_m1] * (1 - self.FILTER_ALPHA)) + (m1_val * self.FILTER_ALPHA)
            smooth_m2 = (self.prev_motor[idx_m2] * (1 - self.FILTER_ALPHA)) + (m2_val * self.FILTER_ALPHA)
            
            self.prev_motor[idx_m1] = smooth_m1
            self.prev_motor[idx_m2] = smooth_m2

            # 6. 전송 (Degree -> Radian)
            joint_commands.append(np.deg2rad(smooth_m1))
            joint_commands.append(np.deg2rad(smooth_m2))

        # MuJoCo로 전송
        out_msg = Float64MultiArray()
        out_msg.data = joint_commands
        self.pub.publish(out_msg)

def main():
    rclpy.init()
    node = RobotHandDriver()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()