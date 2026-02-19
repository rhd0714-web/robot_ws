import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
import cv2
import mediapipe as mp
import numpy as np
import math
import sys  # 종료를 위해 추가

# [AmazingHand] 캘리브레이션 데이터
FINGER_BEND_RANGES = {
    0: [0.85, 1.80], # 검지
    1: [0.90, 1.90], # 중지
    2: [0.80, 1.80], # 약지
    3: [0.55, 1.35], # 엄지
}

FINGER_SPREAD_RANGES = {
    0: [-10, 25], 
    1: [-10, 10], 
    2: [-20, 10], 
    3: [20,  60], 
}

# 랜드마크 인덱스
MP_TIPS = [8, 12, 16, 4]
MP_MCPS = [5, 9, 13, 1]

def get_distance(p1, p2):
    return math.sqrt((p1.x - p2.x)**2 + (p1.y - p2.y)**2)

def get_signed_angle(wrist, mid_mcp, finger_mcp, finger_tip):
    v1_x, v1_y = mid_mcp.x - wrist.x, mid_mcp.y - wrist.y
    v2_x, v2_y = finger_tip.x - finger_mcp.x, finger_tip.y - finger_mcp.y
    ang1 = math.atan2(v1_y, v1_x)
    ang2 = math.atan2(v2_y, v2_x)
    diff = math.degrees(ang2 - ang1)
    if diff > 180: diff -= 360
    elif diff < -180: diff += 360
    return diff

class Finger:
    def __init__(self, idx):
        self.idx = idx
        self.bend_range = FINGER_BEND_RANGES[idx]
        self.spread_range = FINGER_SPREAD_RANGES[idx]
        self.prev_ratio = 0.0
        self.prev_angle = 0.0
        self.DEADBAND_RATIO = 0.02
        self.DEADBAND_ANGLE = 2.0

    def process(self, wrist, mid_mcp, landmarks, palm_size):
        tip = landmarks.landmark[MP_TIPS[self.idx]]
        mcp = landmarks.landmark[MP_MCPS[self.idx]]
        pinky_mcp = landmarks.landmark[17]

        # 1. 거리(Ratio) 계산
        if self.idx == 3: # 엄지
             dist = get_distance(tip, pinky_mcp)
        else:
             dist = get_distance(tip, wrist)
        
        # 0으로 나누기 방지
        if palm_size == 0: palm_size = 1.0
        curr_ratio = dist / palm_size

        # 2. 각도(Angle) 계산
        if self.idx == 3: # 엄지
            curr_angle = abs(get_signed_angle(wrist, landmarks.landmark[5], wrist, tip))
        else:
            curr_angle = get_signed_angle(wrist, mid_mcp, mcp, tip)

        # 3. 불감대 (Deadband)
        if abs(curr_ratio - self.prev_ratio) < self.DEADBAND_RATIO:
            curr_ratio = self.prev_ratio
        if abs(curr_angle - self.prev_angle) < self.DEADBAND_ANGLE:
            curr_angle = self.prev_angle
        
        self.prev_ratio = curr_ratio
        self.prev_angle = curr_angle

        # 4. 정규화 (0.0 ~ 1.0)
        norm_bend = np.interp(curr_ratio, self.bend_range, [0.0, 1.0])
        
        if self.idx == 3:
            norm_side = np.interp(curr_angle, self.spread_range, [0.0, 1.0])
        else:
            norm_side = np.interp(curr_angle, self.spread_range, [-1.0, 1.0])

        return norm_bend, norm_side

class VisionNode(Node):
    def __init__(self):
        super().__init__('vision_node')
        # 토픽 이름을 '/hand_control'로 유지 (MuJoCo 노드가 이걸 받는지 확인 필요!)
        self.control_pub_ = self.create_publisher(Float32MultiArray, '/hand_control', 10)
        
        # [문제 1 해결]: 타이머 주기를 0.03 (약 30fps)으로 조금 더 빠르게 설정
        self.timer = self.create_timer(0.03, self.timer_callback)
        
        # 카메라 초기화
        # [팁] 도커에서 V4L2 백엔드가 가끔 느릴 수 있습니다. 느리면 CAP_V4L2 제거해보세요.
        self.cap = cv2.VideoCapture(0, cv2.CAP_V4L2) 
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
        self.cap.set(cv2.CAP_PROP_FPS, 30)
        
        # MediaPipe 초기화
        self.mp_hands = mp.solutions.hands.Hands(
            max_num_hands=1, 
            min_detection_confidence=0.7,
            min_tracking_confidence=0.5
        )
        # [문제 4 해결]: 그리기 도구 추가
        self.mp_drawing = mp.solutions.drawing_utils
        self.mp_drawing_styles = mp.solutions.drawing_styles
        
        self.fingers = [Finger(i) for i in range(4)]
        self.get_logger().info("📷 Vision Node Started (Press 'q' to exit)")

    def timer_callback(self):
        ret, frame = self.cap.read()
        if not ret: return
        
        frame = cv2.flip(frame, 1)
        rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        
        # 성능 최적화를 위해 쓰기 금지 설정
        rgb.flags.writeable = False
        results = self.mp_hands.process(rgb)
        rgb.flags.writeable = True

        packet = [] # 데이터 패킷

        if results.multi_hand_landmarks:
            for hand_landmarks in results.multi_hand_landmarks:
                # [문제 4 해결]: 뼈대 그리기 (여기에 있어야 함)
                self.mp_drawing.draw_landmarks(
                    frame,
                    hand_landmarks,
                    mp.solutions.hands.HAND_CONNECTIONS,
                    self.mp_drawing_styles.get_default_hand_landmarks_style(),
                    self.mp_drawing_styles.get_default_hand_connections_style()
                )

                wrist = hand_landmarks.landmark[0]
                mid_mcp = hand_landmarks.landmark[9]
                
                # 손가락별 데이터 계산
                for finger in self.fingers:
                    current_palm = get_distance(wrist, mid_mcp)
                    if finger.idx == 3: # 엄지는 기준점이 다를 수 있음
                         current_palm = get_distance(wrist, hand_landmarks.landmark[5])
                    
                    b, s = finger.process(wrist, mid_mcp, hand_landmarks, current_palm)
                    packet.append(float(b))
                    packet.append(float(s))

                # 데이터 전송 (손이 감지되었을 때만)
                msg = Float32MultiArray()
                msg.data = packet
                self.control_pub_.publish(msg)
                # [디버깅용] 데이터가 잘 가는지 로그 찍어보기 (너무 빠르면 주석 처리)
                # self.get_logger().info(f'Pub: {packet[:2]}...') 

        # [문제 2, 3 해결 핵심]: 화면 출력 코드를 if문 '바깥'으로 뺐습니다!
        # 이제 손이 없어도 화면이 갱신되고, 키 입력을 받습니다.
        cv2.imshow("Vision Control", frame)
        
        # 'q' 누르면 종료
        key = cv2.waitKey(1)
        if key & 0xFF == ord('q'):
            self.get_logger().info("Shutting down Vision Node...")
            rclpy.shutdown() # ROS 종료 신호 보냄
            sys.exit(0)      # 프로그램 강제 종료

def main():
    rclpy.init()
    try:
        node = VisionNode()
        rclpy.spin(node)
    except SystemExit:
        pass # 정상 종료
    except KeyboardInterrupt:
        pass
    finally:
        # 자원 해제
        if 'node' in locals():
            node.cap.release()
            node.destroy_node()
        cv2.destroyAllWindows()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
