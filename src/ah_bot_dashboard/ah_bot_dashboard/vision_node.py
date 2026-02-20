import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
import cv2
import mediapipe as mp
import numpy as np
import math
import sys

# [AmazingHand] 캘리브레이션 데이터
FINGER_BEND_RANGES = {
    0: [0.77, 1.76], 1: [0.72, 1.86], 2: [0.63, 1.83], 3: [0.53, 1.01],
}
FINGER_SPREAD_RANGES = {
    0: [-10, 25], 1: [-10, 10], 2: [-20, 10], 3: [20,  60],
}
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

        if self.idx == 3: dist = get_distance(tip, pinky_mcp)
        else: dist = get_distance(tip, wrist)
        
        if palm_size == 0: palm_size = 1.0
        curr_ratio = dist / palm_size

        if self.idx == 3:
            curr_angle = abs(get_signed_angle(wrist, landmarks.landmark[5], wrist, tip))
        else:
            curr_angle = get_signed_angle(wrist, mid_mcp, mcp, tip)

        if abs(curr_ratio - self.prev_ratio) < self.DEADBAND_RATIO: curr_ratio = self.prev_ratio
        if abs(curr_angle - self.prev_angle) < self.DEADBAND_ANGLE: curr_angle = self.prev_angle
        
        self.prev_ratio = curr_ratio
        self.prev_angle = curr_angle

        norm_bend = np.interp(curr_ratio, self.bend_range, [0.0, 1.0])
        if self.idx == 3: norm_side = np.interp(curr_angle, self.spread_range, [0.0, 1.0])
        else: norm_side = np.interp(curr_angle, self.spread_range, [-1.0, 1.0])

        return norm_bend, norm_side

class VisionNode(Node):
    def __init__(self):
        super().__init__('vision_node')
        self.control_pub_ = self.create_publisher(Float32MultiArray, '/hand_control', 10)
        
        self.timer = self.create_timer(0.03, self.timer_callback)
        
        # [보완 1] 카메라 연결 확인 로직 추가
        self.cap = cv2.VideoCapture(0)
        if not self.cap.isOpened():
            self.get_logger().error("❌ Camera open failed! Check Docker '-v' or '--device' settings.")
            # 카메라가 없으면 바로 종료 (SystemExit)
            raise SystemExit 

        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
        self.cap.set(cv2.CAP_PROP_FPS, 30) # FPS 명시적 설정
        
        self.mp_hands = mp.solutions.hands.Hands(
            max_num_hands=1, 
            min_detection_confidence=0.7,
            min_tracking_confidence=0.5,
            model_complexity=0 # [팁] 0으로 하면 속도가 빨라짐 (정확도는 약간 하락)
        )
        self.mp_drawing = mp.solutions.drawing_utils
        self.mp_drawing_styles = mp.solutions.drawing_styles
        
        self.fingers = [Finger(i) for i in range(4)]
        self.window_name = "Vision Control"
        self.get_logger().info("📷 Vision Node Started (Press 'q' or click 'X' to exit)")

    def timer_callback(self):
        try:
            # 창이 생성된 이후에만 닫힘 여부를 확인
            if cv2.getWindowProperty(self.window_name, cv2.WND_PROP_VISIBLE) >= 0:
                 if cv2.getWindowProperty(self.window_name, cv2.WND_PROP_VISIBLE) < 1:
                    self.close_node()
                    return
        except:
            pass

        ret, frame = self.cap.read()
        if not ret: return
        
        frame = cv2.flip(frame, 1)
        rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        
        rgb.flags.writeable = False
        results = self.mp_hands.process(rgb)
        rgb.flags.writeable = True

        packet = []

        if results.multi_hand_landmarks:
            for hand_landmarks in results.multi_hand_landmarks:
                self.mp_drawing.draw_landmarks(
                    frame,
                    hand_landmarks,
                    mp.solutions.hands.HAND_CONNECTIONS,
                    self.mp_drawing_styles.get_default_hand_landmarks_style(),
                    self.mp_drawing_styles.get_default_hand_connections_style()
                )

                wrist = hand_landmarks.landmark[0]
                mid_mcp = hand_landmarks.landmark[9]
                
                for finger in self.fingers:
                    current_palm = get_distance(wrist, mid_mcp)
                    if finger.idx == 3: current_palm = get_distance(wrist, hand_landmarks.landmark[5])
                    
                    b, s = finger.process(wrist, mid_mcp, hand_landmarks, current_palm)
                    packet.append(float(b))
                    packet.append(float(s))
                
                # 데이터가 8개(4손가락 * 2개)일 때만 전송
                if len(packet) == 8:
                    msg = Float32MultiArray()
                    msg.data = packet
                    self.control_pub_.publish(msg)

        cv2.imshow(self.window_name, frame)
        
        key = cv2.waitKey(1)
        if key & 0xFF == ord('q'):
            self.close_node()

    def close_node(self):
        self.get_logger().info("🛑 Shutting down Vision Node...")
        self.timer.cancel()
        if self.cap.isOpened():
            self.cap.release()
        cv2.destroyAllWindows()
        raise SystemExit 

def main():
    rclpy.init()
    # 노드 생성 시 에러(카메라 없음 등)가 발생하면 바로 종료 처리
    try:
        node = VisionNode()
        rclpy.spin(node)
    except SystemExit:
        pass
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f"Error: {e}")
    finally:
        # 안전한 종료 보장
        if 'node' in locals():
            if node.cap.isOpened():
                node.cap.release()
            node.destroy_node()
        cv2.destroyAllWindows()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()