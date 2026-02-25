import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
import cv2
import mediapipe as mp
import numpy as np
import yaml
import os

# config.yaml 기본 경로
DEFAULT_CONFIG_PATH = os.path.join(
    os.path.dirname(os.path.abspath(__file__)),
    '..', '..', '..', 'config', 'config.yaml'
)

# config.yaml이 없을 때 사용하는 기본값 (world_landmarks 미터 단위)
DEFAULT_BEND_RANGES = {
    0: [-0.01, 0.08],
    1: [-0.01, 0.09],
    2: [-0.01, 0.08],
    3: [ 0.00, 0.07],
}
DEFAULT_SPREAD_RANGES = {
    0: [-0.03, 0.03],
    1: [-0.02, 0.02],
    2: [-0.03, 0.03],
    3: [-0.04, 0.04],
}

MP_TIPS      = [8, 12, 16, 4]
MP_MCPS      = [5,  9, 13, 1]
FINGER_NAMES = ["Index", "Middle", "Ring", "Thumb"]
FINGER_KEYS  = ["index", "middle", "ring", "thumb"]


def load_calib_from_config(config_path: str):
    """
    config.yaml에서 vision.fingers 캘리브레이션 값을 읽어옴.
    파일이 없거나 값이 없으면 기본값 사용.
    """
    bend_ranges   = dict(DEFAULT_BEND_RANGES)
    spread_ranges = dict(DEFAULT_SPREAD_RANGES)

    try:
        resolved = os.path.realpath(config_path)
        if not os.path.exists(resolved):
            return bend_ranges, spread_ranges

        with open(resolved, 'r') as f:
            cfg = yaml.safe_load(f) or {}

        fingers_cfg = cfg.get('vision', {}).get('fingers', {})
        for i, key in enumerate(FINGER_KEYS):
            fcfg = fingers_cfg.get(key, {})

            br = fcfg.get('bend_range', None)
            sr = fcfg.get('spread_range', None)

            # 값이 있고 길이가 정확히 2개일 때만 적용 (버그 방지)
            if br and len(br) == 2:
                bend_ranges[i] = [float(br[0]), float(br[1])]
            if sr and len(sr) == 2:
                spread_ranges[i] = [float(sr[0]), float(sr[1])]

    except Exception as e:
        print(f"⚠️  Config 읽기 실패, 기본값 사용: {e}")

    return bend_ranges, spread_ranges


class Finger:
    def __init__(self, idx, bend_range, spread_range):
        self.idx          = idx
        self.bend_range   = bend_range
        self.spread_range = spread_range
        self.prev_bend    = 0.0
        self.prev_side    = 0.0
        self.DEADBAND     = 0.002   # 2mm (world_landmarks 미터 단위)

    def update_ranges(self, bend_range, spread_range):
        """캘리브레이션 값 핫 업데이트 (재시작 없이 적용)"""
        self.bend_range   = bend_range
        self.spread_range = spread_range

    def process(self, R, landmarks_world):
        tip_lm = landmarks_world.landmark[MP_TIPS[self.idx]]
        mcp_lm = landmarks_world.landmark[MP_MCPS[self.idx]]

        tip_vec   = np.array([tip_lm.x - mcp_lm.x,
                               tip_lm.y - mcp_lm.y,
                               tip_lm.z - mcp_lm.z])
        local_vec = R @ tip_vec

        curr_bend = local_vec[2]   # z: 굽힘
        curr_side = local_vec[0]   # x: 좌우 벌림

        # 데드밴드 필터
        if abs(curr_bend - self.prev_bend) < self.DEADBAND:
            curr_bend = self.prev_bend
        if abs(curr_side - self.prev_side) < self.DEADBAND:
            curr_side = self.prev_side

        self.prev_bend = curr_bend
        self.prev_side = curr_side

        norm_bend = float(np.clip(
            np.interp(curr_bend, self.bend_range, [0.0, 1.0]), 0.0, 1.0))

        if self.idx == 3:  # 엄지
            norm_side = float(np.clip(
                np.interp(curr_side, self.spread_range, [0.0, 1.0]), 0.0, 1.0))
        else:
            norm_side = float(np.clip(
                np.interp(curr_side, self.spread_range, [-1.0, 1.0]), -1.0, 1.0))

        return norm_bend, norm_side


class VisionNode(Node):
    def __init__(self):
        super().__init__('vision_node')

        # config 경로 파라미터
        self.declare_parameter('config_path', DEFAULT_CONFIG_PATH)
        config_path = self.get_parameter(
            'config_path').get_parameter_value().string_value
        self.config_path = config_path

        # 캘리브레이션 값 로드
        bend_ranges, spread_ranges = load_calib_from_config(config_path)
        self.get_logger().info(f"✅ Config loaded: {config_path}")
        for i, key in enumerate(FINGER_KEYS):
            self.get_logger().info(
                f"   [{key}] bend={bend_ranges[i]}  spread={spread_ranges[i]}")

        self.control_pub_ = self.create_publisher(
            Float32MultiArray, '/hand_control', 10)
        self.timer = self.create_timer(0.03, self.timer_callback)

        # 캘리브레이션 리로드 타이머 (5초마다 config 재확인)
        # hand_calib.py로 저장하면 자동으로 반영됨
        self.reload_timer = self.create_timer(5.0, self.reload_config)
        self.last_config_mtime = 0.0

        # 카메라
        self.cap = cv2.VideoCapture(0)
        if not self.cap.isOpened():
            self.get_logger().error(
                "❌ Camera open failed! Check Docker '--device' settings.")
            raise SystemExit

        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
        self.cap.set(cv2.CAP_PROP_FPS, 30)

        # MediaPipe
        self.mp_hands = mp.solutions.hands.Hands(
            max_num_hands=1,
            min_detection_confidence=0.7,
            min_tracking_confidence=0.5,
            model_complexity=0
        )
        self.mp_drawing       = mp.solutions.drawing_utils
        self.mp_drawing_styles = mp.solutions.drawing_styles

        self.fingers = [
            Finger(i, bend_ranges[i], spread_ranges[i]) for i in range(4)]

        self.window_name = "Vision Control"
        self.frame_count = 0
        self.fps_timer   = self.get_clock().now()
        self.current_fps = 0.0

        # Palm Frame 특이점 발생 시 재사용할 이전 프레임 저장
        self.prev_R = np.eye(3)

        self.get_logger().info("📷 Vision Node Started [world_landmarks mode]")

    def build_palm_frame(self, landmarks_world):
        """
        손바닥 좌표계(Palm Frame) 생성.
        카메라 각도/거리 무관하게 손가락 움직임을 일관되게 측정.

        z축: 손목 → 중지 뿌리 (손가락 뻗는 방향)
        x축: 손바닥 법선 방향
        y축: 손가락 좌우 방향

        특이점 발생 시 이전 프레임(prev_R)을 재사용하여 급격한 튐 방지.
        """
        wrist     = np.array([landmarks_world.landmark[0].x,
                              landmarks_world.landmark[0].y,
                              landmarks_world.landmark[0].z])
        mid_mcp   = np.array([landmarks_world.landmark[9].x,
                              landmarks_world.landmark[9].y,
                              landmarks_world.landmark[9].z])
        pinky_mcp = np.array([landmarks_world.landmark[17].x,
                              landmarks_world.landmark[17].y,
                              landmarks_world.landmark[17].z])

        vec_z  = mid_mcp - wrist
        norm_z = np.linalg.norm(vec_z)
        if norm_z < 1e-6:
            self.get_logger().warn("⚠️ Palm Frame z축 특이점 — 이전 프레임 유지")
            return self.prev_R
        unit_z = vec_z / norm_z

        cross  = np.cross(pinky_mcp - wrist, unit_z)
        norm_x = np.linalg.norm(cross)
        if norm_x < 1e-6:
            self.get_logger().warn("⚠️ Palm Frame x축 특이점 — 이전 프레임 유지")
            return self.prev_R
        unit_x = cross / norm_x

        unit_y = np.cross(unit_z, unit_x)
        R = np.array([unit_x, unit_y, unit_z])
        self.prev_R = R
        return R

    def reload_config(self):
        """
        config.yaml 변경 감지 후 캘리브레이션 값 자동 리로드.
        hand_calib.py로 저장하면 5초 이내에 자동 반영됨.
        노드 재시작 불필요.
        """
        try:
            resolved = os.path.realpath(self.config_path)
            if not os.path.exists(resolved):
                return
            mtime = os.path.getmtime(resolved)
            if mtime <= self.last_config_mtime:
                return   # 변경 없으면 무시

            self.last_config_mtime = mtime
            bend_ranges, spread_ranges = load_calib_from_config(self.config_path)

            for i in range(4):
                self.fingers[i].update_ranges(bend_ranges[i], spread_ranges[i])

            self.get_logger().info("🔄 캘리브레이션 값 자동 리로드 완료")

        except Exception as e:
            self.get_logger().warn(f"⚠️  Config 리로드 실패: {e}")

    def timer_callback(self):
        try:
            if cv2.getWindowProperty(self.window_name,
                                     cv2.WND_PROP_VISIBLE) < 1:
                self.close_node()
                return
        except Exception:
            pass

        ret, frame = self.cap.read()
        if not ret:
            return

        frame = cv2.flip(frame, 1)
        rgb   = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        rgb.flags.writeable = False
        results = self.mp_hands.process(rgb)
        rgb.flags.writeable = True

        packet = []

        if (results.multi_hand_landmarks
                and results.multi_hand_world_landmarks):
            for hand_lm, hand_world_lm in zip(
                    results.multi_hand_landmarks,
                    results.multi_hand_world_landmarks):

                self.mp_drawing.draw_landmarks(
                    frame, hand_lm,
                    mp.solutions.hands.HAND_CONNECTIONS,
                    self.mp_drawing_styles.get_default_hand_landmarks_style(),
                    self.mp_drawing_styles.get_default_hand_connections_style())

                R = self.build_palm_frame(hand_world_lm)

                for finger in self.fingers:
                    b, s = finger.process(R, hand_world_lm)
                    packet.append(b)
                    packet.append(s)

                if len(packet) == 8:
                    msg = Float32MultiArray()
                    msg.data = packet
                    self.control_pub_.publish(msg)
                    self._draw_debug_info(frame, packet)

        self._update_fps()
        cv2.putText(frame, f"FPS: {self.current_fps:.1f}",
                    (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
        cv2.putText(frame, "world_landmarks",
                    (10, 55), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 200, 255), 1)

        cv2.imshow(self.window_name, frame)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            self.close_node()

    def _draw_debug_info(self, frame, packet):
        y = 85
        for i, name in enumerate(FINGER_NAMES):
            bend = packet[i * 2]
            side = packet[i * 2 + 1]
            cv2.putText(frame,
                        f"{name}: bend={bend:.2f}  side={side:+.2f}",
                        (10, y), cv2.FONT_HERSHEY_SIMPLEX,
                        0.5, (255, 255, 255), 1)
            y += 22

    def _update_fps(self):
        self.frame_count += 1
        now     = self.get_clock().now()
        elapsed = (now - self.fps_timer).nanoseconds / 1e9
        if elapsed >= 1.0:
            self.current_fps = self.frame_count / elapsed
            self.frame_count = 0
            self.fps_timer   = now

    def close_node(self):
        self.get_logger().info("🛑 Shutting down Vision Node...")
        self.timer.cancel()
        self.reload_timer.cancel()
        if self.cap.isOpened():
            self.cap.release()
        cv2.destroyAllWindows()
        raise SystemExit


def main():
    rclpy.init()
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
        if 'node' in locals() and hasattr(node, 'cap'):
            if node.cap.isOpened():
                node.cap.release()
        if 'node' in locals() and hasattr(node, 'destroy_node'):
            node.destroy_node()
        cv2.destroyAllWindows()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()