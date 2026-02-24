import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray, Float64MultiArray
import numpy as np
import yaml
import os
import struct

# ============================================================
# Feetech SCS0009 Protocol V1 — pyserial 직접 구현
#
# 오픈소스(AHControl)의 rustypot이 하던 일을 Python으로 대체.
# 외부 SDK 없이 pyserial만으로 동작.
#
# 패킷 구조:
#   [0xFF][0xFF][ID][LENGTH][INSTRUCTION][PARAMS...][CHECKSUM]
#   CHECKSUM = ~(ID + LENGTH + INSTRUCTION + PARAMS) & 0xFF
# ============================================================
try:
    import serial
    SERIAL_AVAILABLE = True
except ImportError:
    SERIAL_AVAILABLE = False

# SCS0009 레지스터 주소
ADDR_TORQUE_ENABLE   = 40
ADDR_GOAL_POSITION   = 42
ADDR_PRESENT_POSITION = 56

# SCS0009 스펙: 0~1023 (10bit), 0~300도, 중앙 512
SCS_CENTER    = 512
SCS_MAX_POS   = 1023
SCS_DEG_RANGE = 300.0

# Feetech 명령어
INST_WRITE    = 0x03
INST_SYNC_WRITE = 0x83

DEFAULT_CONFIG_PATH = os.path.join(
    os.path.dirname(os.path.abspath(__file__)),
    '..', '..', '..', 'config', 'config.yaml'
)
FINGER_NAMES = ["index", "middle", "ring", "thumb"]


def load_config(path: str) -> dict:
    resolved = os.path.realpath(path)
    if not os.path.exists(resolved):
        raise FileNotFoundError(f"Config not found: {resolved}")
    with open(resolved, 'r') as f:
        return yaml.safe_load(f)


def deg_to_scs(deg: float) -> int:
    """각도(Degree) → SCS0009 위치값(0~1023). 0도=512(중앙)."""
    pos = int(SCS_CENTER + (deg / SCS_DEG_RANGE) * SCS_MAX_POS)
    return int(np.clip(pos, 0, SCS_MAX_POS))


def calc_checksum(data: list) -> int:
    """Feetech V1 체크섬: ~(sum of bytes) & 0xFF"""
    return (~sum(data)) & 0xFF


def make_write_packet(motor_id: int, address: int, value: int, size: int = 2) -> bytes:
    """
    단일 모터 쓰기 패킷 생성.
    size=2: 2바이트(위치값), size=1: 1바이트(토크 등)
    """
    if size == 2:
        params = [address, value & 0xFF, (value >> 8) & 0xFF]
    else:
        params = [address, value & 0xFF]

    length   = len(params) + 2  # INSTRUCTION + PARAMS + CHECKSUM
    body     = [motor_id, length, INST_WRITE] + params
    checksum = calc_checksum(body)
    return bytes([0xFF, 0xFF] + body + [checksum])


def make_sync_write_packet(address: int, motor_ids: list,
                            positions: list) -> bytes:
    """
    여러 모터에 동시에 위치 명령 전송 (sync write).
    오픈소스 rustypot의 sync_write_goal_position과 동일한 역할.

    패킷 구조:
      FF FF FE [LEN] 83 [ADDR] 02 [ID1][POS_L][POS_H] [ID2]... [CHECKSUM]
    """
    data_per_motor = 2   # 위치값 2바이트
    params = [address, data_per_motor]
    for mid, pos in zip(motor_ids, positions):
        pos = int(np.clip(pos, 0, SCS_MAX_POS))
        params += [mid, pos & 0xFF, (pos >> 8) & 0xFF]

    length   = len(params) + 2
    body     = [0xFE, length, INST_SYNC_WRITE] + params
    checksum = calc_checksum(body)
    return bytes([0xFF, 0xFF] + body + [checksum])


class HardwareInterface:
    """
    SCS0009 시리얼 통신 클래스.
    pyserial만으로 Feetech Protocol V1 직접 구현.
    """

    def __init__(self, cfg: dict, logger):
        self.logger    = logger
        self.available = False
        self.port      = None

        if not SERIAL_AVAILABLE:
            logger.warn("⚠️  pyserial 미설치 → pip3 install pyserial")
            return

        hw_cfg         = cfg.get('hardware', {})
        serial_port    = hw_cfg.get('serial_port', '/dev/ttyACM0')
        baudrate       = hw_cfg.get('baudrate', 1_000_000)
        timeout        = hw_cfg.get('timeout_ms', 10) / 1000.0
        on_fail        = hw_cfg.get('on_connect_fail', 'warn')
        self.angle_min = hw_cfg.get('angle_limit_min', -150.0)
        self.angle_max = hw_cfg.get('angle_limit_max',  150.0)

        # 모터 설정 파싱
        self.motor_ids     = []
        self.motor_offsets = []   # Degree
        self.motor_inverts = []

        for m in hw_cfg.get('motors', []):
            self.motor_ids.extend([m['motor1_id'], m['motor2_id']])
            self.motor_offsets.extend([
                np.degrees(m.get('motor1_offset_rad', 0.0)),
                np.degrees(m.get('motor2_offset_rad', 0.0))
            ])
            self.motor_inverts.extend([
                m.get('motor1_invert', False),
                m.get('motor2_invert', False)
            ])

        try:
            self.port = serial.Serial(
                port     = serial_port,
                baudrate = baudrate,
                timeout  = timeout
            )

            # 전체 모터 토크 활성화
            for mid in self.motor_ids:
                pkt = make_write_packet(mid, ADDR_TORQUE_ENABLE, 1, size=1)
                self.port.write(pkt)
                self.port.flush()

            self.available = True
            logger.info(f"✅ 하드웨어 연결 성공: {serial_port} @ {baudrate}bps")
            logger.info(f"   모터 ID: {self.motor_ids}")

        except Exception as e:
            msg = f"❌ 하드웨어 연결 실패: {e}"
            if on_fail == 'error':
                raise RuntimeError(msg)
            else:
                logger.warn(msg)
                logger.warn("   시뮬레이션 모드로 계속 실행합니다.")

    def send_joint_commands(self, joint_commands_deg: list):
        """
        모든 모터에 목표 각도 동시 전송 (sync write).
        offset + invert 적용 후 Degree → SCS 위치값 변환.
        """
        if not self.available or self.port is None:
            return

        positions = []
        for idx, mid in enumerate(self.motor_ids):
            if idx >= len(joint_commands_deg):
                break

            angle = float(joint_commands_deg[idx])

            # 영점 오프셋 적용
            angle += self.motor_offsets[idx]

            # 방향 반전
            if self.motor_inverts[idx]:
                angle = -angle

            # 안전 클리핑
            angle = float(np.clip(angle, self.angle_min, self.angle_max))

            positions.append(deg_to_scs(angle))

        try:
            pkt = make_sync_write_packet(
                ADDR_GOAL_POSITION, self.motor_ids, positions)
            self.port.write(pkt)
            self.port.flush()
        except Exception as e:
            self.logger.warn(f"⚠️  시리얼 전송 오류: {e}")

    def disable_torque(self):
        """종료 시 전체 모터 토크 해제"""
        if not self.available or self.port is None:
            return
        for mid in self.motor_ids:
            try:
                pkt = make_write_packet(mid, ADDR_TORQUE_ENABLE, 0, size=1)
                self.port.write(pkt)
                self.port.flush()
            except Exception:
                pass
        self.port.close()
        self.logger.info("🔌 하드웨어 연결 해제 완료")


class RobotHandDriver(Node):
    def __init__(self):
        super().__init__('ah_bot_driver')

        self.declare_parameter('config_path', DEFAULT_CONFIG_PATH)
        config_path = self.get_parameter(
            'config_path').get_parameter_value().string_value

        try:
            cfg = load_config(config_path)
            self.get_logger().info(f"✅ Config loaded: {config_path}")
        except FileNotFoundError as e:
            self.get_logger().warn(f"⚠️  {e} → 기본값으로 실행")
            cfg = {}

        self.mode = cfg.get('mode', 'simulation')
        self.get_logger().info(f"🚀 실행 모드: {self.mode}")

        driver_cfg            = cfg.get('driver', {})
        self.FILTER_ALPHA     = driver_cfg.get('filter_alpha', 0.15)
        self.RANGE_BEND       = driver_cfg.get('bend_range_deg', [100.0, -30.0])
        self.RANGE_SIDE       = driver_cfg.get('side_range_deg', [-20.0, 20.0])
        self.RANGE_THUMB_SIDE = driver_cfg.get('thumb_side_range_deg', [0.0, 45.0])

        fingers_cfg = driver_cfg.get('fingers', {})
        self.motor1_offsets = []
        self.motor2_offsets = []
        self.invert_sides   = []
        for name in FINGER_NAMES:
            fcfg = fingers_cfg.get(name, {})
            self.motor1_offsets.append(fcfg.get('motor1_offset', 0.0))
            self.motor2_offsets.append(fcfg.get('motor2_offset', 0.0))
            self.invert_sides.append(fcfg.get('invert_side', False))

        self.prev_motor = np.zeros(8)

        # 하드웨어 인터페이스
        self.hw = None
        if self.mode in ('hardware', 'both'):
            self.hw = HardwareInterface(cfg, self.get_logger())

        # ROS2 토픽
        self.sub = self.create_subscription(
            Float32MultiArray, '/hand_control', self.callback, 10)

        self.sim_pub = None
        if self.mode in ('simulation', 'both'):
            self.sim_pub = self.create_publisher(
                Float64MultiArray, '/joint_commands', 10)

        self.get_logger().info(
            f"🧠 Driver 준비완료 | "
            f"sim={'ON' if self.sim_pub else 'OFF'} | "
            f"hw={'ON' if (self.hw and self.hw.available) else 'OFF'}"
        )

    def callback(self, msg):
        inputs = msg.data
        if len(inputs) != 8:
            self.get_logger().warn(f"⚠️  예상 8개, 수신 {len(inputs)}개.")
            return

        joint_commands_rad = []
        joint_commands_deg = []

        for i in range(4):
            in_bend = inputs[i * 2]
            in_side = inputs[i * 2 + 1]

            target_bend = np.interp(in_bend, [0.0, 1.0], self.RANGE_BEND)
            if i == 3:
                target_side = np.interp(
                    in_side, [0.0, 1.0], self.RANGE_THUMB_SIDE)
            else:
                target_side = np.interp(
                    in_side, [-1.0, 1.0], self.RANGE_SIDE)

            if self.invert_sides[i]:
                target_side = -target_side

            m1_val = target_side + target_bend + self.motor1_offsets[i]
            m2_val = target_side - target_bend + self.motor2_offsets[i]

            idx_m1, idx_m2 = i * 2, i * 2 + 1
            smooth_m1 = (self.prev_motor[idx_m1] * (1 - self.FILTER_ALPHA)
                         + m1_val * self.FILTER_ALPHA)
            smooth_m2 = (self.prev_motor[idx_m2] * (1 - self.FILTER_ALPHA)
                         + m2_val * self.FILTER_ALPHA)
            self.prev_motor[idx_m1] = smooth_m1
            self.prev_motor[idx_m2] = smooth_m2

            joint_commands_deg.extend([smooth_m1, smooth_m2])
            joint_commands_rad.extend([
                np.deg2rad(smooth_m1),
                np.deg2rad(smooth_m2)
            ])

        if self.sim_pub:
            out_msg = Float64MultiArray()
            out_msg.data = joint_commands_rad
            self.sim_pub.publish(out_msg)

        if self.hw:
            self.hw.send_joint_commands(joint_commands_deg)

    def destroy_node(self):
        if self.hw:
            self.hw.disable_torque()
        super().destroy_node()


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