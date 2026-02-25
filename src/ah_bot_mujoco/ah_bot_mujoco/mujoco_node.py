import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import JointState
import mujoco
import mujoco.viewer
import numpy as np
import time
import threading
import yaml
import os

DEFAULT_CONFIG_PATH = os.path.join(
    os.path.dirname(os.path.abspath(__file__)),
    '..', '..', '..', 'config', 'config.yaml'
)


class MujocoNode(Node):
    def __init__(self):
        super().__init__('mujoco_node')

        # config_path 파라미터 (driver_node.py와 동일한 패턴)
        self.declare_parameter('config_path', DEFAULT_CONFIG_PATH)
        config_path = self.get_parameter('config_path').get_parameter_value().string_value

        # config.yaml에서 xml_path 읽기
        xml_path = ''
        try:
            resolved = os.path.realpath(config_path)
            if os.path.exists(resolved):
                with open(resolved, 'r') as f:
                    cfg = yaml.safe_load(f) or {}
                xml_path = cfg.get('mujoco', {}).get('xml_path', '')
                self.get_logger().info(f"✅ Config loaded: {resolved}")
            else:
                self.get_logger().warn(f"⚠️  Config not found: {resolved}")
        except Exception as e:
            self.get_logger().warn(f"⚠️  Config 읽기 실패: {e}")

        if not xml_path:
            self.get_logger().error("❌ mujoco.xml_path가 config.yaml에 없습니다.")
            raise SystemExit
        if not os.path.exists(xml_path):
            self.get_logger().error(f"❌ XML 파일 없음: {xml_path}")
            raise SystemExit

        # 1. Driver에서 오는 명령(모터 8개 각도) 받기
        self.subscription = self.create_subscription(
            Float64MultiArray,
            '/joint_commands',
            self.command_callback,
            10)

        # 2. 현재 상태(PlotJuggler용) 보내기
        self.publisher_ = self.create_publisher(JointState, '/joint_states', 10)

        # MuJoCo 모델 로드
        try:
            self.model = mujoco.MjModel.from_xml_path(xml_path)
            self.data = mujoco.MjData(self.model)
            self.get_logger().info(f"✅ MuJoCo Model Loaded: {xml_path}")
            self.get_logger().info(f"   actuators(nu): {self.model.nu}  joints(njnt): {self.model.njnt}")
        except Exception as e:
            self.get_logger().error(f"❌ MuJoCo 모델 로드 실패: {e}")
            raise e

        # XML에서 실제 관절 이름 미리 읽기 (publish_joint_states 최적화)
        self.joint_names = []
        for i in range(self.model.njnt):
            name = mujoco.mj_id2name(self.model, mujoco.mjtObj.mjOBJ_JOINT, i)
            self.joint_names.append(name if name is not None else f"joint_{i}")

        self.get_logger().info(f"   관절 이름: {self.joint_names}")

        # 시뮬레이션 루프 시작
        self.running = True
        self.sim_thread = threading.Thread(target=self.run_simulation)
        self.sim_thread.start()

    def command_callback(self, msg):
        # Driver가 보낸 모터 각도(라디안)를 MuJoCo actuator에 적용
        if len(msg.data) == self.model.nu:
            self.data.ctrl[:] = msg.data
        else:
            limit = min(len(msg.data), self.model.nu)
            self.data.ctrl[:limit] = msg.data[:limit]

    def run_simulation(self):
        with mujoco.viewer.launch_passive(self.model, self.data) as viewer:
            while viewer.is_running() and self.running:
                step_start = time.time()

                mujoco.mj_step(self.model, self.data)
                viewer.sync()
                self.publish_joint_states()

                time_until_next_step = self.model.opt.timestep - (time.time() - step_start)
                if time_until_next_step > 0:
                    time.sleep(time_until_next_step)

    def publish_joint_states(self):
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = self.joint_names
        msg.position = self.data.qpos.tolist()
        self.publisher_.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    try:
        node = MujocoNode()
    except SystemExit:
        if rclpy.ok():
            rclpy.shutdown()
        return
    except Exception as e:
        print(f"MujocoNode 초기화 실패: {e}")
        if rclpy.ok():
            rclpy.shutdown()
        return

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.running = False
        node.sim_thread.join()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
