import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import JointState
import mujoco
import mujoco.viewer
import numpy as np
import time
import threading

# XML 경로 (사용자님 환경에 맞춤)
XML_PATH = "/home/user/robot_ws/AmazingHand-main/Demo/AHSimulation/AHSimulation/AH_Left/mjcf/scene.xml"

class MujocoNode(Node):
    def __init__(self):
        super().__init__('mujoco_node')

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
            self.model = mujoco.MjModel.from_xml_path(XML_PATH)
            self.data = mujoco.MjData(self.model)
            self.get_logger().info(f"MuJoCo Model Loaded: {XML_PATH}")
        except Exception as e:
            self.get_logger().error(f"Failed to load XML: {e}")
            raise e

        # [수정] XML에서 실제 관절 이름을 미리 읽어서 저장
        # 이유: publish_joint_states()가 매우 빠른 주기로 호출되는데,
        #       매번 model.joint(i).name 을 호출하면 불필요한 연산이 반복됨.
        #       한 번만 읽어서 리스트로 저장해두면 효율적임.
        #
        # MuJoCo API 설명:
        #   model.nq  = 관절 자유도(DOF) 총 개수  예) 8
        #   model.njnt = 관절(joint) 개수          예) 8
        #   mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_JOINT, i)
        #     → i번째 관절의 이름을 XML에서 읽어옴
        #     예) "index_joint", "middle_joint" 등
        #
        # 만약 이름이 None으로 반환되면 (이름이 없는 관절)
        # 기존처럼 "joint_0" 형태로 대체함
        self.joint_names = []
        for i in range(self.model.njnt):
            name = mujoco.mj_id2name(self.model, mujoco.mjtObj.mjOBJ_JOINT, i)
            if name is not None:
                self.joint_names.append(name)
            else:
                self.joint_names.append(f"joint_{i}")

        self.get_logger().info(f"관절 이름 목록: {self.joint_names}")

        # 시뮬레이션 루프 시작
        self.running = True
        self.sim_thread = threading.Thread(target=self.run_simulation)
        self.sim_thread.start()

    def command_callback(self, msg):
        # Driver가 보낸 모터 각도를 MuJoCo 로봇에 적용
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

        # [수정] __init__에서 미리 읽어둔 실제 관절 이름 사용
        # 이유: PlotJuggler의 test_layout.xml은 "/joint_states/index_joint/position" 처럼
        #       XML에 정의된 실제 이름을 기준으로 그래프를 그림.
        #       기존 코드처럼 "joint_0", "joint_1" 을 보내면
        #       PlotJuggler가 해당 토픽을 찾지 못해 그래프가 빈 상태로 보임.
        msg.name = self.joint_names

        # qpos: 각 관절의 현재 위치(각도) 배열
        msg.position = self.data.qpos.tolist()
        self.publisher_.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = MujocoNode()
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
