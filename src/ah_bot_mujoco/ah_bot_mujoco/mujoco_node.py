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

        # 시뮬레이션 루프 시작
        self.running = True
        self.sim_thread = threading.Thread(target=self.run_simulation)
        self.sim_thread.start()

    def command_callback(self, msg):
        # Driver가 보낸 모터 각도를 MuJoCo 로봇에 적용
        if len(msg.data) == self.model.nu: # 모터 개수가 맞는지 확인
            self.data.ctrl[:] = msg.data
        else:
            # 개수가 안 맞으면 앞에서부터 채움 (안전장치)
            limit = min(len(msg.data), self.model.nu)
            self.data.ctrl[:limit] = msg.data[:limit]

    def run_simulation(self):
        # MuJoCo 뷰어 실행
        with mujoco.viewer.launch_passive(self.model, self.data) as viewer:
            while viewer.is_running() and self.running:
                step_start = time.time()

                # 물리 엔진 계산
                mujoco.mj_step(self.model, self.data)

                # 뷰어 업데이트
                viewer.sync()

                # 현재 관절 상태 ROS로 쏘기 (PlotJuggler용)
                self.publish_joint_states()

                # 60FPS 유지
                time_until_next_step = self.model.opt.timestep - (time.time() - step_start)
                if time_until_next_step > 0:
                    time.sleep(time_until_next_step)

    def publish_joint_states(self):
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        # 관절 이름 가져오기 (없으면 그냥 번호로)
        msg.name = [f"joint_{i}" for i in range(self.model.nq)]
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
