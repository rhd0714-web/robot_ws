import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
import tkinter as tk
import threading
import math

class RawMuJoCoSlider(Node):
    def __init__(self):
        super().__init__('raw_mujoco_slider')
        # driver_node를 거치지 않고 MuJoCo로 다이렉트로 쏩니다!
        self.pub = self.create_publisher(Float64MultiArray, '/joint_commands', 10)
        self.get_logger().info('⚠️ 바이패스 모드 가동: MuJoCo 모터 다이렉트 제어 중!')

    def publish_raw_joints(self, raw_bend, raw_side):
        # 믹싱 공식 (M1 = B + S, M2 = -B + S)
        m1_deg = raw_bend + raw_side
        m2_deg = -raw_bend + raw_side

        # Radian으로 변환
        m1_rad = math.radians(m1_deg)
        m2_rad = math.radians(m2_deg)

        msg = Float64MultiArray()
        # 검지, 중지, 약지, 엄지 (8개 모터에 동일한 값 전송하여 테스트)
        msg.data = [m1_rad, m2_rad, m1_rad, m2_rad, m1_rad, m2_rad, m1_rad, m2_rad]
        self.pub.publish(msg)

def run_gui(ros_node):
    root = tk.Tk()
    root.title("MuJoCo 다이렉트 한계 측정기")
    root.geometry("400x200")

    def on_slider_change(event):
        bend = bend_slider.get()
        side = side_slider.get()
        ros_node.publish_raw_joints(bend, side)

    # 제한 없이 넓은 범위(-180도 ~ 180도)를 직접 줍니다.
    tk.Label(root, text="🔥 Raw Bend (Degree)", font=("Arial", 12)).pack(pady=5)
    bend_slider = tk.Scale(root, from_=-180.0, to=180.0, resolution=1.0, orient=tk.HORIZONTAL, length=350, command=on_slider_change)
    bend_slider.set(0.0)
    bend_slider.pack()

    tk.Label(root, text="🔥 Raw Side (Degree)", font=("Arial", 12)).pack(pady=5)
    side_slider = tk.Scale(root, from_=-90.0, to=90.0, resolution=1.0, orient=tk.HORIZONTAL, length=350, command=on_slider_change)
    side_slider.set(0.0)
    side_slider.pack()

    root.mainloop()

def main():
    rclpy.init()
    node = RawMuJoCoSlider()
    
    ros_thread = threading.Thread(target=lambda: rclpy.spin(node), daemon=True)
    ros_thread.start()
    
    try:
        run_gui(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
