#!/usr/bin/env python3
import sys
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray

from PyQt6.QtWidgets import (QApplication, QWidget, QVBoxLayout, 
                             QSlider, QLabel, QHBoxLayout, QGroupBox)
from PyQt6.QtCore import Qt, QTimer

class RobotControllerGUI(QWidget):
    def __init__(self):
        super().__init__()
        
        # 1. ROS 노드 초기화
        rclpy.init(args=None)
        self.node = Node('gui_controller_node')
        
        # 명령을 보낼 Publisher (토픽: /joint_commands)
        self.publisher = self.node.create_publisher(Float64MultiArray, '/joint_commands', 10)
        
        # 2. UI 설정
        self.setWindowTitle("🎮 Manual Controller")
        self.setGeometry(600, 100, 400, 300)
        self.init_ui()
        
        # 현재 모터 값 저장 (4개 모터 예시)
        self.motor_values = [0.0, 0.0, 0.0, 0.0]

    def init_ui(self):
        layout = QVBoxLayout()
        
        # 스타일
        self.setStyleSheet("""
            QWidget { background-color: #333; color: white; }
            QSlider::groove:horizontal { height: 8px; background: #555; border-radius: 4px; }
            QSlider::handle:horizontal { background: #00bcd4; width: 20px; margin: -6px 0; border-radius: 10px; }
            QGroupBox { border: 1px solid #555; margin-top: 10px; font-weight: bold; }
            QGroupBox::title { subcontrol-origin: margin; left: 10px; padding: 0 5px; }
        """)

        # 슬라이더 4개 생성 (Index, Middle, Ring, Thumb)
        joint_names = ["Index Finger", "Middle Finger", "Ring Finger", "Thumb"]
        self.sliders = []
        
        for i, name in enumerate(joint_names):
            group = QGroupBox(name)
            vbox = QVBoxLayout()
            
            # 라벨 (현재 값 표시)
            lbl = QLabel("0.0")
            lbl.setAlignment(Qt.AlignmentFlag.AlignCenter)
            
            # 슬라이더
            slider = QSlider(Qt.Orientation.Horizontal)
            slider.setRange(-150, 150) # -1.5 ~ 1.5 라디안 정도
            slider.setValue(0)
            
            # 이벤트 연결 (람다 함수로 인덱스 전달)
            slider.valueChanged.connect(lambda val, idx=i, label=lbl: self.on_slider_change(idx, val, label))
            
            self.sliders.append(slider)
            vbox.addWidget(lbl)
            vbox.addWidget(slider)
            group.setLayout(vbox)
            layout.addWidget(group)

        self.setLayout(layout)

    def on_slider_change(self, index, value, label):
        # 1. 값 변환 (슬라이더 정수 -> 실수 라디안)
        rad_val = value / 100.0
        label.setText(f"{rad_val:.2f}")
        
        # 2. 데이터 업데이트
        self.motor_values[index] = rad_val
        
        # 3. ROS 메시지 전송
        msg = Float64MultiArray()
        msg.data = self.motor_values
        self.publisher.publish(msg)
        # self.node.get_logger().info(f"Sent: {self.motor_values}")

    def closeEvent(self, event):
        self.node.destroy_node()
        rclpy.shutdown()
        event.accept()

def main():
    app = QApplication(sys.argv)
    gui = RobotControllerGUI()
    gui.show()
    sys.exit(app.exec())

if __name__ == '__main__':
    main()
