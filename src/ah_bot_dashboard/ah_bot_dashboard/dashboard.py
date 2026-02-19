#!/usr/bin/env python3
import sys
import subprocess
import os
import signal

# PyQt6 라이브러리
from PyQt6.QtWidgets import (QApplication, QMainWindow, QWidget, QVBoxLayout, 
                             QHBoxLayout, QPushButton, QLabel, QFrame, QMessageBox)
from PyQt6.QtCore import QTimer, Qt
from PyQt6.QtGui import QFont, QColor

# ROS 2 라이브러리
import rclpy
from rclpy.node import Node

class RobotDashboard(QMainWindow):
    def __init__(self):
        super().__init__()
        
        # ROS 2 노드 초기화
        rclpy.init(args=None)
        self.node = Node('ah_bot_dashboard')
        
        # 프로세스 관리 (실행된 프로그램들을 저장)
        self.processes = {} 

        self.init_ui()
        
        # ROS 스핀을 위한 타이머 (UI가 멈추지 않게 함)
        self.timer = QTimer()
        self.timer.timeout.connect(self.spin_ros)
        self.timer.start(100) # 0.1초마다 ROS 상태 확인

    def init_ui(self):
        self.setWindowTitle("AH Bot Commander 🤖")
        self.setGeometry(100, 100, 400, 600) # 버튼이 늘어나서 창 높이를 조금 키웠습니다
        self.setStyleSheet("background-color: #2b2b2b; color: white;")

        central_widget = QWidget()
        self.setCentralWidget(central_widget)
        layout = QVBoxLayout(central_widget)
        layout.setSpacing(20)
        layout.setContentsMargins(30, 30, 30, 30)

        # 1. 헤더 (타이틀 & 상태)
        title = QLabel("ROBOT DASHBOARD")
        title.setAlignment(Qt.AlignmentFlag.AlignCenter)
        title.setFont(QFont("Arial", 20, QFont.Weight.Bold))
        title.setStyleSheet("color: #00bcd4; margin-bottom: 10px;")
        layout.addWidget(title)
        
        self.status_label = QLabel("System Status: 🟡 Ready")
        self.status_label.setAlignment(Qt.AlignmentFlag.AlignCenter)
        self.status_label.setStyleSheet("background-color: #444; padding: 5px; border-radius: 5px;")
        layout.addWidget(self.status_label)

        # 구분선
        line = QFrame()
        line.setFrameShape(QFrame.Shape.HLine)
        line.setStyleSheet("color: #555;")
        layout.addWidget(line)

        # 2. 컨트롤 버튼들
        
        # (1) 비전 모드 (수정됨: 활성화 및 기능 연결)
        self.btn_vision = self.create_button("👁️ Start Vision Node", "#9C27B0", self.toggle_vision)
        # self.btn_vision.setEnabled(False) # 비활성화 제거함
        layout.addWidget(self.btn_vision)

        # (2) 드라이버 실행 버튼
        self.btn_driver = self.create_button("🧠 Start Driver Node", "#4CAF50", self.toggle_driver)
        layout.addWidget(self.btn_driver)

        # (3) 무조코 실행 버튼 (추가됨!)
        self.btn_mujoco = self.create_button("🦾 Start MuJoCo", "#FF9800", self.toggle_mujoco)
        layout.addWidget(self.btn_mujoco)

        # (4) PlotJuggler 실행 버튼
        self.btn_plot = self.create_button("📈 Open PlotJuggler", "#2196F3", self.launch_plotjuggler)
        layout.addWidget(self.btn_plot)

        layout.addStretch()

        # 3. 비상 정지 (E-STOP)
        btn_stop = QPushButton("🛑 EMERGENCY STOP")
        btn_stop.setFont(QFont("Arial", 14, QFont.Weight.Bold))
        btn_stop.setStyleSheet("""
            QPushButton { background-color: #f44336; color: white; border-radius: 10px; padding: 15px; }
            QPushButton:hover { background-color: #d32f2f; }
        """)
        btn_stop.clicked.connect(self.emergency_stop)
        layout.addWidget(btn_stop)

    def create_button(self, text, color, func):
        btn = QPushButton(text)
        btn.setFont(QFont("Arial", 12))
        btn.setStyleSheet(f"""
            QPushButton {{ background-color: {color}; color: white; border-radius: 8px; padding: 10px; }}
            QPushButton:hover {{ background-color: white; color: {color}; }}
        """)
        btn.clicked.connect(func)
        return btn

    # --- 기능 함수들 ---

    def spin_ros(self):
        # 1. ROS 이벤트 처리
        rclpy.spin_once(self.node, timeout_sec=0)
        
        # 2. (추가됨) 실행 중인 프로세스가 죽었는지 감시
        # 창의 [X] 버튼이나 'q'를 눌러서 껐을 때, 대시보드 버튼도 같이 꺼지게 함
        self.check_process_alive("vision", self.btn_vision, "👁️ Start Vision Node", "#9C27B0")
        self.check_process_alive("driver", self.btn_driver, "🧠 Start Driver Node", "#4CAF50")
        self.check_process_alive("mujoco", self.btn_mujoco, "🦾 Start MuJoCo", "#FF9800")

    def check_process_alive(self, key, button, default_text, default_color):
        """프로세스가 종료되었는지 확인하고 UI 업데이트"""
        if key in self.processes:
            # poll()이 None이 아니면 프로세스가 종료된 것임
            if self.processes[key].poll() is not None:
                del self.processes[key] # 목록에서 삭제
                button.setText(default_text)
                button.setStyleSheet(f"background-color: {default_color}; color: white; border-radius: 8px; padding: 10px;")
                print(f"Process {key} stopped.")

    def toggle_vision(self):
        # (추가됨) 비전 노드 실행/종료 로직
        if "vision" in self.processes:
            self.kill_process("vision")
            # UI 업데이트는 spin_ros에서 자동으로 처리하거나 여기서 즉시 처리
            self.btn_vision.setText("👁️ Start Vision Node")
            self.btn_vision.setStyleSheet("background-color: #9C27B0; color: white; border-radius: 8px; padding: 10px;")
        else:
            # setup.py에 등록된 'vision_node' 실행
            proc = subprocess.Popen(["ros2", "run", "ah_bot_dashboard", "vision_node"])
            self.processes["vision"] = proc
            self.btn_vision.setText("⏹ Stop Vision")
            self.btn_vision.setStyleSheet("background-color: #555; color: white; border-radius: 8px; padding: 10px;")

    def toggle_driver(self):
        if "driver" in self.processes:
            self.kill_process("driver")
            self.btn_driver.setText("🧠 Start Driver Node")
            self.btn_driver.setStyleSheet("background-color: #4CAF50; color: white; border-radius: 8px; padding: 10px;")
            self.status_label.setText("System Status: 🟡 Ready")
        else:
            proc = subprocess.Popen(["ros2", "run", "ah_bot_driver", "driver_node"])
            self.processes["driver"] = proc
            self.btn_driver.setText("⏹ Stop Driver Node")
            self.btn_driver.setStyleSheet("background-color: #FF9800; color: white; border-radius: 8px; padding: 10px;")
            self.status_label.setText("System Status: 🟢 Driver Running")

    def toggle_mujoco(self):
        # (추가됨) 무조코 실행/종료 로직
        if "mujoco" in self.processes:
            self.kill_process("mujoco")
            self.btn_mujoco.setText("🦾 Start MuJoCo")
            self.btn_mujoco.setStyleSheet("background-color: #FF9800; color: white; border-radius: 8px; padding: 10px;")
        else:
            # ah_bot_mujoco 패키지 실행
            proc = subprocess.Popen(["ros2", "run", "ah_bot_mujoco", "mujoco_node"])
            self.processes["mujoco"] = proc
            self.btn_mujoco.setText("⏹ Stop MuJoCo")
            self.btn_mujoco.setStyleSheet("background-color: #555; color: white; border-radius: 8px; padding: 10px;")

    def launch_plotjuggler(self):
        subprocess.Popen(["ros2", "run", "plotjuggler", "plotjuggler"])

    def kill_process(self, name):
        if name in self.processes:
            proc = self.processes[name]
            proc.terminate() # 부드러운 종료
            try:
                proc.wait(timeout=2)
            except subprocess.TimeoutExpired:
                proc.kill() # 강제 종료
            
            if name in self.processes:
                del self.processes[name]

    def emergency_stop(self):
        # 모든 프로세스 강제 종료
        for name in list(self.processes.keys()):
            self.kill_process(name)
        
        # 모든 버튼 상태 초기화
        self.btn_vision.setText("👁️ Start Vision Node")
        self.btn_vision.setStyleSheet("background-color: #9C27B0; color: white; border-radius: 8px; padding: 10px;")
        
        self.btn_driver.setText("🧠 Start Driver Node")
        self.btn_driver.setStyleSheet("background-color: #4CAF50; color: white; border-radius: 8px; padding: 10px;")
        
        self.btn_mujoco.setText("🦾 Start MuJoCo")
        self.btn_mujoco.setStyleSheet("background-color: #FF9800; color: white; border-radius: 8px; padding: 10px;")

        self.status_label.setText("System Status: 🔴 STOPPED")
        QMessageBox.critical(self, "EMERGENCY", "All systems confirmed stopped.")

    def closeEvent(self, event):
        # 창 닫을 때 자식 프로세스도 같이 끄기
        self.emergency_stop()
        self.node.destroy_node()
        rclpy.shutdown()
        event.accept()

def main():
    app = QApplication(sys.argv)
    win = RobotDashboard()
    win.show()
    sys.exit(app.exec())

if __name__ == '__main__':
    main()
