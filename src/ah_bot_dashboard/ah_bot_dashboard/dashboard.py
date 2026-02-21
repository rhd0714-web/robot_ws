#!/usr/bin/env python3
import sys
import subprocess
import os
import signal
import time

# PyQt6 라이브러리
from PyQt6.QtWidgets import (QApplication, QMainWindow, QWidget, QVBoxLayout, 
                             QHBoxLayout, QPushButton, QLabel, QFrame, QMessageBox)
from PyQt6.QtCore import QTimer, Qt
from PyQt6.QtGui import QFont

# ROS 2 라이브러리
import rclpy
from rclpy.node import Node

class RobotDashboard(QMainWindow):
    def __init__(self):
        super().__init__()
        
        rclpy.init(args=None)
        self.node = Node('ah_bot_dashboard')
        
        # 프로세스 객체를 저장할 딕셔너리
        self.processes = {} 
        
        # [핵심] 각 노드의 실제 프로세스 이름 (pkill용)
        # setup.py의 entry_points에 등록된 이름과 일치해야 함
        self.NODE_NAMES = {
            "vision": "vision_node",
            "driver": "driver_node",
            "mujoco": "mujoco_node"
        }

        self.init_ui()
        
        # 0.5초마다 프로세스 상태 확인 (UI 동기화)
        self.timer = QTimer()
        self.timer.timeout.connect(self.update_status)
        self.timer.start(500)

    def init_ui(self):
        self.setWindowTitle("AH Bot Commander 🤖")
        self.setGeometry(100, 100, 400, 600)
        self.setStyleSheet("background-color: #2b2b2b; color: white;")

        central_widget = QWidget()
        self.setCentralWidget(central_widget)
        layout = QVBoxLayout(central_widget)
        layout.setSpacing(15)
        layout.setContentsMargins(30, 30, 30, 30)

        # 1. 헤더
        title = QLabel("ROBOT DASHBOARD")
        title.setAlignment(Qt.AlignmentFlag.AlignCenter)
        title.setFont(QFont("Arial", 20, QFont.Weight.Bold))
        title.setStyleSheet("color: #00bcd4; margin-bottom: 10px;")
        layout.addWidget(title)
        
        self.status_label = QLabel("System Status: 🟡 Ready")
        self.status_label.setAlignment(Qt.AlignmentFlag.AlignCenter)
        self.status_label.setStyleSheet("background-color: #444; padding: 8px; border-radius: 5px;")
        layout.addWidget(self.status_label)

        line = QFrame()
        line.setFrameShape(QFrame.Shape.HLine)
        line.setStyleSheet("color: #555;")
        layout.addWidget(line)

        # 2. 컨트롤 버튼
        self.btn_vision = self.create_button("👁️ Start Vision Node", "#9C27B0", self.toggle_vision)
        layout.addWidget(self.btn_vision)

        self.btn_driver = self.create_button("🧠 Start Driver Node", "#4CAF50", self.toggle_driver)
        layout.addWidget(self.btn_driver)

        self.btn_mujoco = self.create_button("🦾 Start MuJoCo", "#FF9800", self.toggle_mujoco)
        layout.addWidget(self.btn_mujoco)

        self.btn_plot = self.create_button("📈 Open PlotJuggler", "#2196F3", self.launch_plotjuggler)
        layout.addWidget(self.btn_plot)

        layout.addStretch()

        # 3. 전체 종료 버튼 (Emergency)
        btn_kill_all = QPushButton("🛑 KILL ALL NODES")
        btn_kill_all.setFont(QFont("Arial", 14, QFont.Weight.Bold))
        btn_kill_all.setStyleSheet("""
            QPushButton { background-color: #f44336; color: white; border-radius: 10px; padding: 15px; }
            QPushButton:hover { background-color: #d32f2f; }
        """)
        btn_kill_all.clicked.connect(self.emergency_stop)
        layout.addWidget(btn_kill_all)

    def create_button(self, text, color, func):
        btn = QPushButton(text)
        btn.setFont(QFont("Arial", 12))
        btn.setStyleSheet(f"""
            QPushButton {{ background-color: {color}; color: white; border-radius: 8px; padding: 12px; }}
            QPushButton:hover {{ background-color: white; color: {color}; }}
        """)
        btn.clicked.connect(func)
        return btn

    # --- 프로세스 관리 핵심 로직 ---

    def force_kill(self, process_keyword):
        """운영체제 레벨에서 프로세스를 강제 종료 (좀비 방지)"""
        try:
            # pkill -f [이름]: 이름이 포함된 모든 프로세스 사살
            subprocess.run(["pkill", "-f", process_keyword], check=False)
            print(f"💀 Force killed: {process_keyword}")
        except Exception as e:
            print(f"Error killing {process_keyword}: {e}")

    def update_status(self):
        """백그라운드에서 노드가 죽었는지 확인하고 버튼 상태 동기화"""
        # 1. Vision
        if "vision" in self.processes and self.processes["vision"].poll() is not None:
            del self.processes["vision"]
            self.btn_vision.setText("👁️ Start Vision Node")
            self.btn_vision.setStyleSheet("background-color: #9C27B0; color: white; border-radius: 8px; padding: 12px;")
        
        # 2. Driver (이게 중요! 중복 실행 방지)
        if "driver" in self.processes and self.processes["driver"].poll() is not None:
            del self.processes["driver"]
            self.btn_driver.setText("🧠 Start Driver Node")
            self.btn_driver.setStyleSheet("background-color: #4CAF50; color: white; border-radius: 8px; padding: 12px;")
            self.status_label.setText("System Status: 🟡 Ready")

        # 3. MuJoCo
        if "mujoco" in self.processes and self.processes["mujoco"].poll() is not None:
            del self.processes["mujoco"]
            self.btn_mujoco.setText("🦾 Start MuJoCo")
            self.btn_mujoco.setStyleSheet("background-color: #FF9800; color: white; border-radius: 8px; padding: 12px;")

        rclpy.spin_once(self.node, timeout_sec=0)

    # --- 버튼 동작 ---

    def toggle_vision(self):
        # [끄는 로직]
        if "vision" in self.processes:
            # 1. 운영체제 레벨에서 확실히 죽임
            self.force_kill(self.NODE_NAMES["vision"])
            # 2. 딕셔너리에서 제거 및 UI 업데이트는 update_status가 처리할 수도 있지만
            #    즉각적인 반응을 위해 여기서 처리해도 됨
            if "vision" in self.processes: del self.processes["vision"]
            self.btn_vision.setText("👁️ Start Vision Node")
            self.btn_vision.setStyleSheet("background-color: #9C27B0; color: white; border-radius: 8px; padding: 12px;")
        
        # [켜는 로직]
        else:
            self.force_kill(self.NODE_NAMES["vision"]) # 혹시 모를 좀비 확인 사살
            time.sleep(0.1)
            self.processes["vision"] = subprocess.Popen(["ros2", "run", "ah_bot_dashboard", "vision_node"])
            self.btn_vision.setText("⏹ Stop Vision")
            self.btn_vision.setStyleSheet("background-color: #555; color: white; border-radius: 8px; padding: 12px;")

    def toggle_driver(self):
        # [끄는 로직]
        if "driver" in self.processes:
            self.force_kill(self.NODE_NAMES["driver"]) # pkill로 강제 종료!
            if "driver" in self.processes: del self.processes["driver"]
            
            self.btn_driver.setText("🧠 Start Driver Node")
            self.btn_driver.setStyleSheet("background-color: #4CAF50; color: white; border-radius: 8px; padding: 12px;")
            self.status_label.setText("System Status: 🟡 Ready")
        
        # [켜는 로직]
        else:
            self.force_kill(self.NODE_NAMES["driver"])
            time.sleep(0.1)
            self.processes["driver"] = subprocess.Popen(["ros2", "run", "ah_bot_driver", "driver_node"])
            self.btn_driver.setText("⏹ Stop Driver Node")
            self.btn_driver.setStyleSheet("background-color: #555; color: white; border-radius: 8px; padding: 12px;")
            self.status_label.setText("System Status: 🟢 Driver Running")

    def toggle_mujoco(self):
        # [끄는 로직]
        if "mujoco" in self.processes:
            self.force_kill(self.NODE_NAMES["mujoco"])
            if "mujoco" in self.processes: del self.processes["mujoco"]
            
            self.btn_mujoco.setText("🦾 Start MuJoCo")
            self.btn_mujoco.setStyleSheet("background-color: #FF9800; color: white; border-radius: 8px; padding: 12px;")
        
        # [켜는 로직]
        else:
            self.force_kill(self.NODE_NAMES["mujoco"])
            time.sleep(0.1)
            self.processes["mujoco"] = subprocess.Popen(["ros2", "run", "ah_bot_mujoco", "mujoco_node"])
            self.btn_mujoco.setText("⏹ Stop MuJoCo")
            self.btn_mujoco.setStyleSheet("background-color: #555; color: white; border-radius: 8px; padding: 12px;")

    def launch_plotjuggler(self):
        subprocess.Popen(["ros2", "run", "plotjuggler", "plotjuggler"])

    def emergency_stop(self):
        """모든 노드 강제 종료"""
        self.status_label.setText("System Status: 🔴 KILLING ALL...")
        QApplication.processEvents() # UI 갱신

        # 1. 딕셔너리에 있는 것들 종료
        for key, proc in self.processes.items():
            proc.terminate()
        
        # 2. 운영체제 레벨에서 확실히 확인 사살 (pkill)
        for name in self.NODE_NAMES.values():
            self.force_kill(name)
            
        self.processes = {}
        self.status_label.setText("System Status: 🔴 STOPPED")
        
        # 버튼 UI 초기화는 update_status가 알아서 함

    def closeEvent(self, event):
        """대시보드 창 닫을 때 실행"""
        self.emergency_stop() # 다 죽이고
        self.node.destroy_node()
        rclpy.shutdown()
        event.accept() # 창 닫기 허용

def main():
    app = QApplication(sys.argv)
    win = RobotDashboard()
    win.show()
    sys.exit(app.exec())

if __name__ == '__main__':
    main()