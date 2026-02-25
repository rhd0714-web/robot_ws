#!/usr/bin/env python3
"""
dashboard.py — AH Bot 통합 대시보드

수정사항:
  1. 이모지 제거 → 텍스트로 교체 (Docker 환경 폰트 문제 해결)
  2. Mode 변경 시 Driver 자동 재시작 (모드 즉시 반영)
  3. KILL ALL 후 재시작 시 포트 충돌 방지 (딜레이 추가)
"""

import sys
import subprocess
import os
import time
import yaml

from PyQt6.QtWidgets import (
    QApplication, QMainWindow, QWidget, QVBoxLayout, QHBoxLayout,
    QPushButton, QLabel, QFrame, QComboBox, QGroupBox, QGridLayout,
    QProgressBar,
)
from PyQt6.QtCore import QTimer, Qt
from PyQt6.QtGui import QFont

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray, Float64MultiArray

# ── 경로 설정 ────────────────────────────────────────────────
WS_ROOT     = os.path.expanduser('~/robot_ws')
CONFIG_PATH = os.path.join(WS_ROOT, 'config', 'config.yaml')
CALIB_PATH  = os.path.join(WS_ROOT, 'hand_calib.py')

FINGER_NAMES = ["Index", "Middle", "Ring", "Thumb"]
MODES        = ["simulation", "hardware", "both"]


# ── config.yaml 읽기/쓰기 ────────────────────────────────────
def read_config() -> dict:
    if os.path.exists(CONFIG_PATH):
        with open(CONFIG_PATH, 'r') as f:
            return yaml.safe_load(f) or {}
    return {}


def write_mode(mode: str):
    cfg = read_config()
    cfg['mode'] = mode
    with open(CONFIG_PATH, 'w') as f:
        yaml.dump(cfg, f, default_flow_style=False, allow_unicode=True)


# ── ROS2 토픽 모니터 노드 ────────────────────────────────────
class TopicMonitorNode(Node):
    def __init__(self):
        super().__init__('dashboard_monitor')
        self.hand_data  = []
        self.joint_data = []
        self.create_subscription(
            Float32MultiArray, '/hand_control',
            lambda msg: setattr(self, 'hand_data', list(msg.data)), 10)
        self.create_subscription(
            Float64MultiArray, '/joint_commands',
            lambda msg: setattr(self, 'joint_data', list(msg.data)), 10)


# ── 토픽 모니터 위젯 ─────────────────────────────────────────
class TopicMonitorWidget(QGroupBox):
    def __init__(self):
        # [수정] 이모지 제거 → 텍스트로 교체
        super().__init__("[MONITOR] Real-time Topic Data")
        self.setStyleSheet("""
            QGroupBox {
                border: 1px solid #444;
                border-radius: 6px;
                margin-top: 8px;
                color: #aaa;
                font-size: 11px;
            }
            QGroupBox::title {
                subcontrol-origin: margin;
                left: 10px;
                padding: 0 6px;
                color: #00bcd4;
            }
        """)

        layout = QGridLayout()
        layout.setSpacing(4)
        self.setLayout(layout)

        for col, text in enumerate(["Finger", "Bend (0~1)", "Side (-1~1)",
                                     "M1 (rad)", "M2 (rad)"]):
            lbl = QLabel(text)
            lbl.setStyleSheet("color: #888; font-size: 10px;")
            lbl.setAlignment(Qt.AlignmentFlag.AlignCenter)
            layout.addWidget(lbl, 0, col)

        self.bend_bars = []
        self.side_lbls = []
        self.m1_lbls   = []
        self.m2_lbls   = []

        for i, name in enumerate(FINGER_NAMES):
            row = i + 1

            name_lbl = QLabel(name)
            name_lbl.setStyleSheet("color: #ccc; font-size: 11px;")
            layout.addWidget(name_lbl, row, 0)

            bar = QProgressBar()
            bar.setRange(0, 100)
            bar.setValue(0)
            bar.setTextVisible(False)
            bar.setFixedHeight(14)
            bar.setStyleSheet("""
                QProgressBar { background: #333; border-radius: 3px; }
                QProgressBar::chunk { background: #00bcd4; border-radius: 3px; }
            """)
            layout.addWidget(bar, row, 1)
            self.bend_bars.append(bar)

            sl = QLabel("0.00")
            sl.setAlignment(Qt.AlignmentFlag.AlignCenter)
            sl.setStyleSheet(
                "color: #ff9800; font-size: 11px; font-family: monospace;")
            layout.addWidget(sl, row, 2)
            self.side_lbls.append(sl)

            m1 = QLabel("0.000")
            m1.setAlignment(Qt.AlignmentFlag.AlignCenter)
            m1.setStyleSheet(
                "color: #4caf50; font-size: 11px; font-family: monospace;")
            layout.addWidget(m1, row, 3)
            self.m1_lbls.append(m1)

            m2 = QLabel("0.000")
            m2.setAlignment(Qt.AlignmentFlag.AlignCenter)
            m2.setStyleSheet(
                "color: #4caf50; font-size: 11px; font-family: monospace;")
            layout.addWidget(m2, row, 4)
            self.m2_lbls.append(m2)

    def update_data(self, hand_data: list, joint_data: list):
        for i in range(4):
            if len(hand_data) == 8:
                self.bend_bars[i].setValue(int(hand_data[i * 2] * 100))
                self.side_lbls[i].setText(f"{hand_data[i * 2 + 1]:+.2f}")
            if len(joint_data) == 8:
                self.m1_lbls[i].setText(f"{joint_data[i * 2]:+.3f}")
                self.m2_lbls[i].setText(f"{joint_data[i * 2 + 1]:+.3f}")


# ── 메인 대시보드 ────────────────────────────────────────────
class RobotDashboard(QMainWindow):
    def __init__(self):
        super().__init__()

        rclpy.init(args=None)
        self.monitor_node = TopicMonitorNode()

        self.processes = {}
        self.NODE_NAMES = {
            "vision": "vision_node",
            "driver": "driver_node",
            "mujoco": "mujoco_node",
            "calib" : "hand_calib",
        }

        self.init_ui()

        self.timer = QTimer()
        self.timer.timeout.connect(self.update_all)
        self.timer.start(500)

    def init_ui(self):
        # [수정] 창 제목 이모지 제거
        self.setWindowTitle("AH Bot Commander")
        self.setGeometry(100, 100, 480, 740)
        self.setStyleSheet("background-color: #1e1e1e; color: white;")

        central = QWidget()
        self.setCentralWidget(central)
        root = QVBoxLayout(central)
        root.setSpacing(10)
        root.setContentsMargins(20, 20, 20, 20)

        # 헤더
        title = QLabel("ROBOT DASHBOARD")
        title.setAlignment(Qt.AlignmentFlag.AlignCenter)
        title.setFont(QFont("Arial", 18, QFont.Weight.Bold))
        title.setStyleSheet("color: #00bcd4; margin-bottom: 4px;")
        root.addWidget(title)

        self.status_label = QLabel("System Status: [READY]")
        self.status_label.setAlignment(Qt.AlignmentFlag.AlignCenter)
        self.status_label.setStyleSheet(
            "background-color: #2a2a2a; padding: 6px; "
            "border-radius: 5px; font-size: 12px;")
        root.addWidget(self.status_label)

        root.addWidget(self._hline())

        # Mode 선택
        mode_row = QHBoxLayout()
        mode_lbl = QLabel("Mode:")
        mode_lbl.setStyleSheet("font-size: 12px; color: #aaa;")
        mode_row.addWidget(mode_lbl)

        self.mode_combo = QComboBox()
        self.mode_combo.addItems(MODES)
        self.mode_combo.setStyleSheet("""
            QComboBox {
                background: #333; color: white; border: 1px solid #555;
                border-radius: 5px; padding: 4px 10px; font-size: 12px;
            }
            QComboBox::drop-down { border: none; }
            QComboBox QAbstractItemView { background: #333; color: white; }
        """)
        current_mode = read_config().get('mode', 'simulation')
        idx = MODES.index(current_mode) if current_mode in MODES else 0
        self.mode_combo.setCurrentIndex(idx)
        # [수정] 시그널 연결 전에 인덱스 설정 (초기화 시 on_mode_changed 호출 방지)
        self.mode_combo.currentTextChanged.connect(self.on_mode_changed)
        mode_row.addWidget(self.mode_combo)

        # [추가] 모드 설명 라벨
        self.mode_desc_label = QLabel(self._mode_desc(current_mode))
        self.mode_desc_label.setStyleSheet("font-size: 10px; color: #666;")
        mode_row.addWidget(self.mode_desc_label)

        root.addLayout(mode_row)
        root.addWidget(self._hline())

        # [수정] 버튼 텍스트 이모지 제거
        self.btn_vision = self._make_btn(
            "[CAM]  Vision Node", "#9C27B0", self.toggle_vision)
        root.addWidget(self.btn_vision)

        self.btn_driver = self._make_btn(
            "[DRV]  Driver Node", "#4CAF50", self.toggle_driver)
        root.addWidget(self.btn_driver)

        self.btn_mujoco = self._make_btn(
            "[SIM]  MuJoCo Sim", "#FF9800", self.toggle_mujoco)
        root.addWidget(self.btn_mujoco)

        self.btn_plot = self._make_btn(
            "[PLT]  PlotJuggler", "#2196F3", self.launch_plotjuggler)
        root.addWidget(self.btn_plot)

        root.addWidget(self._hline())

        self.btn_calib = self._make_btn(
            "[CAL]  Calibration (hand_calib.py)", "#607D8B", self.toggle_calib)
        self.btn_calib.setToolTip(
            "손을 카메라 앞에 놓고 실행\n"
            "1단계: 손 펴기 → 2단계: 주먹 → 자동 저장"
        )
        root.addWidget(self.btn_calib)

        root.addWidget(self._hline())

        self.topic_monitor = TopicMonitorWidget()
        root.addWidget(self.topic_monitor)

        root.addStretch()

        # KILL ALL
        btn_kill = QPushButton("KILL ALL NODES")
        btn_kill.setFont(QFont("Arial", 13, QFont.Weight.Bold))
        btn_kill.setStyleSheet("""
            QPushButton {
                background-color: #c62828; color: white;
                border-radius: 8px; padding: 14px;
            }
            QPushButton:hover { background-color: #e53935; }
        """)
        btn_kill.clicked.connect(self.emergency_stop)
        root.addWidget(btn_kill)

    def _hline(self):
        line = QFrame()
        line.setFrameShape(QFrame.Shape.HLine)
        line.setStyleSheet("color: #333;")
        return line

    def _make_btn(self, text, color, func):
        btn = QPushButton(text)
        btn.setFont(QFont("Arial", 11))
        btn.setStyleSheet(f"""
            QPushButton {{
                background-color: {color}; color: white;
                border-radius: 7px; padding: 11px;
                text-align: left; padding-left: 16px;
            }}
            QPushButton:hover {{ background-color: #555; color: white; }}
        """)
        btn.clicked.connect(func)
        return btn

    def _stopped_style(self, color):
        return (f"background-color: {color}; color: white; border-radius: 7px; "
                f"padding: 11px; text-align: left; padding-left: 16px;")

    def _running_style(self):
        return ("background-color: #37474f; color: #80cbc4; border-radius: 7px; "
                "padding: 11px; text-align: left; padding-left: 16px; "
                "border: 1px solid #00bcd4;")

    def _mode_desc(self, mode: str) -> str:
        return {"simulation": "MuJoCo only",
                "hardware"  : "Real motor only",
                "both"      : "Sim + Motor"}.get(mode, "")

    # ── Mode 변경 ─────────────────────────────────────────────
    def on_mode_changed(self, mode: str):
        """
        [수정] Mode 변경 시:
          1. config.yaml 저장
          2. Driver가 켜져 있으면 자동 재시작 (새 모드 즉시 반영)
        """
        write_mode(mode)
        self.mode_desc_label.setText(self._mode_desc(mode))
        self.status_label.setText(f"System Status: [MODE] {mode}")

        # Driver가 실행 중이면 재시작
        if "driver" in self.processes:
            self.status_label.setText(
                f"System Status: [MODE] {mode} - Driver 재시작 중...")
            QApplication.processEvents()

            self.force_kill(self.NODE_NAMES["driver"])
            time.sleep(0.5)   # 포트 해제 대기

            self.processes["driver"] = subprocess.Popen(
                ["ros2", "run", "ah_bot_driver", "driver_node",
                 "--ros-args", "-p", f"config_path:={CONFIG_PATH}"]
            )
            self.status_label.setText(
                f"System Status: [ON] Driver Running [{mode}]")

    # ── 주기적 업데이트 ───────────────────────────────────────
    def update_all(self):
        rclpy.spin_once(self.monitor_node, timeout_sec=0)
        self.topic_monitor.update_data(
            self.monitor_node.hand_data,
            self.monitor_node.joint_data
        )
        self._check_proc("vision", self.btn_vision,
                         "[CAM]  Vision Node", "#9C27B0")
        self._check_proc("driver", self.btn_driver,
                         "[DRV]  Driver Node", "#4CAF50")
        self._check_proc("mujoco", self.btn_mujoco,
                         "[SIM]  MuJoCo Sim",  "#FF9800")
        self._check_proc("calib",  self.btn_calib,
                         "[CAL]  Calibration (hand_calib.py)", "#607D8B")

    def _check_proc(self, key, btn, label, color):
        if key in self.processes and self.processes[key].poll() is not None:
            del self.processes[key]
            btn.setText(label)
            btn.setStyleSheet(self._stopped_style(color))
            if key == "driver":
                self.status_label.setText("System Status: [READY]")

    # ── 노드 토글 ─────────────────────────────────────────────
    def _toggle(self, key, btn, label, color, cmd):
        if key in self.processes:
            self.force_kill(self.NODE_NAMES[key])
            if key in self.processes:
                del self.processes[key]
            btn.setText(label)
            btn.setStyleSheet(self._stopped_style(color))
        else:
            self.force_kill(self.NODE_NAMES[key])
            time.sleep(0.2)   # [수정] 포트 충돌 방지 딜레이 증가
            self.processes[key] = subprocess.Popen(cmd)
            btn.setText(f"[ON]  {label.split(']  ', 1)[-1]}")
            btn.setStyleSheet(self._running_style())

    def toggle_vision(self):
        self._toggle("vision", self.btn_vision, "[CAM]  Vision Node", "#9C27B0",
                     ["ros2", "run", "ah_bot_dashboard", "vision_node",
                      "--ros-args", "-p", f"config_path:={CONFIG_PATH}"])

    def toggle_driver(self):
        if "driver" in self.processes:
            self.force_kill(self.NODE_NAMES["driver"])
            if "driver" in self.processes:
                del self.processes["driver"]
            self.btn_driver.setText("[DRV]  Driver Node")
            self.btn_driver.setStyleSheet(self._stopped_style("#4CAF50"))
            self.status_label.setText("System Status: [READY]")
        else:
            self.force_kill(self.NODE_NAMES["driver"])
            time.sleep(0.2)
            self.processes["driver"] = subprocess.Popen(
                ["ros2", "run", "ah_bot_driver", "driver_node",
                 "--ros-args", "-p", f"config_path:={CONFIG_PATH}"]
            )
            self.btn_driver.setText("[ON]  Driver Node")
            self.btn_driver.setStyleSheet(self._running_style())
            mode = self.mode_combo.currentText()
            self.status_label.setText(
                f"System Status: [ON] Driver Running [{mode}]")

    def toggle_mujoco(self):
        self._toggle("mujoco", self.btn_mujoco, "[SIM]  MuJoCo Sim", "#FF9800",
                     ["ros2", "run", "ah_bot_mujoco", "mujoco_node",
                      "--ros-args", "-p", f"config_path:={CONFIG_PATH}"])

    def toggle_calib(self):
        self._toggle("calib", self.btn_calib,
                     "[CAL]  Calibration (hand_calib.py)", "#607D8B",
                     ["python3", CALIB_PATH])

    def launch_plotjuggler(self):
        layout_path = os.path.join(WS_ROOT, 'test_layout.xml')
        cmd = ["ros2", "run", "plotjuggler", "plotjuggler"]
        if os.path.exists(layout_path):
            cmd += ["--layout", layout_path]
        subprocess.Popen(cmd)

    def force_kill(self, keyword):
        try:
            subprocess.run(["pkill", "-f", keyword], check=False)
        except Exception as e:
            print(f"kill error: {e}")

    def emergency_stop(self):
        self.status_label.setText("System Status: [STOP] Killing all...")
        QApplication.processEvents()
        for proc in self.processes.values():
            proc.terminate()
        for name in self.NODE_NAMES.values():
            self.force_kill(name)
        self.processes = {}
        # [수정] 포트 완전 해제 대기
        time.sleep(0.5)
        self.status_label.setText("System Status: [STOP] All stopped")

    def closeEvent(self, event):
        self.emergency_stop()
        self.monitor_node.destroy_node()
        rclpy.shutdown()
        event.accept()


def main():
    app = QApplication(sys.argv)
    win = RobotDashboard()
    win.show()
    sys.exit(app.exec())


if __name__ == '__main__':
    main()