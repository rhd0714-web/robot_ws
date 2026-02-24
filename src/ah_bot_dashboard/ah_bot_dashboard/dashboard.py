#!/usr/bin/env python3
"""
dashboard.py — AH Bot 통합 대시보드

추가 기능:
  1. 캘리브레이션 버튼 → hand_calib.py 실행
  2. Mode 드롭다운     → config.yaml의 mode 실시간 변경 (simulation/hardware/both)
  3. 토픽 모니터창    → /hand_control, /joint_commands 실시간 수치 표시
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
from PyQt6.QtCore import QTimer, Qt, QThread, pyqtSignal
from PyQt6.QtGui import QFont, QColor

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


# ── ROS2 토픽 수신 스레드 ────────────────────────────────────
class TopicMonitorNode(Node):
    """
    /hand_control, /joint_commands 두 토픽을 구독해서
    최신 데이터를 저장만 함. UI 갱신은 QTimer가 polling.
    """
    def __init__(self):
        super().__init__('dashboard_monitor')
        self.hand_data  = []   # Float32 8개
        self.joint_data = []   # Float64 8개

        self.create_subscription(
            Float32MultiArray, '/hand_control',
            lambda msg: setattr(self, 'hand_data', list(msg.data)), 10)
        self.create_subscription(
            Float64MultiArray, '/joint_commands',
            lambda msg: setattr(self, 'joint_data', list(msg.data)), 10)


# ── 토픽 모니터 위젯 ─────────────────────────────────────────
class TopicMonitorWidget(QGroupBox):
    def __init__(self):
        super().__init__("📊 실시간 토픽 모니터")
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

        # 헤더
        for col, text in enumerate(["손가락", "Bend (0~1)", "Side (-1~1)",
                                     "M1 (rad)", "M2 (rad)"]):
            lbl = QLabel(text)
            lbl.setStyleSheet("color: #888; font-size: 10px;")
            lbl.setAlignment(Qt.AlignmentFlag.AlignCenter)
            layout.addWidget(lbl, 0, col)

        # 손가락별 행
        self.bend_bars  = []
        self.side_lbls  = []
        self.m1_lbls    = []
        self.m2_lbls    = []

        for i, name in enumerate(FINGER_NAMES):
            row = i + 1

            name_lbl = QLabel(name)
            name_lbl.setStyleSheet("color: #ccc; font-size: 11px;")
            layout.addWidget(name_lbl, row, 0)

            # Bend 진행바
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

            # Side
            sl = QLabel("0.00")
            sl.setAlignment(Qt.AlignmentFlag.AlignCenter)
            sl.setStyleSheet("color: #ff9800; font-size: 11px; font-family: monospace;")
            layout.addWidget(sl, row, 2)
            self.side_lbls.append(sl)

            # M1
            m1 = QLabel("0.000")
            m1.setAlignment(Qt.AlignmentFlag.AlignCenter)
            m1.setStyleSheet("color: #4caf50; font-size: 11px; font-family: monospace;")
            layout.addWidget(m1, row, 3)
            self.m1_lbls.append(m1)

            # M2
            m2 = QLabel("0.000")
            m2.setAlignment(Qt.AlignmentFlag.AlignCenter)
            m2.setStyleSheet("color: #4caf50; font-size: 11px; font-family: monospace;")
            layout.addWidget(m2, row, 4)
            self.m2_lbls.append(m2)

    def update_data(self, hand_data: list, joint_data: list):
        for i in range(4):
            # /hand_control: [bend0, side0, bend1, side1, ...]
            if len(hand_data) == 8:
                bend = hand_data[i * 2]
                side = hand_data[i * 2 + 1]
                self.bend_bars[i].setValue(int(bend * 100))
                self.side_lbls[i].setText(f"{side:+.2f}")

            # /joint_commands: [m1_0, m2_0, m1_1, m2_1, ...]
            if len(joint_data) == 8:
                m1 = joint_data[i * 2]
                m2 = joint_data[i * 2 + 1]
                self.m1_lbls[i].setText(f"{m1:+.3f}")
                self.m2_lbls[i].setText(f"{m2:+.3f}")


# ── 메인 대시보드 ────────────────────────────────────────────
class RobotDashboard(QMainWindow):
    def __init__(self):
        super().__init__()

        rclpy.init(args=None)
        self.monitor_node = TopicMonitorNode()

        self.processes = {}
        self.NODE_NAMES = {
            "vision"  : "vision_node",
            "driver"  : "driver_node",
            "mujoco"  : "mujoco_node",
            "calib"   : "hand_calib",
        }

        self.init_ui()

        # 500ms마다 프로세스 상태 + 토픽 데이터 갱신
        self.timer = QTimer()
        self.timer.timeout.connect(self.update_all)
        self.timer.start(500)

    # ── UI 구성 ──────────────────────────────────────────────
    def init_ui(self):
        self.setWindowTitle("AH Bot Commander 🤖")
        self.setGeometry(100, 100, 480, 720)
        self.setStyleSheet("background-color: #1e1e1e; color: white;")

        central = QWidget()
        self.setCentralWidget(central)
        root = QVBoxLayout(central)
        root.setSpacing(10)
        root.setContentsMargins(20, 20, 20, 20)

        # ── 헤더 ─────────────────────────────────────────────
        title = QLabel("ROBOT DASHBOARD")
        title.setAlignment(Qt.AlignmentFlag.AlignCenter)
        title.setFont(QFont("Arial", 18, QFont.Weight.Bold))
        title.setStyleSheet("color: #00bcd4; margin-bottom: 4px;")
        root.addWidget(title)

        self.status_label = QLabel("System Status: 🟡 Ready")
        self.status_label.setAlignment(Qt.AlignmentFlag.AlignCenter)
        self.status_label.setStyleSheet(
            "background-color: #2a2a2a; padding: 6px; border-radius: 5px; font-size: 12px;")
        root.addWidget(self.status_label)

        root.addWidget(self._hline())

        # ── Mode 선택 ─────────────────────────────────────────
        mode_row = QHBoxLayout()
        mode_lbl = QLabel("실행 모드:")
        mode_lbl.setStyleSheet("font-size: 12px; color: #aaa;")
        mode_row.addWidget(mode_lbl)

        self.mode_combo = QComboBox()
        self.mode_combo.addItems(["simulation", "hardware", "both"])
        self.mode_combo.setStyleSheet("""
            QComboBox {
                background: #333; color: white; border: 1px solid #555;
                border-radius: 5px; padding: 4px 10px; font-size: 12px;
            }
            QComboBox::drop-down { border: none; }
            QComboBox QAbstractItemView { background: #333; color: white; }
        """)
        # 현재 config의 mode로 초기화
        current_mode = read_config().get('mode', 'simulation')
        idx = MODES.index(current_mode) if current_mode in MODES else 0
        self.mode_combo.setCurrentIndex(idx)
        self.mode_combo.currentTextChanged.connect(self.on_mode_changed)
        mode_row.addWidget(self.mode_combo)
        root.addLayout(mode_row)

        root.addWidget(self._hline())

        # ── 노드 버튼들 ───────────────────────────────────────
        self.btn_vision = self._make_btn(
            "👁️  Vision Node", "#9C27B0", self.toggle_vision)
        root.addWidget(self.btn_vision)

        self.btn_driver = self._make_btn(
            "🧠  Driver Node", "#4CAF50", self.toggle_driver)
        root.addWidget(self.btn_driver)

        self.btn_mujoco = self._make_btn(
            "🦾  MuJoCo Sim", "#FF9800", self.toggle_mujoco)
        root.addWidget(self.btn_mujoco)

        self.btn_plot = self._make_btn(
            "📈  PlotJuggler", "#2196F3", self.launch_plotjuggler)
        root.addWidget(self.btn_plot)

        root.addWidget(self._hline())

        # ── 캘리브레이션 버튼 ─────────────────────────────────
        self.btn_calib = self._make_btn(
            "🎯  캘리브레이션 (hand_calib.py)", "#607D8B", self.toggle_calib)
        self.btn_calib.setToolTip(
            "손을 카메라 앞에 놓고 실행\n"
            "[R] 측정시작/정지  [S] config.yaml 저장  [Q] 종료"
        )
        root.addWidget(self.btn_calib)

        root.addWidget(self._hline())

        # ── 토픽 모니터 ───────────────────────────────────────
        self.topic_monitor = TopicMonitorWidget()
        root.addWidget(self.topic_monitor)

        root.addStretch()

        # ── KILL ALL ──────────────────────────────────────────
        btn_kill = QPushButton("🛑  KILL ALL NODES")
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
                border-radius: 7px; padding: 11px; text-align: left; padding-left: 16px;
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

    # ── Mode 변경 ─────────────────────────────────────────────
    def on_mode_changed(self, mode: str):
        write_mode(mode)
        self.status_label.setText(f"System Status: ✅ 모드 변경됨 → {mode}")

    # ── 주기적 업데이트 ───────────────────────────────────────
    def update_all(self):
        # ROS2 콜백 처리
        rclpy.spin_once(self.monitor_node, timeout_sec=0)

        # 토픽 모니터 갱신
        self.topic_monitor.update_data(
            self.monitor_node.hand_data,
            self.monitor_node.joint_data
        )

        # 프로세스 종료 감지 → 버튼 UI 자동 복원
        self._check_proc("vision", self.btn_vision, "👁️  Vision Node", "#9C27B0")
        self._check_proc("driver", self.btn_driver, "🧠  Driver Node", "#4CAF50")
        self._check_proc("mujoco", self.btn_mujoco, "🦾  MuJoCo Sim",  "#FF9800")
        self._check_proc("calib",  self.btn_calib,
                         "🎯  캘리브레이션 (hand_calib.py)", "#607D8B")

    def _check_proc(self, key, btn, label, color):
        if key in self.processes and self.processes[key].poll() is not None:
            del self.processes[key]
            btn.setText(label)
            btn.setStyleSheet(self._stopped_style(color))
            if key == "driver":
                self.status_label.setText("System Status: 🟡 Ready")

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
            time.sleep(0.1)
            self.processes[key] = subprocess.Popen(cmd)
            btn.setText(f"⏹  {label.split('  ', 1)[-1]} (실행 중)")
            btn.setStyleSheet(self._running_style())

    def toggle_vision(self):
        self._toggle("vision", self.btn_vision, "👁️  Vision Node", "#9C27B0",
                     ["ros2", "run", "ah_bot_dashboard", "vision_node",
                      "--ros-args", "-p", f"config_path:={CONFIG_PATH}"])

    def toggle_driver(self):
        if "driver" in self.processes:
            self.force_kill(self.NODE_NAMES["driver"])
            if "driver" in self.processes:
                del self.processes["driver"]
            self.btn_driver.setText("🧠  Driver Node")
            self.btn_driver.setStyleSheet(self._stopped_style("#4CAF50"))
            self.status_label.setText("System Status: 🟡 Ready")
        else:
            self.force_kill(self.NODE_NAMES["driver"])
            time.sleep(0.1)
            self.processes["driver"] = subprocess.Popen(
                ["ros2", "run", "ah_bot_driver", "driver_node",
                 "--ros-args", "-p", f"config_path:={CONFIG_PATH}"]
            )
            self.btn_driver.setText("⏹  Driver Node (실행 중)")
            self.btn_driver.setStyleSheet(self._running_style())
            mode = self.mode_combo.currentText()
            self.status_label.setText(f"System Status: 🟢 Driver Running [{mode}]")

    def toggle_mujoco(self):
        self._toggle("mujoco", self.btn_mujoco, "🦾  MuJoCo Sim", "#FF9800",
                     ["ros2", "run", "ah_bot_mujoco", "mujoco_node"])

    def toggle_calib(self):
        """캘리브레이션 툴 실행/종료"""
        self._toggle("calib", self.btn_calib,
                     "🎯  캘리브레이션 (hand_calib.py)", "#607D8B",
                     ["python3", CALIB_PATH])

    def launch_plotjuggler(self):
        layout = os.path.join(WS_ROOT, 'test_layout.xml')
        cmd = ["ros2", "run", "plotjuggler", "plotjuggler"]
        if os.path.exists(layout):
            cmd += ["--layout", layout]
        subprocess.Popen(cmd)

    def force_kill(self, keyword):
        try:
            subprocess.run(["pkill", "-f", keyword], check=False)
        except Exception as e:
            print(f"kill error: {e}")

    def emergency_stop(self):
        self.status_label.setText("System Status: 🔴 KILLING ALL...")
        QApplication.processEvents()
        for proc in self.processes.values():
            proc.terminate()
        for name in self.NODE_NAMES.values():
            self.force_kill(name)
        self.processes = {}
        self.status_label.setText("System Status: 🔴 STOPPED")

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