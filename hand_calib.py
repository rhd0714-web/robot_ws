"""
hand_calib.py — 자동 안정화 캘리브레이션 툴

개선사항:
  - 손이 정지했을 때만 샘플 수집 (흔들림 제거)
  - 200개 샘플 모이면 자동 저장 (median 사용, 이상치 제거)
  - 단계별 안내 (1단계: 손 펴기 → 2단계: 주먹 쥐기 → 자동 완료)
  - [R] 키로 현재 단계 재측정 가능
"""

import cv2
import mediapipe as mp
import numpy as np
import yaml
import os
from collections import deque

SCRIPT_DIR  = os.path.dirname(os.path.abspath(__file__))
CONFIG_PATH = os.path.join(SCRIPT_DIR, 'config', 'config.yaml')

mp_hands          = mp.solutions.hands
mp_drawing        = mp.solutions.drawing_utils
mp_drawing_styles = mp.solutions.drawing_styles

MP_TIPS      = [8, 12, 16, 4]
MP_MCPS      = [5,  9, 13, 1]
FINGER_NAMES = ["Index", "Middle", "Ring", "Thumb"]
FINGER_KEYS  = ["index", "middle", "ring", "thumb"]

STABILITY_WINDOW = 10
STABILITY_THRESH = 0.003  # 이 이하로 떨림 없으면 "정지"
REQUIRED_SAMPLES = 200    # 이만큼 쌓이면 자동 완료


def build_palm_frame(lm_world):
    wrist     = np.array([lm_world.landmark[0].x,
                          lm_world.landmark[0].y,
                          lm_world.landmark[0].z])
    mid_mcp   = np.array([lm_world.landmark[9].x,
                          lm_world.landmark[9].y,
                          lm_world.landmark[9].z])
    pinky_mcp = np.array([lm_world.landmark[17].x,
                          lm_world.landmark[17].y,
                          lm_world.landmark[17].z])
    unit_z = mid_mcp - wrist
    n = np.linalg.norm(unit_z)
    if n < 1e-6: return np.eye(3)
    unit_z /= n
    unit_x = np.cross(pinky_mcp - wrist, unit_z)
    n = np.linalg.norm(unit_x)
    if n < 1e-6: return np.eye(3)
    unit_x /= n
    unit_y = np.cross(unit_z, unit_x)
    return np.array([unit_x, unit_y, unit_z])


def get_finger_vectors(lm_world, R):
    vecs = []
    for i in range(4):
        tip   = lm_world.landmark[MP_TIPS[i]]
        mcp   = lm_world.landmark[MP_MCPS[i]]
        v     = np.array([tip.x - mcp.x, tip.y - mcp.y, tip.z - mcp.z])
        local = R @ v
        vecs.append((float(local[2]), float(local[0])))
    return vecs


class StabilityDetector:
    """
    최근 N프레임 bend 값 변화량이 임계값 이하면 "정지" 판정.
    진동 감쇠 후 정상상태(steady-state) 판별과 동일한 개념.
    """
    def __init__(self):
        self.history = deque(maxlen=STABILITY_WINDOW)

    def update(self, vecs):
        self.history.append(vecs[0][0])  # 검지 bend로 대표

    def is_stable(self):
        if len(self.history) < STABILITY_WINDOW:
            return False
        return (max(self.history) - min(self.history)) < STABILITY_THRESH

    def reset(self):
        self.history.clear()


def save_to_config(open_bends, open_sides, fist_bends, fist_sides):
    if os.path.exists(CONFIG_PATH):
        with open(CONFIG_PATH, 'r') as f:
            cfg = yaml.safe_load(f) or {}
    else:
        cfg = {}
        os.makedirs(os.path.dirname(CONFIG_PATH), exist_ok=True)

    if 'vision' not in cfg:
        cfg['vision'] = {}
    cfg['vision']['fingers'] = {}

    print("\n✅ 캘리브레이션 결과:")
    for i, key in enumerate(FINGER_KEYS):
        fist_b = float(np.median(fist_bends[i]))
        open_b = float(np.median(open_bends[i]))
        open_s = float(np.median(open_sides[i]))
        fist_s = float(np.median(fist_sides[i]))

        margin_b = abs(open_b - fist_b) * 0.05
        margin_s = max(abs(open_s - fist_s) * 0.05, 0.005)

        bend_range   = [round(fist_b - margin_b, 4),
                        round(open_b + margin_b, 4)]
        spread_range = [round(min(open_s, fist_s) - margin_s, 4),
                        round(max(open_s, fist_s) + margin_s, 4)]

        cfg['vision']['fingers'][key] = {
            'bend_range'  : bend_range,
            'spread_range': spread_range,
        }
        print(f"  [{FINGER_NAMES[i]:<8}] bend={bend_range}  spread={spread_range}")

    with open(CONFIG_PATH, 'w') as f:
        yaml.dump(cfg, f, default_flow_style=False, allow_unicode=True)
    print(f"\n  저장 완료: {CONFIG_PATH}")


def draw_ui(frame, stage, vecs, is_stable, count):
    h, w = frame.shape[:2]
    cv2.rectangle(frame, (0, 0), (w, 95), (20, 20, 20), -1)

    stage_info = {
        1: ("1단계: 손을 완전히 펴고 고정하세요",  (0, 200, 100)),
        2: ("2단계: 주먹을 완전히 쥐고 고정하세요", (0, 120, 255)),
        3: ("✅ 완료! config.yaml 저장됨",          (0, 220, 220)),
    }
    text, color = stage_info.get(stage, ("", (255, 255, 255)))
    cv2.putText(frame, text, (12, 30),
                cv2.FONT_HERSHEY_SIMPLEX, 0.72, color, 2)

    # 안정성
    stab_text  = "✅ 정지 감지 — 측정 중" if is_stable else "⏳ 손을 고정해주세요..."
    stab_color = (0, 220, 100) if is_stable else (120, 120, 120)
    cv2.putText(frame, stab_text, (12, 56),
                cv2.FONT_HERSHEY_SIMPLEX, 0.55, stab_color, 1)

    # 진행바
    bar_w  = w - 24
    filled = int(bar_w * min(count / REQUIRED_SAMPLES, 1.0))
    cv2.rectangle(frame, (12, 66), (12 + bar_w, 80), (50, 50, 50), -1)
    cv2.rectangle(frame, (12, 66), (12 + filled, 80), (0, 188, 212), -1)
    cv2.putText(frame, f"{count}/{REQUIRED_SAMPLES}", (w - 100, 79),
                cv2.FONT_HERSHEY_SIMPLEX, 0.44, (180, 180, 180), 1)

    # 수치
    if vecs:
        cv2.putText(frame, "[R]=재측정  [Q]=종료",
                    (12, 100), cv2.FONT_HERSHEY_SIMPLEX, 0.42, (100, 100, 100), 1)
        y = 124
        cv2.putText(frame, "         bend_z     side_x",
                    (12, y), cv2.FONT_HERSHEY_SIMPLEX, 0.44, (130, 130, 130), 1)
        y += 20
        for name, (bz, sx) in zip(FINGER_NAMES, vecs):
            col = (200, 230, 255) if is_stable else (100, 100, 100)
            cv2.putText(frame, f"{name:<8}  {bz:+.4f}   {sx:+.4f}",
                        (12, y), cv2.FONT_HERSHEY_SIMPLEX, 0.5, col, 1)
            y += 22


def main():
    print("=" * 50)
    print("  🖐  AH Bot 자동 캘리브레이션")
    print("=" * 50)
    print("  1단계: 손 완전히 펴기 → 정지하면 자동 측정")
    print("  2단계: 주먹 완전히 쥐기 → 정지하면 자동 측정")
    print("  각 자세 200샘플 모이면 자동으로 다음 단계 진행")
    print("  [R] 현재 단계 재측정  [Q] 종료")
    print()

    cap = cv2.VideoCapture(0)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 800)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 600)

    stability = StabilityDetector()
    samples = {
        1: {'bend': [[] for _ in range(4)], 'side': [[] for _ in range(4)]},
        2: {'bend': [[] for _ in range(4)], 'side': [[] for _ in range(4)]},
    }
    stage = 1

    with mp_hands.Hands(
        max_num_hands=1,
        min_detection_confidence=0.7,
        min_tracking_confidence=0.5,
        model_complexity=0
    ) as hands:

        while cap.isOpened():
            ret, frame = cap.read()
            if not ret:
                break

            frame = cv2.flip(frame, 1)
            rgb   = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
            rgb.flags.writeable = False
            results = hands.process(rgb)
            rgb.flags.writeable = True

            vecs      = None
            is_stable = False

            if (results.multi_hand_landmarks
                    and results.multi_hand_world_landmarks):

                hand_lm       = results.multi_hand_landmarks[0]
                hand_world_lm = results.multi_hand_world_landmarks[0]

                mp_drawing.draw_landmarks(
                    frame, hand_lm, mp_hands.HAND_CONNECTIONS,
                    mp_drawing_styles.get_default_hand_landmarks_style(),
                    mp_drawing_styles.get_default_hand_connections_style())

                R         = build_palm_frame(hand_world_lm)
                vecs      = get_finger_vectors(hand_world_lm, R)
                stability.update(vecs)
                is_stable = stability.is_stable()

                if is_stable and stage in (1, 2):
                    for i in range(4):
                        samples[stage]['bend'][i].append(vecs[i][0])
                        samples[stage]['side'][i].append(vecs[i][1])

                    count = len(samples[stage]['bend'][0])
                    if count >= REQUIRED_SAMPLES:
                        if stage == 1:
                            print(f"✅ 1단계 완료 ({count}샘플) — 이제 주먹을 쥐세요!")
                            stage = 2
                            stability.reset()
                        else:
                            print(f"✅ 2단계 완료 ({count}샘플)")
                            stage = 3
                            save_to_config(
                                samples[1]['bend'], samples[1]['side'],
                                samples[2]['bend'], samples[2]['side']
                            )
                            draw_ui(frame, 3, vecs, True, REQUIRED_SAMPLES)
                            cv2.imshow("Hand Calibration", frame)
                            cv2.waitKey(2500)
                            break

            count = len(samples[stage]['bend'][0]) if stage in (1, 2) else REQUIRED_SAMPLES
            draw_ui(frame, stage, vecs, is_stable, count)
            cv2.imshow("Hand Calibration", frame)

            key = cv2.waitKey(1) & 0xFF
            if key == ord('q'):
                print("⚠️  중단. 저장 안 됨.")
                break
            elif key in (ord('r'), ord('R')):
                if stage in (1, 2):
                    samples[stage] = {
                        'bend': [[] for _ in range(4)],
                        'side': [[] for _ in range(4)]
                    }
                    stability.reset()
                    print(f"🔄 {stage}단계 초기화. 다시 측정하세요.")

    cap.release()
    cv2.destroyAllWindows()


if __name__ == '__main__':
    main()