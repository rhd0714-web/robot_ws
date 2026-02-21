import cv2
import mediapipe as mp
import math

# 미디어파이프 설정
mp_hands = mp.solutions.hands
mp_drawing = mp.solutions.drawing_utils

# 랜드마크 인덱스
MP_TIPS = [8, 12, 16, 4]  # 검지, 중지, 약지, 엄지 끝
MP_MCPS = [5, 9, 13, 1]   # 각 손가락 뿌리
MP_PIPS = [6, 10, 14, 2]  # 각 손가락 중간 관절 

def get_distance(p1, p2):
    return math.sqrt((p1.x - p2.x)**2 + (p1.y - p2.y)**2)

# [추가됨] 각도 계산 함수 (Vision Node와 동일 로직)
def get_signed_angle(wrist, mid_mcp, finger_mcp, finger_tip):
    # 벡터 1: 손바닥 중심 축 (손목 -> 중지 뿌리)
    v1_x = mid_mcp.x - wrist.x
    v1_y = mid_mcp.y - wrist.y
    
    # 벡터 2: 손가락 방향 (손가락 뿌리 -> 손가락 끝)
    v2_x = finger_tip.x - finger_mcp.x
    v2_y = finger_tip.y - finger_mcp.y
    
    # 아크탄젠트로 각도 계산
    ang1 = math.atan2(v1_y, v1_x)
    ang2 = math.atan2(v2_y, v2_x)
    
    # 차이값 (Degree 변환)
    diff = math.degrees(ang2 - ang1)
    
    # -180 ~ 180도로 정규화
    if diff > 180: diff -= 360
    elif diff < -180: diff += 360
    return diff

def main():
    cap = cv2.VideoCapture(0)
    # 해상도 설정 (글자 잘 보이게)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 960)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)
    
    print("========================================")
    print("🖐️  FULL HAND CALIBRATION TOOL")
    print("========================================")
    print("[Bend] 주먹 쥐기 vs 손 펴기 (Ratio)")
    print("[Side] 손가락 좌우로 벌리기 (Angle)")
    print("----------------------------------------")

    with mp_hands.Hands(
        max_num_hands=1,
        min_detection_confidence=0.7,
        min_tracking_confidence=0.5
    ) as hands:
        
        while cap.isOpened():
            ret, frame = cap.read()
            if not ret: break

            frame = cv2.flip(frame, 1)
            rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
            results = hands.process(rgb)
            
            if results.multi_hand_landmarks:
                for hand_landmarks in results.multi_hand_landmarks:
                    mp_drawing.draw_landmarks(frame, hand_landmarks, mp_hands.HAND_CONNECTIONS)
                    
                    wrist = hand_landmarks.landmark[0]
                    mid_mcp = hand_landmarks.landmark[9] # 중지 뿌리 (기준점)
                    
                    palm_size = get_distance(wrist, mid_mcp)
                    
                    finger_names = ["Index ", "Middle", "Ring  ", "Thumb "]
                    
                    # 헤더 출력
                    cv2.putText(frame, "FINGER |  BEND(Ratio)  |  SIDE(Angle)", (10, 30), 
                                cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 0), 2)

                    text_y = 70
                    
                    for i in range(4):
                        tip = hand_landmarks.landmark[MP_TIPS[i]]
                        mcp = hand_landmarks.landmark[MP_MCPS[i]]
                        pip = hand_landmarks.landmark[MP_PIPS[i]]
                        pinky_mcp = hand_landmarks.landmark[17]

                        # ---------------------------
                        # 1. Bend (Ratio) 계산
                        # ---------------------------
                        if i == 3: # 엄지
                            curr_dist = get_distance(tip, pinky_mcp)
                            curr_palm = get_distance(wrist, hand_landmarks.landmark[5])
                        else:
                            curr_dist = get_distance(tip, wrist)
                            curr_palm = palm_size
                        
                        if curr_palm == 0: ratio = 0
                        else: ratio = curr_dist / curr_palm

                        # ---------------------------
                        # 2. Side (Angle) 계산
                        # ---------------------------
                        if i == 3: # 엄지
                            # 엄지는 (손목->검지뿌리) 축과 (손목->엄지끝)의 각도
                            angle = abs(get_signed_angle(wrist, hand_landmarks.landmark[5], wrist, pip))
                        else:
                            # 나머지는 (손목->중지뿌리) 축과 (뿌리->끝)의 각도
                            angle = get_signed_angle(wrist, mid_mcp, mcp, pip)
                        
                        # ---------------------------
                        # 화면 출력
                        # ---------------------------
                        # Bend 색상 (초록: 펴짐 / 빨강: 굽힘)
                        color_bend = (0, 255, 0) if ratio > 1.2 else (0, 0, 255)
                        
                        # Side 색상 (파랑: 평범 / 보라: 많이 벌어짐)
                        color_side = (255, 255, 0)
                        if abs(angle) > 15: color_side = (255, 0, 255)

                        # 텍스트 포맷팅
                        # 예: Index :  1.532   |   -5.4 deg
                        info_text = f"{finger_names[i]} : {ratio:.3f}   |  {angle:.1f} deg"
                        
                        cv2.putText(frame, info_text, (10, text_y), 
                                    cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 255, 255), 2)
                        
                        # 수치 옆에 색상 표시용 원 그리기
                        cv2.circle(frame, (280, text_y-10), 8, color_bend, -1)
                        cv2.circle(frame, (450, text_y-10), 8, color_side, -1)
                        
                        text_y += 50

            cv2.imshow("Full Hand Calibration", frame)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
    
    cap.release()
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()
