# mp_5finger_serial.py
# MediaPipe -> compute angles -> send Thumb,Index,Middle,Ring,Pinky via serial (CSV)

import cv2
import mediapipe as mp
import math
import time
import serial

# -------- CONFIG --------
CAMERA_SOURCE = 0                       # 0 or IP stream URL
SERIAL_PORT = "/dev/cu.usbserial-A5069RR4"  # CHANGE THIS to your port
BAUD = 9600
SEND_INTERVAL = 0.06   # seconds between serial writes (~16Hz)
SMOOTH_ALPHA = 0.8     # higher -> faster snaps (tweak if needed)

# servo ranges (open_angle, closed_angle)
RANGES = {
    "Thumb": (0.0, 180.0),
    "Index": (0.0, 180.0),
    "Middle": (0.0, 180.0),
    "Ring": (13.0, 180.0),   # ring special
    "Pinky": (0.0, 180.0),
}
FINGER_ORDER = ["Thumb", "Index", "Middle", "Ring", "Pinky"]

# geometry thresholds per finger (measured-angle used by MediaPipe)
GEO_RANGES = {
    "Thumb": (8.0, 40.0),   # tuned to be sensitive; adjust if too twitchy
    "Index": (6.0, 95.0),
    "Middle": (6.0, 100.0),
    "Ring": (6.0, 95.0),
    "Pinky": (6.0, 95.0),
}

# --------- SERIAL SETUP ----------
ser = None
try:
    ser = serial.Serial(SERIAL_PORT, BAUD, timeout=0.1)
    time.sleep(2.0)  # let Arduino reset
    # flush initial
    t0 = time.time()
    while time.time() - t0 < 0.5 and ser.in_waiting:
        _ = ser.readline()
    print("Serial connected:", SERIAL_PORT)
except Exception as e:
    print("Serial connect failed:", e)
    ser = None

# -------- math helpers ----------
def vec(a,b): return (b[0]-a[0], b[1]-a[1], b[2]-a[2])
def dot(u,v): return u[0]*v[0] + u[1]*v[1] + u[2]*v[2]
def norm(v): return math.sqrt(v[0]*v[0] + v[1]*v[1] + v[2]*v[2]) + 1e-9
def angle_between(u,v):
    c = dot(u,v) / (norm(u)*norm(v))
    c = max(-1.0, min(1.0, c))
    return math.degrees(math.acos(c))

def finger_curl(lm, mcp, pip, tip):
    return angle_between(vec(lm[mcp], lm[pip]), vec(lm[pip], lm[tip]))

def thumb_curl(lm):
    return angle_between(vec(lm[1], lm[2]), vec(lm[2], lm[4]))

def curl_to_norm(meas_ang, straight_deg, curled_deg):
    if meas_ang <= straight_deg:
        return 0.0
    if meas_ang >= curled_deg:
        return 1.0
    return (meas_ang - straight_deg) / (curled_deg - straight_deg)

def norm_to_servo(norm_val, servo_range):
    open_a, closed_a = servo_range
    return open_a + norm_val * (closed_a - open_a)

# -------- mediapipe ----------
mp_hands = mp.solutions.hands
mp_draw = mp.solutions.drawing_utils

def build_csv_send(angles_dict):
    # order: Thumb,Index,Middle,Ring,Pinky
    vals = [int(round(angles_dict[f])) for f in FINGER_ORDER]
    return f"{vals[0]},{vals[1]},{vals[2]},{vals[3]},{vals[4]}\n"

def main():
    cap = cv2.VideoCapture(CAMERA_SOURCE)
    if not cap.isOpened():
        print("ERROR: camera open failed")
        return

    smoothed = {f: (RANGES[f][0] + RANGES[f][1]) / 2.0 for f in FINGER_ORDER}
    last_send = 0.0

    with mp_hands.Hands(static_image_mode=False, max_num_hands=1,
                        min_detection_confidence=0.6, min_tracking_confidence=0.5) as hands:
        while True:
            ret, frame = cap.read()
            if not ret:
                break

            frame_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
            res = hands.process(frame_rgb)
            h, w = frame.shape[:2]
            display = {f: None for f in FINGER_ORDER}

            if res.multi_hand_landmarks:
                hand = res.multi_hand_landmarks[0]
                lm = [(p.x, p.y, p.z) for p in hand.landmark]

                # measured geometry angles
                meas = {
                    "Thumb": thumb_curl(lm),
                    "Index": finger_curl(lm, 5,6,8),
                    "Middle": finger_curl(lm, 9,10,12),
                    "Ring": finger_curl(lm, 13,14,16),
                    "Pinky": finger_curl(lm, 17,18,20),
                }

                # normalized curls and servo angles
                for f in FINGER_ORDER:
                    straight_deg, curled_deg = GEO_RANGES[f]
                    norm = curl_to_norm(meas[f], straight_deg, curled_deg)
                    tgt = norm_to_servo(norm, RANGES[f])
                    # smoothing (fast)
                    smoothed[f] = SMOOTH_ALPHA * tgt + (1.0 - SMOOTH_ALPHA) * smoothed[f]
                    display[f] = smoothed[f]

                mp_draw.draw_landmarks(frame, hand, mp_hands.HAND_CONNECTIONS)

                # send CSV of all five at limited rate
                now = time.time()
                if ser and (now - last_send) >= SEND_INTERVAL:
                    try:
                        csv = build_csv_send(display)
                        ser.write(csv.encode())
                        last_send = now
                    except Exception:
                        pass

            # draw panel top-right
            lines = []
            for f in FINGER_ORDER:
                if display[f] is None:
                    lines.append(f"{f:6}: --")
                else:
                    lines.append(f"{f:6}: {display[f]:6.1f}°")
            margin = 12
            line_h = 22
            block_w = 260
            block_h = line_h * len(lines) + margin
            x0 = w - block_w - margin
            y0 = margin
            overlay = frame.copy()
            cv2.rectangle(overlay, (x0-6,y0-6), (x0+block_w, y0+block_h), (16,16,16), -1)
            cv2.addWeighted(overlay, 0.6, frame, 0.4, 0, frame)
            yy = y0 + 20
            for ln in lines:
                cv2.putText(frame, ln, (x0, yy), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255,255,255), 2)
                yy += line_h

            cv2.putText(frame, "q: quit", (10, h-10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (200,200,200), 1)
            cv2.imshow("Hand -> 5 Servo Serial", frame)

            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

    cap.release()
    cv2.destroyAllWindows()
    if ser:
        ser.close()

if __name__ == "__main__":
    main()
