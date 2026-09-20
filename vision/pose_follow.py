"""
Drone Follow Simulation — MediaPipe Pose (compatible with MediaPipe 0.10+)
==========================================================================
Uses shoulder width as a relative distance proxy.
Simulates what commands would be sent to a drone.

Controls:
  L = Lock on to current person (set reference distance)
  R = Reset lock
  Q = Quit
"""

import cv2
import numpy as np
import mediapipe as mp
from mediapipe.tasks import python
from mediapipe.tasks.python import vision
import urllib.request
import os

# ── Config ───────────────────────────────────────────────────────────────────
CLOSER_THRESHOLD  = 1.15
FARTHER_THRESHOLD = 0.85
LEFT_THRESHOLD    = -0.12
RIGHT_THRESHOLD   =  0.12
SMOOTHING         = 0.3

LEFT_SHOULDER  = 11
RIGHT_SHOULDER = 12

MODEL_PATH = "pose_landmarker_lite.task"
MODEL_URL  = "https://storage.googleapis.com/mediapipe-models/pose_landmarker/pose_landmarker_lite/float16/latest/pose_landmarker_lite.task"


def download_model():
    if not os.path.exists(MODEL_PATH):
        print("[INFO] Downloading MediaPipe pose model (~5MB)...")
        urllib.request.urlretrieve(MODEL_URL, MODEL_PATH)
        print("[INFO] Model downloaded.")


def get_shoulder_metrics(landmarks, frame_w, frame_h):
    l = landmarks[LEFT_SHOULDER]
    r = landmarks[RIGHT_SHOULDER]
    lx, ly = int(l.x * frame_w), int(l.y * frame_h)
    rx, ry = int(r.x * frame_w), int(r.y * frame_h)
    width  = np.hypot(rx - lx, ry - ly)
    x_off  = ((l.x + r.x) / 2) - 0.5
    return width, x_off, (lx, ly), (rx, ry)


def decide_commands(ratio, x_offset):
    if ratio < FARTHER_THRESHOLD:
        depth_cmd, depth_col = "^ MOVE FORWARD",  (0, 200, 255)
    elif ratio > CLOSER_THRESHOLD:
        depth_cmd, depth_col = "v MOVE BACKWARD", (0, 100, 255)
    else:
        depth_cmd, depth_col = "* HOLD DISTANCE", (0, 220, 0)

    if x_offset < LEFT_THRESHOLD:
        yaw_cmd, yaw_col = "< YAW LEFT",  (255, 180, 0)
    elif x_offset > RIGHT_THRESHOLD:
        yaw_cmd, yaw_col = "> YAW RIGHT", (255, 180, 0)
    else:
        yaw_cmd, yaw_col = "* CENTRED",   (0, 220, 0)

    return depth_cmd, yaw_cmd, depth_col, yaw_col


def draw_landmarks(frame, landmarks, frame_w, frame_h):
    connections = [
        (11,12),(11,13),(13,15),(12,14),(14,16),
        (11,23),(12,24),(23,24),
        (23,25),(25,27),(24,26),(26,28),
    ]
    pts = {}
    for i, lm in enumerate(landmarks):
        pts[i] = (int(lm.x * frame_w), int(lm.y * frame_h))
        cv2.circle(frame, pts[i], 3, (0, 255, 0), -1)
    for a, b in connections:
        if a in pts and b in pts:
            cv2.line(frame, pts[a], pts[b], (0, 180, 0), 2)


def draw_hud(frame, locked, ref_width, smooth_width, ratio,
             depth_cmd, yaw_cmd, depth_col, yaw_col, shoulder_l, shoulder_r):
    h, w = frame.shape[:2]

    cv2.line(frame, shoulder_l, shoulder_r, (255, 255, 0), 2)
    cv2.circle(frame, shoulder_l, 6, (255, 255, 0), -1)
    cv2.circle(frame, shoulder_r, 6, (255, 255, 0), -1)
    mid = ((shoulder_l[0]+shoulder_r[0])//2, (shoulder_l[1]+shoulder_r[1])//2)
    cv2.putText(frame, f"{smooth_width:.0f}px", mid,
                cv2.FONT_HERSHEY_SIMPLEX, 0.55, (255,255,0), 2)

    cv2.line(frame, (w//2, 0), (w//2, h), (80, 80, 80), 1)

    if locked:
        lines = [
            (f"REF WIDTH : {ref_width:.0f}px", (200,200,200)),
            (f"CUR WIDTH : {smooth_width:.0f}px", (200,200,200)),
            (f"RATIO     : {ratio:.2f}x", (200,200,200)),
            ("", None),
            (depth_cmd, depth_col),
            (yaw_cmd,   yaw_col),
        ]
    else:
        lines = [("NO LOCK -- press L to lock", (0,200,255))]

    for i, (text, colour) in enumerate(lines):
        if text:
            cv2.putText(frame, text, (12, 30+i*28),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.65, colour, 2)

    lock_text = "[ LOCKED ]" if locked else "[ UNLOCKED ]"
    lock_col  = (0,255,100) if locked else (0,100,255)
    cv2.putText(frame, lock_text, (w-160, 30),
                cv2.FONT_HERSHEY_SIMPLEX, 0.65, lock_col, 2)
    cv2.putText(frame, "L=Lock  R=Reset  Q=Quit", (12, h-12),
                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (160,160,160), 1)


def main():
    download_model()

    cap = cv2.VideoCapture(0)
    if not cap.isOpened():
        raise RuntimeError("Cannot open camera.")

    ref_width    = None
    smooth_width = None
    locked       = False

    base_options = python.BaseOptions(model_asset_path=MODEL_PATH)
    options      = vision.PoseLandmarkerOptions(
                     base_options=base_options,
                     running_mode=vision.RunningMode.IMAGE,
                     num_poses=1,
                     min_pose_detection_confidence=0.6,
                     min_tracking_confidence=0.6)
    landmarker   = vision.PoseLandmarker.create_from_options(options)

    print("[INFO] Running -- press L in the window to lock onto yourself.")

    while True:
        ret, frame = cap.read()
        if not ret:
            break

        frame = cv2.flip(frame, 1)
        h, w  = frame.shape[:2]

        mp_image = mp.Image(image_format=mp.ImageFormat.SRGB,
                            data=cv2.cvtColor(frame, cv2.COLOR_BGR2RGB))
        result   = landmarker.detect(mp_image)

        key = cv2.waitKey(1) & 0xFF
        if key == ord('q'):
            break
        elif key == ord('r'):
            ref_width = smooth_width = None
            locked = False
            print("[INFO] Lock reset.")

        if result.pose_landmarks:
            landmarks = result.pose_landmarks[0]
            draw_landmarks(frame, landmarks, w, h)

            sw, x_off, sl, sr = get_shoulder_metrics(landmarks, w, h)
            smooth_width = (SMOOTHING * sw + (1 - SMOOTHING) * smooth_width
                            if smooth_width else sw)

            if key == ord('l'):
                ref_width = smooth_width
                locked    = True
                print(f"[INFO] Locked -- reference width: {ref_width:.0f}px")

            if locked and ref_width:
                ratio = smooth_width / ref_width
                depth_cmd, yaw_cmd, dc, yc = decide_commands(ratio, x_off)
            else:
                ratio = 1.0
                depth_cmd = yaw_cmd = ""
                dc = yc = (200, 200, 200)

            draw_hud(frame, locked, ref_width or 0, smooth_width, ratio,
                     depth_cmd, yaw_cmd, dc, yc, sl, sr)
        else:
            cv2.putText(frame, "No person detected", (12, 30),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0,0,255), 2)

        cv2.imshow("Drone Follow -- MediaPipe", frame)

    cap.release()
    cv2.destroyAllWindows()
    landmarker.close()


if __name__ == "__main__":
    main()