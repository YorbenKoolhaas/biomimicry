# This script should be run on a laptop, in combination with pi_script.py which has to be run on a Raspberry Pi 5
# This code contains the connection with the Raspberry Pi 5, performs the object detection + depth estimation +
# conversion to robot base frame coordinates on the images received from the Pi, and finally sends the data back to 
# the Pi

import cv2
import numpy as np
from ultralytics import YOLO
import os
import csv
import zmq
import json
import queue


# USER CONFIG

CALIB_PATH = "stereo_charuco_calibration5.npz"
MODEL_PATH = "runs/detect/train20/weights/best.pt"
LEFT_INPUT_DIR = "received/left"
RIGHT_INPUT_DIR = "received/right"

OUTPUT_DIR = "output_logs"
os.makedirs(OUTPUT_DIR, exist_ok=True)

CONFIDENCE_THRESHOLD = 0.6

# performance
SCALE_FOR_MATCHING = 0.5

# depth constraints
EXPECTED_DISTANCE = 0.20  # meters
MIN_DEPTH = 0.10
MAX_DEPTH = 2.0

CAPTURE_FRAMES = 1
batch_queue = queue.Queue()

# zeromq config (LAPTOP → PI)
PI_IP = "10.104.24.67"   # CHANGE to Pi IP
COORD_PORT = 5557

# load calibration
print("[INFO] Loading calibration...")
calib = np.load(CALIB_PATH)

K_L = calib["K_left"]
D_L = calib["D_left"]
K_R = calib["K_right"]
D_R = calib["D_right"]
R = calib["R"]
T = calib["T"]

baseline = np.linalg.norm(T)
print(f"[INFO] Stereo baseline: {baseline:.4f} m")

# CAMERA → ROBOT BASE TRANSFORM (FILL WITH REAL VALUES)
R_BASE_CAMERA = np.array([
    [ 1,  0,  0],
    [ 0,  1,  0],
    [ 0,  0,  1]
], dtype=np.float64)
# TODO: replace with correct rotation matrix

# Translation: Camera origin in robot base frame (meters)
t_BASE_CAMERA = np.array([
    [0.0],  # x offset
    [0.0],  # y offset
    [0.0]   # z offset
], dtype=np.float64)
# TODO: measure camera position relative to robot base

TCP_OFFSET = np.array([
    [0.0],   # x
    [0.0],   # y
    [0.12]   # z (example: 12 cm gripper length)
], dtype=np.float64)
# TODO: measure real gripper geometry

# depth bias model
MIN_COVERAGE_PCT = 15.0
MAX_VALID_DEPTH = 0.55  # meters

DEPTH_BIAS_A = 0.0025
DEPTH_BIAS_B = -0.02
DEPTH_BIAS_C = 1.1

def apply_depth_bias(Z_cm):
    bias = DEPTH_BIAS_A * Z_cm**2 + DEPTH_BIAS_B * Z_cm + DEPTH_BIAS_C
    return Z_cm - bias

# helper functions
def scale_intrinsics(K, scale):
    K2 = K.copy()
    K2[0, 0] *= scale
    K2[1, 1] *= scale
    K2[0, 2] *= scale
    K2[1, 2] *= scale
    return K2

def setup_stereo_rectification(w, h):
    K_L_s = scale_intrinsics(K_L, SCALE_FOR_MATCHING)
    K_R_s = scale_intrinsics(K_R, SCALE_FOR_MATCHING)

    R1, R2, P1, P2, Q, _, _ = cv2.stereoRectify(
        K_L_s, D_L, K_R_s, D_R, (w, h), R, T, alpha=1
    )

    mapLx, mapLy = cv2.initUndistortRectifyMap(
        K_L_s, D_L, R1, P1, (w, h), cv2.CV_32FC1
    )
    mapRx, mapRy = cv2.initUndistortRectifyMap(
        K_R_s, D_R, R2, P2, (w, h), cv2.CV_32FC1
    )

    return mapLx, mapLy, mapRx, mapRy, Q, K_L_s

def compute_stereo_disparity(rectL, rectR, K_L_s):
    grayL = cv2.cvtColor(rectL, cv2.COLOR_BGR2GRAY)
    grayR = cv2.cvtColor(rectR, cv2.COLOR_BGR2GRAY)

    clahe = cv2.createCLAHE(3.0, (8, 8))
    grayL = clahe.apply(grayL)
    grayR = clahe.apply(grayR)

    # UNCOMMENT THIS TO EXPERIMENT WITH VARIABLE num_disp AND COMMENT THE num_disp = 320
    # expected_disp = (K_L_s[0, 0] * baseline) / EXPECTED_DISTANCE
    # num_disp = int(np.ceil(expected_disp * 1.8 / 16) * 16)
    # num_disp = max(160, min(num_disp, 640))

    num_disp = 320

    stereo = cv2.StereoSGBM.create(
        minDisparity=0,
        numDisparities=num_disp,
        blockSize=5,
        P1=8 * 5**2,
        P2=32 * 5**2,
        uniquenessRatio=6,
        speckleWindowSize=80,
        speckleRange=32,
        mode=cv2.STEREO_SGBM_MODE_SGBM_3WAY
    )

    return stereo.compute(grayL, grayR).astype(np.float32) / 16.0, num_disp

def estimate_roi_depth(depth_map, box):
    x1, y1, x2, y2 = map(int, box)
    h, w = depth_map.shape

    x1, y1 = max(0, x1), max(0, y1)
    x2, y2 = min(w, x2), min(h, y2)

    if x2 <= x1 or y2 <= y1:
        return None

    roi = depth_map[y1:y2, x1:x2]
    valid = roi[(roi > MIN_DEPTH) & (roi < MAX_DEPTH) & np.isfinite(roi)]

    if len(valid) == 0:
        return None

    return float(np.median(valid)), 100.0 * len(valid) / roi.size

# convert pixel + depth → camera-frame 3D point
def pixel_to_camera_xyz(u, v, Z):
    fx = K_L[0, 0]
    fy = K_L[1, 1]
    cx = K_L[0, 2]
    cy = K_L[1, 2]

    X = (u - cx) * Z / fx
    Y = (v - cy) * Z / fy

    return np.array([[X], [Y], [Z]])

# transform point from camera frame to robot base frame
def camera_to_robot_base(X_cam):
    return R_BASE_CAMERA @ X_cam + t_BASE_CAMERA

# pixel detection → robot grasp position
def compute_grasp_pose(u, v, Z, coverage):
    if coverage < MIN_COVERAGE_PCT:
        return None, "Rejected: low depth coverage"

    if Z > MAX_VALID_DEPTH:
        return None, "Rejected: depth too far"

    # bias correction
    Z_corr = apply_depth_bias(Z * 100)
    if Z_corr is None:
        return None, "Rejected: low depth coverage"
    Z_corr = Z_corr / 100.0

    # pixel → camera XYZ
    P_cam = pixel_to_camera_xyz(u, v, Z_corr)

    # camera → robot Base
    P_base = camera_to_robot_base(P_cam)

    # add gripper TCP offset
    P_grasp = P_base - TCP_OFFSET

    return P_grasp, "OK"

# csv logging setup
def create_numbered_csv(log_folder, log_id):
    filename = f"log_{log_id:03d}.csv"
    path = os.path.join(log_folder, filename)

    file = open(path, "w", newline="")
    writer = csv.writer(file)

    writer.writerow([
        "log_id",
        "frame_idx",
        "u_pixel",
        "v_pixel",
        "depth_m",
        "coverage_pct",
        "X_base_m",
        "Y_base_m",
        "Z_base_m"
    ])

    return file, writer

def create_log_folder(base_dir, prefix="log"):
    existing = [f for f in os.listdir(base_dir) if f.startswith(prefix)]
    next_id = len(existing) + 1
    folder_name = f"{prefix}_{next_id:03d}"
    path = os.path.join(base_dir, folder_name)
    os.makedirs(path, exist_ok=True)
    return path, next_id

# zeromq image receiver thread
def receive_stereo_images():
    ctx = zmq.Context()

    sync = ctx.socket(zmq.REP)
    sync.bind("tcp://*:5556")

    pull = ctx.socket(zmq.PULL)
    pull.bind("tcp://*:5555")

    print("⏳ Waiting for Pi...")
    sync.recv()
    sync.send(b"READY")
    print("📡 Ready to receive stereo images")

    current_batch = []

    while True:
        parts = pull.recv_multipart()
        meta = json.loads(parts[0].decode())

        current_request_id = meta["request_id"]

        left_data = parts[1]
        right_data = parts[2]

        left_path = os.path.join(LEFT_INPUT_DIR, meta["left_name"])
        right_path = os.path.join(RIGHT_INPUT_DIR, meta["right_name"])

        with open(left_path, "wb") as f:
            f.write(left_data)
        with open(right_path, "wb") as f:
            f.write(right_data)

        print(f"✅ Received stereo pair {meta['pair_id']}")

        current_batch.append((left_path, right_path))

        if len(current_batch) == CAPTURE_FRAMES:
            batch_queue.put((current_request_id, current_batch.copy()))
            current_batch.clear()
            print("📦 Batch complete → queued for processing")


# ==========================================================
# MAIN
# ==========================================================
def main():
    import threading
    # start receiver thread
    recv_thread = threading.Thread(target=receive_stereo_images, daemon=True)
    recv_thread.start()

    frame_counter = 0
    image_counter = 0
    detections = []

    # load YOLO model
    model = YOLO(MODEL_PATH)

    # ZeroMQ PUSH to send coordinates back
    ctx = zmq.Context()
    coord_push = ctx.socket(zmq.PUSH)
    coord_push.connect(f"tcp://{PI_IP}:{COORD_PORT}")

    # wait for first batch to infer image size
    print("[INFO] Waiting for the first batch...")
    current_request_id, first_batch = batch_queue.get()

    frameL = cv2.imread(first_batch[0][0])
    h, w = frameL.shape[:2]
    w_s, h_s = int(w * SCALE_FOR_MATCHING), int(h * SCALE_FOR_MATCHING)

    # recompute stereo rectification maps
    mapLx, mapLy, mapRx, mapRy, Q, K_L_s = setup_stereo_rectification(w_s, h_s)

    # prepare logging
    log_folder, log_id = create_log_folder(OUTPUT_DIR)
    csv_file, csv_writer = create_numbered_csv(log_folder, log_id)
    print(f"[INFO] Logging to folder: {log_folder}")
    print("[INFO] Ready — waiting for ENTER on Pi")

    # main loop over folder images
    try:
        while True:
            if frame_counter == 0:
                batch = first_batch
            else:
                current_request_id, batch = batch_queue.get()
            frame_counter += 1
            detections.clear()
            batch_success = False

            print(f"\n[INFO] Processing batch #{frame_counter}")

            for left_path, right_path in batch:
                frameL = cv2.imread(left_path)
                frameR = cv2.imread(right_path)

                if frameL is None or frameR is None:
                    print(f"[WARNING] Could not read frames: {left_path}, {right_path}")
                    continue

                # resize and rectify
                frameL_s = cv2.resize(frameL, (w_s, h_s))
                frameR_s = cv2.resize(frameR, (w_s, h_s))

                rectL = cv2.remap(frameL_s, mapLx, mapLy, cv2.INTER_LINEAR)
                rectR = cv2.remap(frameR_s, mapRx, mapRy, cv2.INTER_LINEAR)

                # compute stereo disparity & depth map
                disparity, _ = compute_stereo_disparity(rectL, rectR, K_L_s)
                points_3d = cv2.reprojectImageTo3D(disparity, Q)
                depth_map = points_3d[:, :, 2]
                depth_map[(disparity <= 0) | ~np.isfinite(depth_map)] = 0

                # run YOLO detection
                results = model.predict(frameL, conf=0.35, verbose=False)

                for r in results:
                    if r.boxes is None:
                        continue

                    for box in r.boxes:

                        if float(box.conf[0]) < CONFIDENCE_THRESHOLD:
                            continue

                        x1, y1, x2, y2 = box.xyxy[0].tolist()
                        box_s = (
                            x1 * SCALE_FOR_MATCHING,
                            y1 * SCALE_FOR_MATCHING,
                            x2 * SCALE_FOR_MATCHING,
                            y2 * SCALE_FOR_MATCHING
                        )

                        depth_info = estimate_roi_depth(depth_map, box_s)
                        if depth_info is None:
                            continue

                        Z, coverage = depth_info

                        # collect detections for batch
                        detections.append((x1, y1, x2, y2, Z, coverage))
                        # save frame as png
                        image_path = os.path.join(log_folder, f"frame_{image_counter:03d}.png")
                        cv2.imwrite(image_path, frameL)
                        image_counter += 1

                    if not detections:
                        print("[WARN] No valid detections in batch")

                        coord_push.send_json({
                            "request_id": current_request_id,
                            "status": "no_detection"
                        })

                        print("[INFO] Sent NO_DETECTION response — ready for next Enter")
                        break

                    chosen = max(detections, key=lambda d: d[5])
                    x1, y1, x2, y2, Z, coverage = chosen
                    u, v = (x1 + x2)/2, (y1 + y2)/2

                    Z_corr = apply_depth_bias(Z * 100) / 100.0
                    P_base, status = compute_grasp_pose(u, v, Z_corr, coverage)

                    if P_base is None:
                        print(f"[REJECTED] {status}")
                        break

                    P_base = P_base.flatten()
                    csv_writer.writerow([
                        log_id, frame_counter,
                        round(u,2), round(v,2), round(Z_corr,4), round(coverage,2),
                        round(float(P_base[0]),4),
                        round(float(P_base[1]),4),
                        round(float(P_base[2]),4)
                    ])
                    csv_file.flush()

                    print("\n=== FINAL OUTPUT ===")
                    print(f"Pixel (u,v): ({u:.1f},{v:.1f})")
                    print(f"Depth: {Z_corr:.3f} m")
                    print("Robot base XYZ (m):")
                    print(P_base)

                    # Send coordinates to Pi
                    coord_payload = {
                        "request_id": current_request_id,
                        "u": float(u),
                        "v": float(v),
                        "depth_m": float(Z_corr),
                        "coverage_pct": float(coverage),
                        "X_base_m": float(P_base[0]),
                        "Y_base_m": float(P_base[1]),
                        "Z_base_m": float(P_base[2]),
                    }
                    coord_push.send_json(coord_payload)
                    print(f"[SEND] request_id={current_request_id}")
                    print("[INFO] Coordinates sent — ready for next Enter")
                    batch_success = True
                    break
            if not batch_success:
                print("[INFO] Batch ended without valid grasp — waiting for next ENTER")
    finally:
        coord_push.close()
        ctx.term()
        csv_file.close()
        print("[INFO] Processing complete.")

if __name__ == "__main__":
    main()
