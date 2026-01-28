# This script contains the code that has to run on the Raspberry Pi 5, while the final_all_in_one_script.py should
# run on an external computer/laptop. This code starts the cameras and sends a frame to the laptop for processing
# upon pressing Enter.

import cv2
import zmq
import json
import time

# USER CONFIG
LAPTOP_IP = "10.104.24.152"   # change this to your laptop's IP

SYNC_PORT = 5556
IMAGE_PORT = 5555
COORD_PORT = 5557

CAM_LEFT = 0 # change these to your camera id's
CAM_RIGHT = 2

CAPTURE_FRAMES = 1
JPEG_QUALITY = 90

# zeromq setup
ctx = zmq.Context()

request_id = int(time.time() * 1000)

# handshake
sync = ctx.socket(zmq.REQ)
sync.connect(f"tcp://{LAPTOP_IP}:{SYNC_PORT}")

# image sender
push = ctx.socket(zmq.PUSH)
push.setsockopt(zmq.LINGER, 5000)
push.connect(f"tcp://{LAPTOP_IP}:{IMAGE_PORT}")

# coordinate receiver
coord_pull = ctx.socket(zmq.PULL)
coord_pull.bind(f"tcp://*:{COORD_PORT}")

print("🔗 Connecting to laptop...")
sync.send(b"HELLO")
sync.recv()
print("✅ Laptop ready")

# camera setup
capL = cv2.VideoCapture(CAM_LEFT)
capR = cv2.VideoCapture(CAM_RIGHT)

for cap in (capL, capR):
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1920)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 1080)

print("🎥 Stereo cameras running")
print("👉 Press ENTER to capture 1 frame | q to quit")


def main():

    # MAIN LOOP
    while True:
        retL, frameL = capL.read()
        retR, frameR = capR.read()

        if not retL or not retR:
            print("❌ Camera read failed")
            break

        # small and fast preview
        preview = cv2.hconcat([
            cv2.resize(frameL, (640, 360)),
            cv2.resize(frameR, (640, 360))
        ])
        cv2.imshow("Stereo Preview", preview)

        key = cv2.waitKey(1)

        if key == ord("q"):
            break

        # ENTER → CAPTURE + SEND
        if key == 13:
            print("\n📸 Capturing frames...")

            for i in range(CAPTURE_FRAMES):
                retL, frameL = capL.read()
                retR, frameR = capR.read()

                if not retL or not retR:
                    continue

                _, left_png = cv2.imencode(".png", frameL)
                _, right_png = cv2.imencode(".png", frameR)

                meta = json.dumps({
                    "request_id": request_id,
                    "pair_id": i,
                    "left_name": f"left_{i:03d}.png",
                    "right_name": f"right_{i:03d}.png"
                }).encode()

                push.send_multipart([meta, left_png.tobytes(), right_png.tobytes()])

                print(f"📤 Sent pair {i}")
                time.sleep(0.05)

            print("⏳ Waiting for coordinates from laptop...")

            # receive coordinates
            while True:
                coord_data = coord_pull.recv_json()

                if coord_data.get("status") == "no_detection":
                    print("⚠️ No strawberry detected — press ENTER to try again\n")
                    break    
                if coord_data.get("request_id") == request_id:
                    break
                else:
                    print("⚠️ Discarded stale coordinate packet")

            print("✅ Coordinates received:")
            print(coord_data)

            yield coord_data

            print("🚀 Handed off to robot pipeline\n")


if __name__ == "__main__":
    main()

    #  cleanup
    capL.release()
    capR.release()
    cv2.destroyAllWindows()
    push.close()
    sync.close()
    coord_pull.close()
    ctx.term()

    print("🛑 Pi sender stopped")
