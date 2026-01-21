# zmq_rtsp_client.py
import zmq
import cv2
import time
import uuid
import sys
import os
import numpy as np
from time import sleep

# add parent folder so you can import SIYISDK
current = os.path.dirname(os.path.realpath(__file__))
parent_directory = os.path.dirname(current)
sys.path.append(parent_directory)
from siyi_sdk import SIYISDK

# ZMQ server address (match server). If server on another machine: tcp://192.168.x.y:5555
ZMQ_ADDR = "ipc:///tmp/zmq_infer.sock"   # or "tcp://<server_ip>:5555"

# tune these
JPEG_QUALITY = 80
SEND_EVERY_N_FRAMES = 1

class SIYIRC:
    def __init__(self, server_ip="192.168.144.25"):
        self.server_ip = server_ip
        self.cam = SIYISDK(server_ip=server_ip)

    def setup_cam_sdk(self):
        if not self.cam.connect():
            print("Cannot connect to SIYI camera")
            return False
        self.cam.requestFollowMode()
        sleep(0.5)
        self.cam.requestSetAngles(0.0, -90.0)
        sleep(0.2)
        self.cam.disconnect()
        return True

    def open_rtsp(self):
        gst_pipeline = (
            f"rtspsrc location=rtsp://{self.server_ip}:8554/main.264 protocols=tcp latency=10 ! "
            "rtph265depay ! h265parse ! nvv4l2decoder ! "
            "nvvidconv ! video/x-raw, format=BGRx ! "
            "videoconvert ! video/x-raw, format=BGR ! "
            "appsink drop=true sync=false"
        )
        cap = cv2.VideoCapture(gst_pipeline, cv2.CAP_GSTREAMER)
        if not cap.isOpened():
            print("Failed to open RTSP")
            return None
        return cap

    def run(self):
        ctx = zmq.Context()
        sock = ctx.socket(zmq.REQ)
        sock.connect(ZMQ_ADDR)
        print("Connected to inference server at", ZMQ_ADDR)

        cap = self.open_rtsp()
        if cap is None:
            return

        frame_idx = 0
        while True:
            ret, frame = cap.read()
            if not ret:
                print("Empty frame, retrying...")
                time.sleep(0.05)
                continue

            frame_idx += 1
            if frame_idx % SEND_EVERY_N_FRAMES != 0:
                cv2.imshow("RTSP (no detect)", frame)
                if cv2.waitKey(1) & 0xFF == ord('q'):
                    break
                continue

            # encode JPEG
            encode_param = [int(cv2.IMWRITE_JPEG_QUALITY), JPEG_QUALITY]
            ok, jpg = cv2.imencode('.jpg', frame, encode_param)
            if not ok:
                continue
            frame_id = str(uuid.uuid4())
            # send [frame_id, jpg bytes]
            try:
                sock.send_multipart([frame_id.encode('utf-8'), jpg.tobytes()])
                # wait for reply (this blocks until inference server replies)
                reply = sock.recv_json()
            except Exception as e:
                print("ZMQ error:", e)
                time.sleep(0.1)
                continue

            # match frame_id (optional)
            # draw detections if present
            dets = reply.get("detections", [])
            for det in dets:
                try:
                    x1, y1, x2, y2 = map(int, det["bbox"])
                    label = f"{det['cls']} {det['conf']:.2f}"
                    cv2.rectangle(frame, (x1, y1), (x2, y2), (0,255,0), 2)
                    cv2.putText(frame, label, (x1, max(15, y1-5)),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255,255,255), 2)
                except Exception as e:
                    pass

            cv2.imshow("RTSP + Detections", frame)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

        cap.release()
        cv2.destroyAllWindows()
        sock.close()
        ctx.term()

if __name__ == "__main__":
    client = SIYIRC()
    # optionally call client.setup_cam_sdk()
    client.run()
