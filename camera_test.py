# #!/usr/bin/env python3
# """
# usb_cam_preview.py

# Usage:
#     python usb_cam_preview.py            # try default camera (0)
#     python usb_cam_preview.py --index 1  # open camera index 1
#     python usb_cam_preview.py --width 1280 --height 720
# """

# import cv2
# import argparse
# import time
# import os

# def parse_args():
#     p = argparse.ArgumentParser(description="Open USB camera and preview with OpenCV")
#     p.add_argument("--index", type=int, default=0, help="Camera index (usually 0 for first webcam)")
#     p.add_argument("--width", type=int, default=640, help="Desired capture width")
#     p.add_argument("--height", type=int, default=480, help="Desired capture height")
#     p.add_argument("--save-dir", default="captures", help="Directory to save frames when pressing 's'")
#     return p.parse_args()

# def main():
#     args = parse_args()
#     cap = cv2.VideoCapture(args.index, cv2.CAP_ANY)  # CV_CAP_ANY lets OpenCV choose backend

#     if not cap.isOpened():
#         print(f"[ERROR] Cannot open camera index {args.index}.")
#         print(" - Check that the camera is connected, not used by another app, and index is correct.")
#         return

#     # Try to set resolution (may be ignored by some cameras/drivers)
#     cap.set(cv2.CAP_PROP_FRAME_WIDTH, args.width)
#     cap.set(cv2.CAP_PROP_FRAME_HEIGHT, args.height)

#     os.makedirs(args.save_dir, exist_ok=True)
#     print("[INFO] Press 'q' to quit, 's' to save current frame.")

#     prev_time = time.time()
#     frame_count = 0

#     while True:
#         ret, frame = cap.read()
#         if not ret:
#             print("[WARNING] Frame not received. Reinitializing capture...")
#             cap.release()
#             time.sleep(0.5)
#             cap = cv2.VideoCapture(args.index, cv2.CAP_ANY)
#             continue

#         frame_count += 1
#         # Compute a simple FPS every 30 frames
#         if frame_count % 30 == 0:
#             now = time.time()
#             fps = 30.0 / (now - prev_time)
#             prev_time = now
#             # draw FPS on frame
#             cv2.putText(frame, f"FPS: {fps:.1f}", (10, 30),
#                         cv2.FONT_HERSHEY_SIMPLEX, 0.9, (0, 255, 0), 2)

#         cv2.imshow("USB Camera Preview", frame)

#         key = cv2.waitKey(1) & 0xFF
#         if key == ord('q'):
#             break
#         if key == ord('s'):
#             # Save frame with timestamp
#             ts = time.strftime("%Y%m%d-%H%M%S")
#             fname = os.path.join(args.save_dir, f"frame_{ts}.jpg")
#             cv2.imwrite(fname, frame)
#             print(f"[INFO] Saved {fname}")

#     cap.release()
#     cv2.destroyAllWindows()
#     print("[INFO] Exited cleanly.")

# if __name__ == "__main__":
#     main()







#!/usr/bin/env python3
"""
usb_cam_simple.py

Simple USB camera preview and optional timed recording.

Usage:
  # show live preview (no recording)
  python usb_cam_simple.py

  # preview and record for 5 minutes
  python usb_cam_simple.py --save --minutes 5

  # choose a different camera index (0, 1, ...)
  python usb_cam_simple.py --index 1 --save --minutes 2
"""

import cv2
import argparse
import time
import os

def parse_args():
    p = argparse.ArgumentParser(description="Simple USB camera preview + optional timed record")
    p.add_argument("--index", "-i", type=int, default=1, help="Camera index (default 0)")
    p.add_argument("--save", "-s", action="store_true", help="Save video to file")
    p.add_argument("--minutes", "-m", type=float, default=1.0,
                   help="Recording duration in minutes when --save is used (default 1.0)")
    p.add_argument("--out", "-o", default=None, help="Output filename (optional). If omitted a timestamped file is used.")
    return p.parse_args()

def main():
    args = parse_args()
    cap = cv2.VideoCapture(args.index)

    if not cap.isOpened():
        print(f"[ERROR] Cannot open camera {args.index}")
        return

    # try to get properties; fallback values if unavailable
    width  = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH) or 640)
    height = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT) or 480)
    fps    = cap.get(cv2.CAP_PROP_FPS) or 20.0  # some cameras don't report fps

    out = None
    if args.save:
        if args.out:
            out_fname = args.out
        else:
            ts = time.strftime("%Y%m%d-%H%M%S")
            out_fname = f"usb_capture_{ts}.avi"

        # FourCC: try mp4v or XVID; .avi container here. Change extension if desired.
        fourcc = cv2.VideoWriter_fourcc(*"XVID")
        out = cv2.VideoWriter(out_fname, fourcc, fps, (width, height))
        if not out.isOpened():
            print(f"[ERROR] Cannot open video writer for file {out_fname}")
            args.save = False
        else:
            print(f"[INFO] Recording to {out_fname} for {args.minutes} minute(s) (FPS={fps}, {width}x{height})")

    print("[INFO] Press 'q' to quit early. Press 's' to save a single frame as image.")

    start_time = time.time()
    record_seconds = args.minutes * 60.0

    while True:
        ret, frame = cap.read()
        print(frame)
        if not ret:
            print("[WARNING] Failed to read frame from camera. Exiting.")
            break

        # cv2.imshow("USB Camera (press q to quit)", frame)

        # write frame if recording
        if args.save:
            out.write(frame)
            # stop automatically after desired duration
            if (time.time() - start_time) >= record_seconds:
                print("[INFO] Reached requested recording duration. Stopping recording.")
                break

        key = cv2.waitKey(1) & 0xFF
        if key == ord('q'):
            print("[INFO] Quit requested by user.")
            break
        if key == ord('s'):
            # save a single jpg snapshot
            fname = f"snapshot_{int(time.time())}.jpg"
            cv2.imwrite(fname, frame)
            print(f"[INFO] Saved snapshot: {fname}")

    cap.release()
    if out:
        out.release()
    cv2.destroyAllWindows()
    print("[INFO] Exited cleanly.")

if __name__ == "__main__":
    main()
