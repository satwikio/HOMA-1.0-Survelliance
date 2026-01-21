import cv2
from siyi_cam2 import SIYICam

def main():
    cam = SIYICam()
    cam.open_stream()

    for frame in cam.frames():
        cv2.imshow("Viewer 1 Stream", frame)

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    cam.release()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()
