from siyi_sdk import SIYISDK
import time

CAMERA_IP = "192.168.144.108"

cam = SIYISDK(server_ip=CAMERA_IP)

cam.connect()
time.sleep(1)

# Move gimbal: pan right, tilt up
cam.gimbal_speed_control(pan_speed=30, tilt_speed=-20)
time.sleep(2)

# Stop
cam.gimbal_speed_control(0, 0)

# Center gimbal
cam.gimbal_center()

cam.disconnect()
