from maix import time, camera, display, image
from dredge.uarttransmit import CMD_Packet, DataTransimit
from config import *
from navigation.model import CarModel
from location import Location
import numpy as np

# ===== 全局变量 =====
global start_time
global cur_time
action = [0, 0, 0]

CMD = CMD_Packet()
car_model = CarModel()


def update_time():
    global cur_time
    cur_time = time.time() - start_time


# ===== 初始化硬件 =====
cam = camera.Camera(width=320, height=240, fps=15)

screen = display.Display(width=640, height=480)


def motor_run(Vx, Vy, Vw, motorsPos = []):
    CMD.Vx = -Vx
    CMD.Vy = -Vy
    CMD.Vw = Vw
    CMD.motorsPos = motorsPos
    DataTransimit(CMD)


start_time = time.time()
cur_time = 0.0
while True:
    img = cam.read()

    # Locate myself using AprilTag
    positionX, positionY, direction, img = Location().locate(img)
    if direction is not None:
        direction -= np.pi / 2

    # Move
    update_time()
    observations = {
        "positionX": positionX,
        "positionY": positionY,
        "direction": direction,
        "time": cur_time,
    }
    
    print(f"Time: {cur_time:.3f}")
    print(f"  Detect Pos: {positionX}, {positionY}, {direction}")

    action = car_model.predict(observations)

    if cur_time > 60:
        break

    # action = [0, 0, 1]
    motor_run(action[0], action[1], action[2])

    # screen.show(img)

    update_time()

    print(
        f"  Pos: ({car_model.positionX:.1f}, {car_model.positionY:.1f}, {car_model.direction:.2f})"
    )
    print(f"  Action: (Vx: {action[0]:.2f}, Vy: {action[1]:.2f}, Vw: {action[2]:.2f})")
