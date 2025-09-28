from maix import time, camera, display, image
from dredge.uarttransmit import CMD_Packet, DataTransimit
from config import *
from navigation.model import CarModel
from location import Location


# ===== 全局变量 =====
start_time = 0.0
cur_time = 0.0
action = [0, 0, 0]

CMD = CMD_Packet()

# ===== 初始化硬件 =====
cam = camera.Camera(width=320, height=240, fps=30)

screen = display.Display(width=640, height=480)

def motor_run(Vx, Vy, Vw, motorsPos):
    CMD.Vx = -Vx
    CMD.Vy = -Vy
    CMD.Vw = Vw
    CMD.motorsPos = motorsPos
    DataTransimit(CMD)

start_time = time.time()
while True:
    img = cam.read()
    screen.show(img)

    # Locate myself using AprilTag
    img, positionX, positionY, direction = Location().locate(img)

    # Move
    observations = {
        "positionX": positionX,
        "positionY": positionY,
        "direction": direction,
        "time": time,
    }
    # action = CarModel.main(observations)

    action = [0,0,1]
    if cur_time > 3:
        action = [0,0,0]

    motor_run(action[0], action[1], action[2], [])

    cur_time = time.time() - start_time
    # print(f"{cur_time:.3f}")
