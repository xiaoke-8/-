from maix import time, camera, display, image
from dredge.uarttransmit import CMD_Packet, DataTransimit
from config import *
from navigation.model import CarModel
from location import Location


# ===== 全局变量 =====
cur_time = 0.0
delta_T = 1 / fps

CMD = CMD_Packet()

# ===== 初始化硬件 =====
cam = camera.Camera(width=320, height=240, fps=fps)

screen = display.Display(width=640, height=480)


def motor_run(Vx, Vy, Vw, motorsPos):
    CMD.Vx = -Vx
    CMD.Vy = -Vy
    CMD.Vw = Vw
    CMD.motorsPos = motorsPos
    DataTransimit(CMD)


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

    # action = [1,0,0]
    # if cur_time > 3:
    #     action = [0,0,0]

    # motor_run(action[0], action[1], action[2], [])

    time.sleep_ms(delta_T)
    cur_time += delta_T
