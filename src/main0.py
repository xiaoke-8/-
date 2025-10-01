from maix import time, camera, display, image
from dredge.uarttransmit import CMD_Packet, DataTransimit
from config import *
from navigation.model import CarModel
from navigation.model import check_collide_with_walls
from location import Location
from seafood import Seafood
import numpy as np

# ===== 全局变量 =====
global start_time
global cur_time
action = [0, 0, 0]

CMD = CMD_Packet()


def update_time():
    global cur_time
    cur_time = time.time() - start_time


# ===== 初始化硬件 =====
cam = camera.Camera(width=320, height=240, fps=15)
screen = display.Display(width=640, height=480)


def motor_run(Vx, Vy, Vw, motorsPos=[]):
    CMD.Vx = -Vx
    CMD.Vy = -Vy
    CMD.Vw = Vw
    CMD.motorsPos = motorsPos
    DataTransimit(CMD)


while True:
    start_time = time.time()
    cur_time = 0.0
    update_time()
    motor_run(1, 0, 0)
    time.sleep_ms(20)
