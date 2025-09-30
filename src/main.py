from maix import time, camera, display, image
from dredge.uarttransmit import CMD_Packet, DataTransimit
from config import *
from navigation.model import CarModel
from location import Location
from seafood import Seafood
import numpy as np

# ===== 全局变量 =====
global start_time
global cur_time
action = [0, 0, 0]

state = 1
# 1: 导航
# 2: 捕捞
# 3: 过渡
next_state = 0
transition_end_time = 0.0

CMD = CMD_Packet()
car_model = CarModel()
seafood = Seafood()
locator = Location()


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


def change_state(new_state, duration):
    global state, next_state, transition_start_time
    next_state = new_state
    state = 3  # Transition
    transition_end_time = cur_time + duration
    print(f"State change to {new_state}, transition...")


start_time = time.time()
cur_time = 0.0
while True:
    update_time()

    if cur_time > 60:
        break

    if state == 3:
        if cur_time < transition_end_time:
            motor_run(0.0, 0.0, 0.0)
            time.sleep_ms(50)
            continue
        else:
            print("Transition end, continue to next state " + str(next_state))
            state = next_state
            if state == 1:
                car_model.__init__()
            elif state == 2:
                seafood.__init__()

    img = cam.read()

    if state == 1:
        # Navigation
        seafood_objs = seafood.detect(img)

        if len(seafood_objs) > 0:
            change_state(2, 0)
            continue

        # Locate myself using AprilTag
        positionX, positionY, direction, img = locator.locate(img)
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

        motor_run(action[0], action[1], action[2])

    elif state == 2:
        # Seafood picking
        seafood_objs = seafood.detect(img)

        update_time()
        Vx, Vy, Vw, goodbye = seafood.pick(img, seafood_objs, cur_time)
        print(f"Seafood go: {Vx:.2f}, {Vy:.2f}, {Vw:.2f}, goodbye: {goodbye}")
        motor_run(Vx, Vy, Vw)

        screen.show(img)

        if goodbye:
            change_state(1, 0)
            continue
