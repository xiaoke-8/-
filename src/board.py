from maix import time
from dredge.uarttransmit import CMD_Packet, DataTransimit
import numpy as np

# ===== 全局变量 =====
action = [0, 0, 0]


servo_yaw = 0
servo_pitch = 0

positions = [
    (0, 0),
    (158, 45),  # 1
    (145, 55),  # 2
    (125, 65),  # 3
    (152, 20),  # 4
    (138, 28),  # 5
    (113, 45),  # 6
    (150, 0),  # 7
    (134, 6),  # 8
    (110, 7),  # 9
]

perm = [1, 8, 3, 4, 9, 2, 7, 6, 5]


def shoot(id):
    (servo_yaw, servo_pitch) = positions[id]
    motor_run(0, 0, 0, [servo_yaw, servo_pitch])
    print(servo_yaw, servo_pitch)


start_time = time.time()
cur_time = 0.0

shoot(9)
time.sleep(3)

for r in range(2):
    for i in range(9):
        shoot(perm[i])
        time.sleep(1.5)
    time.sleep(1)
