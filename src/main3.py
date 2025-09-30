from maix import time, camera, display, image
from dredge.uarttransmit import CMD_Packet, DataTransimit
from config import *
from color_recognition import ColorRecognition
import numpy as np

# ===== 全局变量 =====
global start_time
global cur_time
action = [0, 0, 0]

CMD = CMD_Packet()


def update_time():
    global cur_time
    cur_time = time.time() - start_time


image.load_font("ppocr", "/maixapp/share/font/ppocr_keys_v1.ttf", size=20)
image.set_default_font("ppocr")

# ===== 初始化硬件 =====
cam = camera.Camera(width=320, height=240, fps=30)
screen = display.Display(width=640, height=480)
color_recognition = ColorRecognition()

start_time = time.time()
cur_time = 0.0
while True:
    img = cam.read()

    # Do color recognition
    color_recognition.recognize_color(img)

    screen.show(img)

    time.sleep_ms(30)
    update_time()
    print(f"Time: {cur_time:.3f}")
