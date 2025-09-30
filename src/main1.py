from maix import camera, display, image, nn, app, comm
from maix._maix import err
from dredge.uarttransmit import CMD_Packet, DataTransimit
import struct, os
import time  # 这是 MaixPy 的 time 模块
from location import Location
from navigation.model import CarModel


def motor_run(Vx, Vy, Vw, motorsPos):
    CMD.Vx = -Vx
    CMD.Vy = -Vy
    CMD.Vw = Vw
    print(CMD.Vw)
    CMD.motorsPos = motorsPos
    DataTransimit(CMD)


report_on = True
APP_CMD_DETECT_RES = 0x02


def encode_objs(objs):
    """
    encode objs info to bytes body for protocol
    2B x(LE) + 2B y(LE) + 2B w(LE) + 2B h(LE) + 2B idx + 4B score(float) ...
    """
    body = b""
    for obj in objs:
        body += struct.pack(
            "<hhHHHf", obj.x, obj.y, obj.w, obj.h, obj.class_id, obj.score
        )
    return body


model_path = "model_235995.mud"
if not os.path.exists(model_path):
    model_path = "/root/models/race/model_235995.mud"
detector = nn.YOLOv5(model=model_path)

cam = camera.Camera(
    detector.input_width(), detector.input_height(), detector.input_format()
)
dis = display.Display()

p = comm.CommProtocol(buff_size=1024)

# ===== 添加控制参数 =====

CMD = CMD_Packet()
Kp_turn = 0.01
forward_speed = 1.5  # 前进速度
target_center_x = detector.input_width() // 2  # 图像中心X坐标
dead_zone = 25
print("Seafood 检测器初始化完成，开始物体追踪...")

Vx = 0.0
Vy = 0.0
Vw = 0.0
delay_time = 0

while not app.need_exit():
    img = cam.read()
    objs = detector.detect(img, conf_th=0.5, iou_th=0.45)

    # ===== 添加控制逻辑 =====
    if len(objs) > 0:
        # 选择置信度最高的物体
        best_obj = max(objs, key=lambda obj: obj.score)

        # 计算物体中心坐标
        obj_center_x = best_obj.x + best_obj.w // 2

        # 计算与图像中心的偏差
        error_x = obj_center_x - target_center_x

        # 转向控制
        if abs(error_x) > dead_zone:
            Vw = float(-Kp_turn * error_x)
            Vw = max(min(Vw, 3), -3)
            Vy = 0.0  # 转向时不前进
            print(f"转向中... 误差: {error_x}, {Vw}")
        else:
            # 已对准中线，前进
            Vw = 0.0  # 停止转向
            Vy = forward_speed  # 前进
            delay_time = 10
            print(f"前进中... 物体位于中线，Vy: {Vy}")

        if report_on:
            body = encode_objs(objs)
            p.report(APP_CMD_DETECT_RES, body)
    else:
        if delay_time > 0:
            delay_time -= 1
        else:
            # 没有检测到物体时停止
            Vx = 0.0
            Vy = 0.0
            Vw = 0.0
            print("未检测到物体，停止")

    print(Vx, Vy, Vw)
    motor_run(Vx, Vy, Vw, [])

    # ===== 添加可视化增强 =====
    for obj in objs:
        img.draw_rect(obj.x, obj.y, obj.w, obj.h, color=image.COLOR_RED)
        msg = f"{detector.labels[obj.class_id]}: {obj.score:.2f}"
        img.draw_string(obj.x, obj.y, msg, color=image.COLOR_RED)

        # 绘制物体中心点
        obj_center_x = obj.x + obj.w // 2
        obj_center_y = obj.y + obj.h // 2
        img.draw_circle(
            obj_center_x, obj_center_y, 5, color=image.COLOR_BLUE, thickness=-1
        )

    # 绘制图像中心线
    img.draw_line(
        target_center_x,
        0,
        target_center_x,
        detector.input_height(),
        color=image.COLOR_GREEN,
        thickness=2,
    )

    # 显示控制状态
    status_text = f"Vy: {Vy:.1f}, Vw: {Vw:.1f}"
    img.draw_string(10, 10, status_text, color=image.COLOR_WHITE)

    dis.show(img)
    # 修复时间延迟问题
    time.sleep(0.05)  # 使用 time.sleep() 替代 time.sleep_ms()
