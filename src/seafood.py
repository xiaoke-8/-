from maix import camera, display, image, nn, app, comm, time
from maix._maix import err
import struct, os


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


model_path = "model_237496.mud"
if not os.path.exists(model_path):
    model_path = "/root/race/model-237496.maixcam/model_237496.mud"
detector = nn.YOLOv5(model=model_path)

# Parameters

Kp_turn = 0.003
forward_speed = 1.0  # 前进速度
target_center_x = 160  # 图像中心X坐标
dead_zone = 0.7


class Seafood:
    def __init__(self):
        self.Vx = 0.0
        self.Vy = 0.0
        self.Vw = 0.0
        self.start_delay_time = 0.0
        self.goodbye = False

    def detect(self, img):
        objs = detector.detect(img, conf_th=0.5, iou_th=0.45)
        return objs

    def pick(self, img, objs, cur_time):
        if len(objs) > 0:
            # 选择置信度最高的物体
            best_obj = max(objs, key=lambda obj: obj.score)

            # 计算物体中心坐标
            obj_center_x = best_obj.x + best_obj.w // 2

            # 计算与图像中心的偏差
            error_x = obj_center_x - target_center_x

            # 转向控制
            error_ratio = float(abs(error_x)) / float(best_obj.w)
            print(error_x, best_obj.w, error_ratio)
            if error_ratio > dead_zone:
                self.Vw = float(-Kp_turn * error_x)
                self.Vw = max(min(self.Vw, 0.5), -0.5)
                self.Vy = 0.0  # 转向时不前进
                # print(f"转向中... 误差: {error_x}, {self.Vw}")
            else:
                # 已对准中线，前进
                self.Vw = 0.0  # 停止转向
                self.Vy = forward_speed  # 前进
                self.start_delay_time = cur_time
                # print(f"前进中... 物体位于中线，Vy: {self.Vy}")
        else:
            if cur_time - self.start_delay_time < 0.4:
                self.Vx = 0.0
                self.Vy = forward_speed
                self.Vw = 0.0
            else:
                # 没有检测到物体时停止
                self.Vx = 0.0
                self.Vy = 0.0
                self.Vw = 0.0
                self.goodbye = True
                # print("未检测到物体，停止")

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

        # # 绘制图像中心线
        # img.draw_line(target_center_x, 0, target_center_x, detector.input_height(),
        #              color = image.COLOR_GREEN, thickness = 2)

        # 显示控制状态
        # status_text = f"self.Vy: {self.Vy:.1f}, self.Vw: {self.Vw:.1f}"
        # img.draw_string(10, 10, status_text, color = image.COLOR_WHITE)

        return self.Vx, self.Vy, self.Vw, self.goodbye
