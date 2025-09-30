from maix import image, nn
import numpy as np

# model = "/root/models/pp_ocr.mud"

# Parameters for color recognition
red_typical = [30, 45, 22]
yellow_typical = [71, -10, 70]
orange_typical = [44, 27, 35]
purple_typical = [16, 22, -28]
blue_typical = [17, 12, -31]
green_typical = [39, -15, 15]

red_thresholds = [20, 40, 30, 60, 10, 40]
yellow_thresholds = [40, 90, -30, 0, 40, 80]
orange_thresholds = [30, 50, 10, 40, 15, 45]
purple_thresholds = [10, 20, 15, 25, -30, -15]
blue_thresholds = [10, 20, 0, 20, -40, -25]
green_thresholds = [20, 60, -40, 5, 0, 30]

thresholds = [
    red_thresholds,
    yellow_thresholds,
    orange_thresholds,
    purple_thresholds,
    blue_thresholds,
    green_thresholds,
]
color_name = ["red", "yellow", "orange", "purple", "blue", "green"]

k = None


class ColorRecognition:
    def __init__(self):
        self.blobs = []
        # self.ocr = nn.PP_OCR(model)
        self.recognized_number = [("", 0)] * 9
        pass

    def check_duplicate(self, new_blob):
        for b in self.blobs:
            if (
                abs(b[1][0] - new_blob[0]) < 10
                and abs(b[1][1] - new_blob[1]) < 10
                and abs(b[1][2] - new_blob[2]) < 10
                and abs(b[1][3] - new_blob[3]) < 10
            ):
                return True
        return False

    def update_recognized_number(self, idx, char):
        # 摩尔投票法
        if char == "":
            return
        (old_char, count) = self.recognized_number[idx]
        if old_char != char:
            if count == 0:
                self.recognized_number[idx] = (char, 1)
            else:
                self.recognized_number[idx] = (char, count - 1)
        else:
            self.recognized_number[idx] = (char, count + 1)

    def recognize_color(self, img):
        self.blobs = []
        for i in range(len(thresholds)):
            blobs = img.find_blobs([thresholds[i]], area_threshold=300, merge=True)
            for blob in blobs:
                if not self.check_duplicate(blob):
                    self.blobs.append((color_name[i], blob))

        # 计算平均大小，并去除明显不符合平均大小的色块
        if len(self.blobs) > 0:
            areas = [b[1][2] * b[1][3] for b in self.blobs]
            mean_area = np.mean(areas)
            std_area = np.std(areas)
            filtered_blobs = [
                b
                for b in self.blobs
                if (mean_area - std_area) < (b[1][2] * b[1][3]) < (mean_area + std_area)
            ]
            self.blobs = filtered_blobs

        # 按 y 排序后，取离中位数最近的 9 个
        self.blobs.sort(key=lambda b: b[1][1])
        if len(self.blobs) > 9:
            median_index = len(self.blobs) // 2
            median_x = self.blobs[median_index][1][0]
            median_y = self.blobs[median_index][1][1]
            self.blobs.sort(
                key=lambda b: abs(b[1][0] - median_x) + abs(b[1][1] - median_y)
            )
            self.blobs = self.blobs[:9]

        self.blobs.sort(key=lambda b: (b[1][0], b[1][1]))

        for i, b in enumerate(self.blobs):
            color, blob = b

            box_points = [
                blob[0],
                blob[1],
                blob[0] + blob[2],
                blob[1],
                blob[0] + blob[2],
                blob[1] + blob[3],
                blob[0],
                blob[1] + blob[3],
            ]
            # obj = self.ocr.recognize(img, box_points)
            # char = obj.char_str()[:1] if obj else ""
            # if char not in "123456789":
            #     char = ""
            # self.update_recognized_number(i, char)
            # print(f"ID: {i}, Color: {color}, Char: {char}")

            img.draw_rect(blob[0], blob[1], blob[2], blob[3], image.COLOR_WHITE)
            img.draw_string(
                blob[0],
                blob[1] + blob[3],
                color,
                # color + ("(" + self.recognized_number[i][0] + ")" if obj else ""),
                color=image.COLOR_WHITE,
                scale=0.4,
            )
