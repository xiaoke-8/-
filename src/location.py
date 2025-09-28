from dredge.apriltagmap import generate_map_dict
from maix import image
import numpy as np

# AprilTag 识别参数
roi = []
families = image.ApriltagFamilies.TAG36H11
fx = float(-1)
fy = float(-1)
cx = 320
cy = 240

k = -9


class Location:
    def __init__(self):
        self.coord_map = generate_map_dict()

    def locate(self, img):
        apriltags = img.find_apriltags(roi, families, fx, fy, cx, cy)

        total_x = 0.0
        total_y = 0.0
        total_direction = 0.0
        weight_sum = 0.0

        for a in apriltags:
            tag_id = a.id()

            # 使用映射字典查询坐标
            if tag_id in self.coord_map:
                global_coord = self.coord_map[tag_id]
                x_trans = a.x_translation() * k - 27.0
                y_trans = a.y_translation() * k - 20.0
                z_trans = a.z_translation() * k
                z_rot = a.z_rotation()

                # 计算距离和相对位置
                distance = np.sqrt(x_trans**2 + y_trans**2 + z_trans**2)
                direction = (np.pi * 1.5 - z_rot + np.pi * 2 + 0.14) % (np.pi * 2)

                # 计算绝对坐标
                (idx, idy) = global_coord
                absolute_x = 315 - (24.5 + (idy - 1) * 19)
                absolute_y = 29.5 + (idx - 1) * 19

                # 输出精准坐标信息
                print(
                    f"Tag ID: {tag_id}, 全局坐标: {global_coord}, direction: {direction}"
                )
                print(
                    f"相对位置: X={x_trans:.2f}, Y={y_trans:.2f}, Z={z_trans:.2f} dis={distance:.2f}cm"
                )

                # 计算相机精准坐标
                camera_x = (
                    absolute_x
                    + x_trans * np.sin(direction)
                    + y_trans * np.cos(direction)
                )
                camera_y = (
                    absolute_y
                    - x_trans * np.cos(direction)
                    + y_trans * np.sin(direction)
                )

                # # 显示相机精准坐标
                # img.draw_string(a.x() + a.w() + 5, a.y() + 80, f"Camera: ({camera_x:.1f}, {camera_y:.1f})", image.COLOR_BLUE)
                # print(f"相机坐标: ({camera_x:.1f}, {camera_y:.1f})")
                # print("---")

                weight = 1.0 / distance
                weight_sum += weight
                total_x += camera_x * weight
                total_y += camera_y * weight
                total_direction += direction * weight

            # 绘制Tag边界和信息
            corners = a.corners()
            for i in range(4):
                img.draw_line(
                    corners[i][0],
                    corners[i][1],
                    corners[(i + 1) % 4][0],
                    corners[(i + 1) % 4][1],
                    image.COLOR_RED,
                    2,
                )

            rect = a.rect()
            # img.draw_rect(rect[0], rect[1], rect[2], rect[3], image.COLOR_BLUE, 2)
            img.draw_string(
                rect[0] + 5, rect[1] + 5, "ID: " + str(tag_id), image.COLOR_RED
            )

        if len(apriltags) == 0 or weight_sum < 1e-5:
            total_x = None
            total_y = None
            total_direction = None
        else:
            total_x /= weight_sum
            total_y /= weight_sum
            total_direction /= weight_sum

        # 显示检测状态
        if total_x is not None and total_y is not None and total_direction is not None:
            status_text = f"{total_x:.2f}, {total_y:.2f}, {total_direction:.2f}"
        else:
            status_text = "No Tag"
        img.draw_string(5, 5, status_text, color=image.COLOR_GREEN)

        # print(f"Coordinate: {total_x:.2f}, {total_y:.2f}, {total_direction:.2f}\n")
        return total_x, total_y, total_direction, img
