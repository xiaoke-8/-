import numpy as np
from utils import *

total_time = 5
fps = 10
delta_T = 1.0 / fps

map_width = 315
map_height = 420

startX = 30.43
startY = 42
startDirection = -1


def check_collision(car_x, car_y, car_direction):
    car_points = get_square_points(car_x, car_y, car_direction)
    # 车体四条边
    for i in range(4):
        p1 = car_points[i]
        p2 = car_points[(i + 1) % 4]
        for x1, y1, x2, y2 in walls:
            q1 = (x1, y1)
            q2 = (x2, y2)
            if is_segments_intersect(p1, p2, q1, q2):
                return True
    return False


class NavigationEnv:
    def __init__(self):
        self.car_positionX = startX
        self.car_positionY = startY
        self.car_direction = startDirection

        self.time = 0.0

        self.vx = 0.0
        self.vy = 0.0
        self.vw = 0.0
        return

    def step(self, action):
        self.time += delta_T

        self.vx = action[0]
        self.vy = action[1]
        self.vw = action[2]

        self.vdx = (
            self.vx * np.cos(self.car_direction) - self.vy * np.sin(self.car_direction)
        ) * 100
        self.vdy = (
            self.vx * np.sin(self.car_direction) + self.vy * np.cos(self.car_direction)
        ) * 100
        self.car_positionX += self.vdx * delta_T
        self.car_positionY += self.vdy * delta_T

        self.car_positionX = np.clip(self.car_positionX, 0, map_width)
        self.car_positionY = np.clip(self.car_positionY, 0, map_height)
        self.car_direction += self.vw * delta_T
        self.car_direction = self.car_direction % (2 * np.pi)

        # 检查是否碰撞
        collided = check_collision(
            self.car_positionX, self.car_positionY, self.car_direction
        )
        if collided:
            return False  # 失败
        return True  # 成功

    def render(self):
        # Print every information prettily
        print(
            f"Car Position: ({self.car_positionX:.2f}, {self.car_positionY:.2f}), Direction: {self.car_direction:.2f}, Time: {self.time:.2f}"
        )
        print(f"Velocity: (Vx: {self.vx:.2f}, Vy: {self.vy:.2f}, Vw: {self.vw:.2f})")
