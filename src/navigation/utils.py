import numpy as np

# Environment parameters

car_width = 30

vx_limit = 1.5
vy_limit = 1.5
vw_limit = 3.0

# Model parameters

visit_range = 20.0  # Distance to consider a node visited
target_range = 5.0  # Distance to consider target reached
beta = 0.1  # Weight for distance in target selection
divide_length = 10.0  # Max segment length when dividing path
k = 5.0  # Lookahead distance factor
L0 = 20.0  # Minimum lookahead distance

# Coordinates

platform_area = (67.5, 160.1, 247.5, 259.9)
highland_area = (67.5, 114.1, 247.5, 305.9)

# x1, y1, x2, y2
walls = [
    (0, 0, 315, 0),
    (0, 0, 0, 420),
    (315, 0, 315, 420),
    (0, 420, 315, 420),
    (67.5, 114.1, 67.5, 305.9),
    (247.5, 114.1, 247.5, 305.9),
]

# (x, y, w)
nodes = [
    (120, 225, 1.1),  # 0
    (120, 185, 1.1),  # 1
    (195, 225, 1.1),  # 2
    (195, 185, 1.1),  # 3
    (33, 70, 1),  # 4
    (120, 70, 1),  # 5
    (195, 70, 1),  # 6
    (282, 70, 1),  # 7
    (33, 25, 1),  # 8
    (120, 25, 1),  # 9
    (195, 25, 1),  # 10
    (282, 25, 0),  # 11
    (33, 350, 1),  # 12
    (120, 350, 1),  # 13
    (195, 350, 1),  # 14
    (282, 350, 1),  # 15
    (120, 395, 1),  # 16
    (195, 395, 1),  # 17
    (282, 395, 1),  # 18
    (33, 210, 1.3),  # 19
    (282, 210, 1.3),  # 20
]

edges = [
    (0, 3),
    (2, 1),
    (1, 5),
    (3, 6),
    (0, 13),
    (2, 14),
    (4, 5),
    (6, 7),
    (8, 9),
    (10, 11),
    (5, 10),
    (6, 9),
    (7, 11),
    (4, 8),
    (12, 19),
    (19, 4),
    (7, 20),
    (20, 15),
    (12, 13),
    (14, 15),
    (16, 14),
    (13, 17),
    (17, 18),
    (18, 15),
    (16, 12),
]


# 点到线段的最短距离
def point2line_distance(px, py, x1, y1, x2, y2):
    line_vec = np.array([x2 - x1, y2 - y1])
    point_vec = np.array([px - x1, py - y1])
    line_len_sq = np.dot(line_vec, line_vec)
    if line_len_sq == 0:
        # 线段退化为一个点
        return np.linalg.norm(point_vec)
    t = np.dot(point_vec, line_vec) / line_len_sq
    t = np.clip(t, 0, 1)
    closest = np.array([x1, y1]) + t * line_vec
    return np.linalg.norm(np.array([px, py]) - closest)


# 判断线段p1p2与q1q2是否相交
def is_segments_intersect(p1, p2, q1, q2):
    def ccw(a, b, c):
        return (c[1] - a[1]) * (b[0] - a[0]) > (b[1] - a[1]) * (c[0] - a[0])

    return (ccw(p1, q1, q2) != ccw(p2, q1, q2)) and (ccw(p1, p2, q1) != ccw(p1, p2, q2))
