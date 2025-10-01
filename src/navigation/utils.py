import numpy as np

# Environment parameters


car_width = 30

vx_limit = 1.5
vy_limit = 1.5
vw_limit = 0.5

# Model parameters

visit_range = 20.0  # Distance to consider a node visited
target_range = 80.0  # Distance to consider target reached
beta = 0.1  # Weight for distance in target selection
divide_length = 5.0  # Max segment length when dividing path
k = 30.0  # Lookahead distance factor
L0 = 30.0  # Minimum lookahead distance
align_threshold_move = (
    0.5  # Threshold to consider aligned with target direction when moving
)
align_threshold_adjust = (
    0.12  # Threshold to consider aligned with target direction when adjusting
)

# Coordinates

platform_area = (70, 150, 240, 240)
highland_area = (70, 110, 240, 280)  # 73.5 115.5 235 285

# x1, y1, x2, y2
walls = [
    (0, 0, 315, 0),
    (0, 0, 0, 420),
    (315, 0, 315, 420),
    (0, 420, 315, 420),
    (70, 110, 70, 280),
    (240, 110, 240, 280),
    (90, 110, 90, 280),
    (220, 110, 220, 280),
]

light_walls = [
    (0, 0, 315, 0),
    (0, 0, 0, 420),
    (315, 0, 315, 420),
    (0, 420, 315, 420),
    (70, 110, 70, 280),
    (240, 110, 240, 280),
]

nodes = [
    (140, 227, 1.2),  # 0
    (140, 178, 1.2),  # 1
    (165, 227, 1.2),  # 2
    (165, 178, 1.2),  # 3
    (40, 70, 1),  # 4
    (140, 70, 1),  # 5
    (165, 70, 1),  # 6
    (275, 70, 1),  # 7
    (40, 45, 1),  # 8
    (140, 45, 1),  # 9
    (165, 45, 1),  # 10
    (275, 45, 0),  # 11
    (40, 340, 1),  # 12
    (140, 340, 1),  # 13
    (165, 340, 1),  # 14
    (275, 340, 1),  # 15
    (140, 370, 1),  # 16
    (165, 370, 1),  # 17
    (275, 370, 1),  # 18
    (40, 198, 1.2),  # 19
    (275, 198, 1.2),  # 20
    (40, 370, 0),  # 21
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
    (7, 11),
    (4, 8),
    (12, 19),
    (19, 4),
    (7, 20),
    (20, 15),
    (12, 13),
    (14, 15),
    (17, 18),
    (18, 15),
    (16, 12),
    (16, 17),
    (17, 14),
    (14, 13),
    (13, 16),
    (0, 1),
    (2, 3),
    (5, 6),
    (6, 10),
    (10, 9),
    (9, 5),
    (15, 17),
    (4, 9),
    (10, 7),
    (12, 21),
    (21, 16),
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


def get_square_points(x, y, angle):
    # 计算正方形四个顶点坐标
    half_w = car_width / 2
    # 正方形顶点相对中心的坐标（未旋转）
    rel_points = np.array(
        [[half_w, half_w], [half_w, -half_w], [-half_w, -half_w], [-half_w, half_w]]
    )
    # 旋转矩阵
    rot = np.array([[np.cos(angle), -np.sin(angle)], [np.sin(angle), np.cos(angle)]])
    # 旋转并平移
    abs_points = np.dot(rel_points, rot.T) + np.array([x, y])
    return abs_points  # shape (4,2)
