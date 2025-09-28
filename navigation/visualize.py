import matplotlib.pyplot as plt
import matplotlib.animation as animation
import numpy as np

from env import fps, map_height, map_width
from utils import *


def visualize_episode(env, model, steps, save_path):
    fig, ax = plt.subplots(figsize=(8, 6))

    (positions, directions, vxs, vys, vws) = ([], [], [], [], [])
    texts = []

    for t in range(steps):
        action = model.main(
            {
                "positionX": env.car_positionX,
                "positionY": env.car_positionY,
                "direction": env.car_direction,
                "time": env.time,
            }
        )
        result = env.step(action)
        if not result:
            print("Environment step failed.")
            break

        # env.render()

        # 记录数据
        x, y = env.car_positionX, env.car_positionY
        direction = env.car_direction
        vx, vy = getattr(env, "vx", 0), getattr(env, "vy", 0)
        vw = getattr(env, "vw", 0)
        positions.append((x, y))
        directions.append(direction)
        vxs.append(vx)
        vys.append(vy)
        vws.append(vw)

        texts.append(f"t={t}")

    print("Test round complete. Length:", len(positions))

    def update(frame):
        ax.clear()
        # 保证比例一致
        ax.set_xlim(0, 315)
        ax.set_ylim(0, 420)
        ax.set_aspect("equal")
        ax.set_title(texts[frame])

        # 绘制节点和边
        for i, (nx, ny, nw) in enumerate(nodes):
            ax.plot(nx, ny, "bo", markersize=nw * 3)
            ax.text(nx + 2, ny + 2, str(i), fontsize=8)
        for i, j in edges:
            x1, y1, _ = nodes[i]
            x2, y2, _ = nodes[j]
            ax.plot([x1, x2], [y1, y2], "b--", linewidth=0.5)

        # 绘制 platform_area 和 highland_area（矩形）
        for area, color, label in [
            (platform_area, "cyan", "Platform"),
            (highland_area, "yellow", "Highland"),
        ]:
            x1, y1, x2, y2 = area
            rect = plt.Rectangle(
                (x1, y1),
                x2 - x1,
                y2 - y1,
                linewidth=2,
                edgecolor=color,
                facecolor="none",
                label=label,
            )
            ax.add_patch(rect)

        # 绘制 walls（线段）
        for x1, y1, x2, y2 in walls:
            ax.plot([x1, x2], [y1, y2], color="black", linewidth=2)

        # 绘制小车轨迹
        direction = directions[frame]
        xs, ys = zip(*positions[max(0, frame - 3 * 30) : frame + 1])
        ax.plot(xs, ys, ".", label="Trajectory")

        # 绘制当前小车为正方形
        x, y = xs[-1], ys[-1]
        half_w = car_width / 2
        # 正方形顶点（未旋转）
        rel_points = [
            [half_w, half_w],
            [half_w, -half_w],
            [-half_w, -half_w],
            [-half_w, half_w],
            [half_w, half_w],  # 闭合
        ]

        rot = np.array(
            [
                [np.cos(direction), -np.sin(direction)],
                [np.sin(direction), np.cos(direction)],
            ]
        )
        abs_points = np.dot(rel_points, rot.T) + np.array([x, y])
        ax.plot(abs_points[:, 0], abs_points[:, 1], "r-", linewidth=2, label="Car Body")
        ax.plot(x, y, "ro")  # 车体中心

        # 绘制速度箭头
        vx, vy, vw = vxs[frame], vys[frame], vws[frame]
        dx = vx * np.cos(direction) - vy * np.sin(direction)
        dy = vx * np.sin(direction) + vy * np.cos(direction)
        ax.arrow(
            x,
            y,
            dx * 3,
            dy * 3,
            head_width=5,
            head_length=8,
            fc="g",
            ec="g",
            label="Velocity",
        )

        forward_len = 25
        fx = x + forward_len * np.cos(direction + np.pi / 2)
        fy = y + forward_len * np.sin(direction + np.pi / 2)
        ax.arrow(
            x,
            y,
            fx - x,
            fy - y,
            head_width=5,
            head_length=8,
            fc="orange",
            ec="orange",
            label="Forward",
        )
        ax.text(10, 400, f"vx={vx:.2f}, vy={vy:.2f}, vw={vw:.2f}", fontsize=10)

    print("Visualizing episode start...")
    ani = animation.FuncAnimation(
        fig, update, frames=len(positions), interval=1000 / fps
    )
    ani.save(save_path, fps=fps)
    plt.close(fig)

    print(f"Visualization saved to {save_path}")
