from env import NavigationEnv
from model import CarModel
from visualize import visualize_episode
from env import total_time, fps


def main():
    now_time = __import__("time").strftime(
        "%Y-%m-%d_%H-%M-%S", __import__("time").localtime()
    )
    name = now_time

    # 创建环境
    env = NavigationEnv()

    # 加载模型
    model = CarModel()

    # 测试模型
    visualize_episode(
        env,
        model,
        steps=total_time * fps,
        save_path=f"out/result_video_{name}.mp4",
    )


if __name__ == "__main__":
    main()
