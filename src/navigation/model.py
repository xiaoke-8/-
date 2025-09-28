from navigation.utils import *


class CarModel:
    def __init__(self):
        self.positionX = None
        self.positionY = None
        self.direction = None
        self.vx = 0
        self.vy = 0
        self.vw = 0
        self.time = 0
        self.del_time = 0

        self.path = None
        self.current_path_index = 0
        self.last_visit = np.array([-5] * len(nodes))
        self.current_node = None
        self.current_target = None

    def update_information(self, observation):
        positionX = observation.get("positionX", None)
        positionY = observation.get("positionY", None)
        direction = observation.get("direction", None)
        time = observation.get("time", None)

        self.del_time = time - self.time
        self.time = time

        if positionX is not None and positionY is not None and direction is not None:
            self.positionX = positionX
            self.positionY = positionY
            self.direction = direction
        else:
            # Estimate position based on last known velocity and time difference
            vdx = (
                self.vx * np.cos(self.direction) - self.vy * np.sin(self.direction)
            ) * 100
            vdy = (
                self.vx * np.sin(self.direction) + self.vy * np.cos(self.direction)
            ) * 100
            self.positionX += vdx * self.del_time
            self.positionY += vdy * self.del_time
            self.direction += self.vw * self.del_time
            self.direction = self.direction % (2 * np.pi)

    def update_visit(self):
        if self.positionX is None or self.positionY is None:
            return
        for i, (nx, ny, _) in enumerate(nodes):
            dist = np.hypot(self.positionX - nx, self.positionY - ny)
            if dist < visit_range:  # Within car width to consider visited
                self.last_visit[i] = self.time
                if i == self.current_target:
                    self.path = None  # Reached target, need to replan

    def update_current_node(self):
        if self.positionX is None or self.positionY is None:
            return

        accessible_nodes = []
        for i, (nx, ny, _) in enumerate(nodes):
            if all(
                not is_segments_intersect(
                    (self.positionX, self.positionY), (nx, ny), (x1, y1), (x2, y2)
                )
                for x1, y1, x2, y2 in walls
            ):
                accessible_nodes.append(i)

        if not accessible_nodes:
            # Get the nearest node if no accessible nodes found
            distances = [
                np.hypot(self.positionX - nx, self.positionY - ny) for nx, ny in nodes
            ]
            self.current_node = np.argmin(distances)
            return

        # Choose the closest accessible node
        distances = [
            np.hypot(self.positionX - nodes[i][0], self.positionY - nodes[i][1])
            for i in accessible_nodes
        ]
        self.current_node = accessible_nodes[np.argmin(distances)]

    def find_max_visit_path(self, start, goal, edges, n_nodes):
        # 构建邻接表
        from collections import defaultdict

        graph = defaultdict(list)
        for u, v in edges:
            graph[u].append(v)
            graph[v].append(u)

        max_sum = float("-inf")
        best_path = []

        def dfs(node, path, visited, curr_sum, curr_dist):
            nonlocal max_sum, best_path
            if node == goal:
                total_sum = curr_sum / curr_dist if curr_dist > 0 else curr_sum
                if total_sum > max_sum:
                    # print(
                    #     f"New best path found: {path} with score {total_sum:.2f}, {curr_sum}, {curr_dist}"
                    # )
                    max_sum = total_sum
                    best_path = path[:]
                return
            for neighbor in graph[node]:
                if neighbor not in visited:
                    visit_weight = self.time - self.last_visit[neighbor]
                    distance = np.hypot(
                        nodes[neighbor][0] - nodes[node][0],
                        nodes[neighbor][1] - nodes[node][1],
                    )
                    visited.add(neighbor)
                    path.append(neighbor)
                    dfs(
                        neighbor,
                        path,
                        visited,
                        curr_sum + visit_weight,
                        curr_dist + distance,
                    )
                    path.pop()
                    visited.remove(neighbor)

        dfs(start, [start], set([start]), 0, 0)
        return best_path

    def plan_path(self):
        print("Start planning path..")

        # First find the node that is lowest in visit time
        distances = np.array(
            [
                (
                    np.hypot(self.positionX - nodes[i][0], self.positionY - nodes[i][1])
                    * beta
                    + (self.time - self.last_visit[i])
                )
                * nodes[i][2]
                for i in range(len(nodes))
            ]
        )
        target_node = np.argmax(distances)
        self.current_target = target_node

        print(f"Current node: {self.current_node}, Target node: {target_node}")

        # DFS to find path from current position to target_node
        try:
            path = self.find_max_visit_path(
                self.current_node, target_node, edges, len(nodes)
            )
            print(f"Planned path: {path}")
            # Divide path into segments with length at most divide_length
            divided_path = []
            for i in range(len(path) - 1):
                x1, y1, _ = nodes[path[i]]
                x2, y2, _ = nodes[path[i + 1]]
                dist = np.hypot(x2 - x1, y2 - y1)
                n_divisions = max(0, int(dist // divide_length)) + 1
                for j in range(n_divisions):
                    t = j / n_divisions
                    nx = x1 + t * (x2 - x1)
                    ny = y1 + t * (y2 - y1)
                    divided_path.append((nx, ny))
            # print(f"Divided path: {divided_path}")
            self.path = divided_path
            self.current_path_index = 0
        except ValueError as e:
            print(e)
            self.path = []

    def action(self):
        if self.positionX is None or self.positionY is None or self.direction is None:
            return np.array([0.0, 0.0, 0.0])

        if self.path is None or len(self.path) == 0:
            return np.array([0.0, 0.0, 0.0])
        # print(
        #     f"Current position: ({self.positionX:.2f}, {self.positionY:.2f}), Direction: {self.direction:.2f} time: {self.time:.2f}"
        # )

        # Find the next point that is at least Ld
        velocity = np.sqrt(self.vx**2 + self.vy**2)
        Ld = k * velocity + L0
        while self.current_path_index < len(self.path) - 1:
            px, py = self.path[self.current_path_index]
            dist = np.hypot(px - self.positionX, py - self.positionY)
            if dist >= Ld:
                break
            self.current_path_index += 1

        target_x, target_y = self.path[self.current_path_index]
        # print(f"target_x: {target_x}, target_y: {target_y}")

        remain_dis = 0
        for i in range(self.current_path_index, len(self.path) - 1):
            remain_dis += np.hypot(
                self.path[i + 1][0] - self.path[i][0],
                self.path[i + 1][1] - self.path[i][1],
            )
        # print(f"Remaining distance to target: {remain_dis:.2f}")
        # if remain_dis < target_range:
        #     self.path = None

        # Move towards the target point

        absolute_dx = (target_x - self.positionX) / 100  # convert to meters
        absolute_dy = (target_y - self.positionY) / 100  # convert to meters
        distance = np.hypot(absolute_dx, absolute_dy)

        absolute_angle = np.arctan2(absolute_dy, absolute_dx)
        alpha = (absolute_angle - (self.direction + np.pi / 2) + np.pi) % (
            2 * np.pi
        ) - np.pi
        gamma = 2 * np.sin(alpha) / max(distance, 1e-5)

        # print(
        #     f"Distance to target: {distance:.2f}, Alpha: {alpha:.2f}, Gamma: {gamma:.2f}, Ld: {Ld:.2f}"
        # )

        if abs(alpha) > np.pi / 4:
            # Too large angle, stop and turn
            target_v = 0.0
            target_w = gamma * 2
        else:
            target_v = max(0.3, vx_limit - abs(gamma) * 0.1)
            target_w = gamma * target_v

        absolute_vx = absolute_dx / max(distance, 1e-5) * target_v
        absolute_vy = absolute_dy / max(distance, 1e-5) * target_v
        self.vx = np.clip(
            absolute_vx * np.cos(self.direction) + absolute_vy * np.sin(self.direction),
            -vx_limit,
            vx_limit,
        )
        self.vy = np.clip(
            -absolute_vx * np.sin(self.direction)
            + absolute_vy * np.cos(self.direction),
            -vy_limit,
            vy_limit,
        )
        self.vw = np.clip(target_w, -vw_limit, vw_limit)

        # print(f"Action: vx={self.vx:.2f}, vy={self.vy:.2f}, vw={self.vw:.2f}")
        # print("\n")

        return np.array([self.vx, self.vy, self.vw])

    def main(self, observation):
        # Observation: {positionX: , positionY: , direction: , time: }
        self.update_information(observation)
        self.update_visit()
        self.update_current_node()

        if self.path is None or len(self.path) == 0:
            self.plan_path()

        action = self.action()

        return action
