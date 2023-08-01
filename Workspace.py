from typing import List, Union, Tuple

import matplotlib.figure
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle, Polygon


class WorkSpace:
    def __init__(self, corners: List[float], start: Tuple[float, float], goal: Tuple[float, float], resolution: float = 1):
        self.x_min = corners[0]
        self.x_max = corners[1]
        self.y_min = corners[2]
        self.y_max = corners[3]
        self.start = start
        self.goal = goal
        self.obs = None
        self.resolution = resolution
        self.fig = None
        self.ax = None

    def create_workspace(self):
        fig, ax = plt.subplots()
        self.fig = fig
        self.ax = ax
        plt.ylim(self.y_min - 0.2, self.y_max + 0.2)
        plt.xlim(self.x_min - 0.2, self.x_max + 0.2)

        boundary = set()
        res = self.resolution
        expanded_x_max, expanded_x_min = self.x_max + res, self.x_min - res
        expanded_y_max, expanded_y_min = self.y_max + res, self.y_min - res

        x_val = np.arange(expanded_x_min, expanded_x_max, 0.01) # 0.01 granularity
        y_val = np.arange(expanded_y_min, expanded_y_max, 0.01)

        for x in x_val:
            boundary.add((x, expanded_y_min))
            boundary.add((x, expanded_y_max))

        for y in y_val:
            boundary.add((expanded_x_min, y))
            boundary.add((expanded_x_max, y))

        bound_x = [x[0] for x in boundary]
        bound_y = [x[1] for x in boundary]

        plt.plot(bound_x, bound_y, "sk")

        rect_x = np.arange(self.x_min, self.x_max + self.resolution, self.resolution)
        rect_y = np.arange(self.y_min, self.y_max + self.resolution, self.resolution)

        # for x in rect_x:
        #     for y in rect_y:
        #         plt.plot(x, y, "cs")

        plt.plot(self.start[0], self.start[1], "bs")
        plt.plot(self.goal[0], self.goal[1], "gs")
        plt.axis("equal")

        return

    def add_obstacles(self, obs_pos: List[Tuple[float, float]]):
        self.obs = obs_pos
        obs_x_pos = [x[0] for x in obs_pos]
        obs_y_pos = [x[1] for x in obs_pos]
        plt.plot(obs_x_pos, obs_y_pos, "sk")

    def calculate_centroid(self) -> Tuple[float, float]:
        x_pos = np.array([x[0] for x in self.obs])
        np.append(x_pos, self.start[0])

        y_pos = np.array([x[1] for x in self.obs])
        np.append(y_pos, self.start[1])

        x_centroid = np.average(x_pos)
        y_centroid = np.average(y_pos)

        return x_centroid, y_centroid


class WorkSpaceContinuous(WorkSpace):
    def __init__(self, *args, **kwargs):
        super(WorkSpaceContinuous, self).__init__(*args, **kwargs)

    def add_obstacles(self, obs_pos: List[Tuple[float, float]]):
        y = np.array([[1, 1], [2, 1], [2, 2], [1, 2], [0.5, 1.5]])
        p = Polygon(y, facecolor='k')
        self.ax.add_patch(p)

        y = np.array([[10, 10], [40, 10], [40, 25], [10, 25]])
        p = Polygon(y, facecolor='k')
        self.ax.add_patch(p)


if __name__ == '__main__':
    corners = [-0.8, 0.3, 0, 0.55]
    start = (-0.2, 0.2)
    goal = (0.1, 0.5)
    obstacles = [(-0.1, 0.1), (-0.5, 0.3), (-0.6, 0.3), (-0.1, 0.1), (0.2, 0.3), (0.02, 0.3), (0.025, 0.343), (0.423, 0.097), (0.032, 0.432)]
    # obstacles = [(-0.1, 0.1), (-0.5, 0.1), (-0.1, 0.4), (-0.5, 0.4)]
    ws = WorkSpace(corners, start, goal, 0.05)
    fig = ws.create_workspace()
    ws.add_obstacles(obstacles)

    centroid = ws.calculate_centroid()
    plt.plot(centroid[0], centroid[1], "ro")

    plt.show()
