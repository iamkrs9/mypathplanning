import heapq
import math
from typing import Tuple, List

from Workspace import WorkSpace
import matplotlib.pyplot as plt


class Djikstra:
    def __init__(self, start: Tuple[float, float], goal: Tuple[float, float], obstacles: List[Tuple[float, float]],
                 corners, res):
        self.res = res
        # self.dec = len(str(self.res).split('.')[1])
        self.dec = 1
        self.open = []
        self.closed = []
        self.travel = [
            (-res, res), (0, res),
            (res, res), (-res, 0),
            (res, 0), (-res, -res),
            (0, -res), (res, -res),
        ]
        self.costs = {}
        self.start = tuple((round(start[0], self.dec), round(start[1], self.dec)))
        self.goal = tuple((round(goal[0], self.dec), round(goal[1], self.dec)))
        self.obs = [tuple((round(p[0], self.dec), round(p[1], self.dec))) for p in obstacles]
        self.parent = {}
        self.corners = corners
        self.last_parent = None

    def get_neighbor(self, point: Tuple[float]) -> List[Tuple[float, float]]:
        neighbors = []
        for direction in self.travel:
            neighbor = (round(point[0] + direction[0], self.dec), round(point[1] + direction[1], self.dec))
            if point[0] <= self.corners[0] or point[0] >= self.corners[1] or point[1] <= self.corners[2] or point[1] >= \
                    self.corners[3]:
                continue
            neighbors.append(neighbor)

        return neighbors

    def is_collision(self, point1: Tuple[float, float], point2: Tuple[float, float]) -> bool:
        if (round(point1[0], self.dec), round(point1[1], self.dec)) in self.obs or (
        round(point2[0], self.dec), round(point2[1], self.dec)) in self.obs:
            return True

        return False

    def get_cost(self, point1: Tuple[float, float], point2: Tuple[float, float]) -> float:
        if self.is_collision(point1, point2):
            return math.inf

        return math.hypot(point1[0] - point2[0], point1[1] - point2[1])

    def search(self):
        self.costs[self.start] = 0
        self.costs[self.goal] = math.inf
        self.parent[self.start] = self.start
        heapq.heappush(self.open, (0, self.start))
        goal_found = False

        while self.open:
            _, node = heapq.heappop(self.open)
            self.closed.append(node)

            if node == self.goal:
                goal_found = True
                break

            for neighbor in self.get_neighbor(node):
                new_cost = self.costs[node] + self.get_cost(node, neighbor)

                if neighbor not in self.costs:
                    self.costs[neighbor] = math.inf

                if new_cost < self.costs[neighbor]:
                    self.costs[neighbor] = new_cost
                    self.parent[neighbor] = node
                    self.last_parent = node

                    heapq.heappush(self.open, (new_cost, neighbor))

        if not goal_found:
            print("No Goal Found")
        else:
            print("Wohoooo, Goal Found")

        print(self.start, self.goal)

        return self.extract_path(), self.closed

    def extract_path(self):
        s = None
        if self.goal not in self.parent:
            s = self.last_parent
        else:
            s = self.goal
        path = [s]

        while True:
            s = self.parent[s]
            path.append(s)

            if s == self.start:
                break
        return path

    def plot_visited(self, visited):
        if self.start in visited:
            visited.remove(self.start)

        if self.goal in visited:
            visited.remove(self.goal)

        for p in self.closed:
            plt.plot(p[0], p[1], marker='o', color="gray")
            # plt.plot(p[0], p[1], "mx")

    # def plot_visited(self, visited, cl='gray'):
    #     if self.start in visited:
    #         visited.remove(self.start)
    #
    #     if self.goal in visited:
    #         visited.remove(self.goal)
    #
    #     count = 0
    #
    #     for x in visited:
    #         count += 1
    #         plt.plot(x[0], x[1], color=cl, marker='o')
    #         plt.gcf().canvas.mpl_connect('key_release_event',
    #                                      lambda event: [exit(0) if event.key == 'escape' else None])
    #
    #         if count < len(visited) / 3:
    #             length = 20
    #         elif count < len(visited) * 2 / 3:
    #             length = 30
    #         else:
    #             length = 40
    #         #
    #         # length = 15
    #
    #         if count % length == 0:
    #             plt.pause(0.00001)
    #     plt.pause(0.01)

    def plot_path(self, path):
        if path is not None:
            p_x = [p[0] for p in path]
            p_y = [p[1] for p in path]
            plt.plot(p_x, p_y, color='r', linewidth=3)


def main():
    # res = 0.05
    # corners = [-0.8, 0.3, 0, 0.6]
    # start = (0.25, 0.1)
    # goal = (-0.65, 0.6)
    # obstacles = [(-0.1, 0.1), (-0.5, 0.3), (-0.6, 0.3), (-0.1, 0.2), (0.2, 0.3), (0, 0.4), (-0.1, 0.3), (-0.1, 0.4),
    #              (0.15, 0.15), (-0.65, 0.25)]
    # # change self.dec

    res = 1
    corners = [0, 51, 0, 31]
    start = (5, 5)
    goal = (45, 25)
    obstacles = []
    for i in range(10, 21):
        obstacles.append((i, 15))
    for i in range(15):
        obstacles.append((20, i))

    for i in range(15, 30):
        obstacles.append((30, i))
    for i in range(16):
        obstacles.append((40, i))

    ws = WorkSpace(corners, start, goal, res)
    ws.create_workspace()
    ws.add_obstacles(obstacles)

    centroid = ws.calculate_centroid()
    plt.plot(centroid[0], centroid[1], "ro")

    djikstra = Djikstra(start, goal, obstacles, corners, res)
    path, visited = djikstra.search()
    djikstra.plot_visited(visited)
    djikstra.plot_path(path)
    plt.show()


if __name__ == '__main__':
    main()
