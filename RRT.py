import math
from typing import List, Union, Tuple
import matplotlib.pyplot as plt
import numpy as np

from Workspace import WorkSpaceContinuous

class Node:
    def __init__(self, node):
        self.x = node[0]
        self.y = node[1]
        self.parent = None

class RRT:
    def __init__(self, start, goal, obstacles, corners, step_len, iter):
        self.start = start
        self.goal = goal
        self.obs = obstacles
        self.corners = corners
        self.nodes: List[Node] = [self.start]
        self.step_len = step_len
        self.iter = iter

    def generate_random_node(self) -> Node:
        return Node(
            (np.random.uniform(self.corners[0], self.corners[1]),
             np.random.uniform(self.corners[2], self.corners[3]))
        )

    def get_closest_node(self, node: Node) -> Union[Node, None]:
        dist = math.inf
        res = None
        for n in self.nodes:
            d = math.sqrt(((n.x - node.x)**2) + ((n.y-node.y)**2))
            if d < dist:
                dist = d
                res = n

        return res

    def get_dist_ori(self, random_node: Node, closest_node: Node) -> Tuple[float, float]:
        dx = random_node.x - closest_node.x
        dy = random_node.y - closest_node.y
        dist = math.sqrt(dx ** 2 + dy ** 2)
        ori = math.atan2(dy, dx)

        return dist, ori

    def get_new_node(self, random_node: Node, closest_node: Node) -> Node:
        dist, ori = self.get_dist_ori(random_node, closest_node)

        dist = min(dist, self.step_len)
        new_node = Node(
            (closest_node.x + dist * math.cos(ori),
             (closest_node.y + dist * math.sin(ori)))
        )
        new_node.parent = closest_node

        return new_node

    def is_collision(self, new_node: Node, old_node: Node) -> bool:
        # TODO: Actually check for collisions
        return False

    def extract_path(self, node: Node):
        path = [(self.goal.x, self.goal.y)]
        node_now = node

        while node_now.parent is not None:
            node_now = node_now.parent
            path.append((node_now.x, node_now.y))

        return path

    def search(self):
        for i in range(self.iter):
            random_node = self.generate_random_node()
            closest_node = self.get_closest_node(random_node)
            new_node = self.get_new_node(random_node, closest_node)

            if not self.is_collision(new_node, closest_node):
                self.nodes.append(new_node)
                dist, _ = self.get_dist_ori(new_node, self.goal)

                if dist <= self.step_len and not self.is_collision(new_node, self.goal):
                    self.get_new_node(new_node, self.goal)
                    return self.extract_path(new_node)

        return None

    def plot_visited(self, animate: bool = False) -> None:
        if not animate:
            for node in self.nodes:
                if node.parent:
                    plt.plot([node.parent.x, node.x], [node.parent.y, node.y], "-g")

        else:
            count = 0
            for node in self.nodes:
                count += 1
                if node.parent:
                    plt.plot([node.parent.x, node.x], [node.parent.y, node.y], "-g")
                    plt.gcf().canvas.mpl_connect('key_release_event',
                                                 lambda event:
                                                 [exit(0) if event.key == 'escape' else None])
                    if count % 10 == 0:
                        plt.pause(0.001)

    def plot_path(self, path) -> None:
        plt.plot([x[0] for x in path], [x[1] for x in path], '-r', linewidth=2)
        plt.pause(0.01)


def main():
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

    ws = WorkSpaceContinuous(corners, start, goal, res)
    ws.create_workspace()
    ws.add_obstacles([])

    start = Node((5, 5))
    goal = Node((45, 25))
    obstacles = []
    for i in range(10, 21):
        obstacles.append(Node((i, 15)))
    for i in range(15):
        obstacles.append(Node((20, i)))

    for i in range(15, 30):
        obstacles.append(Node((30, i)))
    for i in range(16):
        obstacles.append(Node((40, i)))

    rrt = RRT(start, goal, obstacles, corners, 0.9, 10000)
    path = rrt.search()
    rrt.plot_visited(animate=True)

    if path:
        rrt.plot_path(path)
    else:
        print(" No Goal Found")

    plt.show()


if __name__ == '__main__':
    main()