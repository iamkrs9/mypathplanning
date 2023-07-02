from abc import ABC
from typing import List


class Obstacle(ABC):
    pass


class Box(Obstacle):
    def __init__(self, size: List[float], pos: List[float]):
        self.size = size
        self.pos = pos

