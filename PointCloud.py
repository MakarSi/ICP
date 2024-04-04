import numpy as np
import random
import copy

from typing import List

from Point import Point


class PointCloud:
    def __init__(self, points: List[Point]):
        self._points = points

    def rotate(self, rotation_matrix: np.array):
        for p in self._points:
            p.rotate(rotation_matrix)

    def shift(self, shifting: np.array):
        shifting_point = Point(shifting[0], shifting[1], shifting[2])
        for i in range(len(self._points)):
            self._points[i] = self._points[i] + shifting_point

    # np.random.permutation ?
    def do_perturbation(self) -> 'PointCloud':
        shift = np.array([random.uniform(-1, 1), random.uniform(-1, 1), random.uniform(-1, 1)])
        axis = np.array([random.uniform(-3, 3), random.uniform(-3, 3), random.uniform(-3, 3)])
        angle = np.random.uniform(0, np.pi / 5)

        rotation_matrix = get_rotation_matrix(axis, angle)

        temp_cloud = copy.deepcopy(self)

        temp_cloud.rotate(rotation_matrix)
        temp_cloud.shift(shift)

        return temp_cloud

    @property
    def mass_center(self) -> Point:
        return sum(self._points, Point()) / len(self._points)

    def get_matrix(self, by_rows: bool = True) -> np.array:
        to_return = []
        for p in self:
            to_return.append([p.x, p.y, p.z])

        return np.array(to_return) if by_rows else np.array(to_return).T

    def __mul__(self, other: int | float):
        if type(other) not in [int, float]:
            raise Exception(f"other type {type(other)} is not {int} or {float}")
        return PointCloud([point * other for point in self._points])

    def __rmul__(self, other: int | float):
        if type(other) not in [int, float]:
            raise Exception(f"other type {type(other)} is not {int} or {float}")
        return PointCloud([point * other for point in self._points])

    def __add__(self, other: Point):
        if type(other) is not Point:
            raise Exception(f"other type {type(other)} is not {Point}")
        return PointCloud([point + other for point in self._points])

    def __sub__(self, other: Point):
        if type(other) is not Point:
            raise Exception(f"other type {type(other)} is not {Point}")
        return PointCloud([point - other for point in self._points])

    def __iter__(self):
        for each in self._points:
            yield each

    def __str__(self):
        return "[" + ", ".join(str(point) for point in self._points) + "]"

    def length(self):
        return len(self._points)

    def draw(self, ax, color):
        for point in self._points:
            point.draw(ax, color)


def get_rotation_matrix(axis, angle: float):
    axis /= np.linalg.norm(axis)
    x, y, z = axis[0], axis[1], axis[2]
    cosa, sina = np.cos(angle), np.sin(angle)
    rotation_matrix = np.zeros((3, 3))
    rotation_matrix[0][0] = cosa + (1 - cosa) * x * x
    rotation_matrix[0][1] = (1 - cosa) * x * y - sina * z
    rotation_matrix[0][2] = (1 - cosa) * x * z + sina * y
    rotation_matrix[1][0] = (1 - cosa) * y * x + sina * z
    rotation_matrix[1][1] = cosa + (1 - cosa) * y * y
    rotation_matrix[1][2] = (1 - cosa) * y * z - sina * x
    rotation_matrix[2][0] = (1 - cosa) * z * x - sina * y
    rotation_matrix[2][1] = (1 - cosa) * z * y + sina * x
    rotation_matrix[2][2] = cosa + (1 - cosa) * z * z
    return rotation_matrix
