from typing import List
import numpy as np

from Point import Point


class PointCloud:
    def __init__(self, points: List[Point]):
        self._points = points

    def rotate(self, rotation_matrix: np.matrix):
        for p in self._points:
            p.rotate(rotation_matrix)

    @property
    def mass_center(self) -> Point:
        return sum(self._points, Point()) / len(self._points)

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
