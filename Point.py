import numpy as np


class Point:
    def __init__(self, x: float = 0.0, y: float = 0.0, z: float = 0.0):
        self._x = x
        self._y = y
        self._z = z

    def rotate(self, rotation_matrix: np.matrix):
        result = np.matmul(rotation_matrix, np.array([self._x, self._y, self._z]))
        self._x, self._y, self._z = result.item(0), result.item(1), result.item(2)

    def __add__(self, other: 'Point'):
        if type(other) is not self.__class__:
            raise Exception(f"other type {type(other)} is not {self.__class__}")
        return Point(self._x + other.x, self._y + other.y, self._z + other.z)

    def __sub__(self, other: 'Point'):
        if type(other) is not self.__class__:
            raise Exception(f"other type {type(other)} is not {self.__class__}")
        return Point(self._x - other.x, self._y - other.y, self._z - other.z)

    def __mul__(self, other: int | float):
        if type(other) not in [int, float]:
            raise Exception(f"other type {type(other)} is not {int} or {float}")
        return Point(self._x * other, self._y * other, self._z * other)

    def __rmul__(self, other: int | float):
        if type(other) not in [int, float]:
            raise Exception(f"other type {type(other)} is not {int} or {float}")
        return Point(self._x * other, self._y * other, self._z * other)

    def __truediv__(self, other: int | float):
        if type(other) not in [int, float]:
            raise Exception(f"other type {type(other)} is not {int} or {float}")
        if other == 0:
            raise Exception("Division by zero")
        return Point(self._x / other, self._y / other, self._z / other)

    def __eq__(self, other):
        return self._x == other.x and self._y == other.y and self._z == other.z

    def __str__(self):
        return f"({self._x}, {self._y}, {self._z})"

    @property
    def x(self):
        return self._x

    @property
    def y(self):
        return self._y

    @property
    def z(self):
        return self._z

    def draw(self, ax, color):
        ax.scatter3D(self._x, self._y, self._z, color=color, s=0.1)
