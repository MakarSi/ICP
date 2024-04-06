import numpy as np
from mpl_toolkits.mplot3d import Axes3D

from Point import Point
from Drawable import Drawable


class Point3D(Point, Drawable):
    """
    Точка в трехмерном пространстве.
    """

    def __init__(self, x: float = 0.0, y: float = 0.0, z: float = 0.0):
        super().__init__([x, y, z])

    @classmethod
    def dimension(cls):
        return 3

    @property
    def x(self):
        return self._coordinates[0]

    @property
    def y(self):
        return self._coordinates[1]

    @property
    def z(self):
        return self._coordinates[2]

    def distance(self, other: 'Point3D') -> float:
        """
        Евклидово расстояние между точками.

        Args:
            other (Point3D): другая точка

        Returns:
            float: расстояние
        """
        return np.sqrt(
            (other.x - self.x) * (other.x - self.x) + (other.y - self.y) * (other.y - self.y) + (other.z - self.z) * (
                    other.z - self.z))

    def rotate(self, rotation_matrix: np.array):
        """
        Повернуть объект.

        Args:
            rotation_matrix (np.array): матрица поворота.
        """
        result = np.matmul(rotation_matrix, np.array([self.x, self.y, self.z]))
        self._coordinates[0], self._coordinates[1], self._coordinates[2] = result.item(0), result.item(1), result.item(
            2)

    def draw(self, ax: Axes3D, color: str, size: float = 0.1):
        """
        Отрисовать объект в matplotlib.

        Args:
            ax (Axes3D): оси matplotlib.
            color (str): цвет объекта.
            size (float): толщина.
        """
        ax.scatter3D(self.x, self.y, self.z, color=color, s=size)

    def __add__(self, other: 'Point3D'):
        if type(other) is not self.__class__:
            raise Exception(f"other type {type(other)} is not {self.__class__}")
        return Point3D(self.x + other.x, self.y + other.y, self.z + other.z)

    def __sub__(self, other: 'Point3D'):
        if type(other) is not self.__class__:
            raise Exception(f"other type {type(other)} is not {self.__class__}")
        return Point3D(self.x - other.x, self.y - other.y, self.z - other.z)

    def __mul__(self, other: int | float):
        if type(other) not in [int, float]:
            raise Exception(f"other type {type(other)} is not {int} or {float}")
        return Point3D(self.x * other, self.y * other, self.z * other)

    def __rmul__(self, other: int | float):
        if type(other) not in [int, float]:
            raise Exception(f"other type {type(other)} is not {int} or {float}")
        return Point3D(self.x * other, self.y * other, self.z * other)

    def __truediv__(self, other: int | float):
        if type(other) not in [int, float]:
            raise Exception(f"other type {type(other)} is not {int} or {float}")
        if other == 0:
            raise Exception("Division by zero")
        return Point3D(self.x / other, self.y / other, self.z / other)

    def __eq__(self, other):
        return self.x == other.x and self.y == other.y and self.z == other.z

    def __iter__(self):
        for each in self._coordinates:
            yield each

    def __getitem__(self, item):
        return self._coordinates[item]

    def __str__(self):
        return f"(x: {self.x}, y: {self.y}, z: {self.z})"

    def __repr__(self):
        return f"({self.x}, {self.y}, {self.z})"
