import numpy as np
from typing import List, Callable
from mpl_toolkits.mplot3d import Axes3D
from Point3D import Point3D
from Drawable import Drawable
from src.Point import PointContainer


class PointCloud(PointContainer, Drawable):
    """
    Облако точек в трехмерном пространстве.
    """

    def __init__(self, points: List[Point3D]):
        super().__init__(points)

    @property
    def mass_center(self) -> Point3D:
        return sum(self._points, Point3D()) / len(self._points)

    @property
    def length(self) -> int:
        return len(self._points)

    @classmethod
    def point_class(cls):
        return Point3D

    def sort(self, key: Callable = lambda point: (point.x, point.y, point.z)):
        """
        Сортировка коллекции по ключу.

        Args:
            key (Callable): метод сравнения точек.
        """
        self._points = sorted(self._points, key=key)

    def get_matrix(self, by_rows: bool = True) -> np.array:
        """
        Получить матричное представление.

        Args:
            by_rows (bool): записывать точки в строки матрицы, иначе в столбцы.

        Returns:
            np.array: матрица точек.
        """
        matrix = np.array([[p.x, p.y, p.z] for p in self])
        return matrix if by_rows else matrix.T

    def rotate(self, rotation_matrix: np.array):
        """
        Повернуть объект.

        Args:
            rotation_matrix (np.array): матрица поворота.
        """
        for p in self._points:
            p.rotate(rotation_matrix)

    def draw(self, ax: Axes3D, color: str):
        """
        Отрисовать объект в matplotlib.

        Args:
            ax (Axes3D): оси matplotlib.
            color (str): цвет объекта.
        """
        points_array = self.get_matrix().T
        ax.scatter3D(*points_array, color=color)

    def __mul__(self, other: int | float):
        if type(other) not in [int, float]:
            raise Exception(f"other type {type(other)} is not {int} or {float}")
        return PointCloud([point * other for point in self._points])

    def __rmul__(self, other: int | float):
        if type(other) not in [int, float]:
            raise Exception(f"other type {type(other)} is not {int} or {float}")
        return PointCloud([point * other for point in self._points])

    def __add__(self, other: Point3D):
        if type(other) is not Point3D:
            raise Exception(f"other type {type(other)} is not {Point3D}")
        return PointCloud([point + other for point in self._points])

    def __sub__(self, other: Point3D):
        if type(other) is not Point3D:
            raise Exception(f"other type {type(other)} is not {Point3D}")
        return PointCloud([point - other for point in self._points])

    def __iter__(self):
        for each in self._points:
            yield each

    def __getitem__(self, item):
        return self._points[item]

    def __len__(self):
        return len(self._points)

    def __str__(self):
        return "[" + ", ".join(str(point) for point in self._points) + "]"

    def __repr__(self):
        return "[" + ", ".join(point.__repr__() for point in self._points) + "]"
