from abc import ABCMeta
import numpy as np

from mpl_toolkits.mplot3d import Axes3D


class Shape(metaclass=ABCMeta):
    """
    Интерфейс для геометрических объектов.
    """

    def rotate(self, rotation_matrix: np.array):
        """
        Повернуть объект.

        Args:
            rotation_matrix (np.array): матрица поворота.
        """
        raise NotImplementedError

    def draw(self, ax: Axes3D, color: str):
        """
        Отрисовать объект в matplotlib.

        Args:
            ax (Axes3D): оси matplotlib.
            color (str): цвет объекта.
        """
        raise NotImplementedError
