from abc import ABCMeta
from mpl_toolkits.mplot3d import Axes3D


class Drawable(metaclass=ABCMeta):

    def draw(self, ax: Axes3D, color: str):
        """
        Отрисовать объект в matplotlib.

        Args:
            ax (Axes3D): оси matplotlib.
            color (str): цвет объекта.
        """
        raise NotImplementedError
