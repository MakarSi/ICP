from typing import Optional
import numpy as np
from matplotlib import pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import KDTree as kdt
from PointCloud import PointCloud


class ICP:
    """
    Реализует алгоритм Iterative Closest Point.
    """

    def __init__(self, source: PointCloud, target: PointCloud, penalty_bound: float = 1e-18, max_iter: int = 20):
        """
        Инициализация класса, реализующего алгоритм ICP.
        Смещает облако точек source к облаку точек target.

        Args:
            source (PointCloud): смещаемое облако точек.
            target (PointCloud): облако точек, к которому смещаем.
            penalty_bound (float): значение штрафное функции, при котором оканчиваем алгоритм.
            max_iter (int): максимальное число итераций алгоритма.
        """
        self._source = source
        self._target = target
        self._penalty_bound = penalty_bound
        self._max_iter = max_iter

        self._kd_tree = kdt.KDTree(PointCloud)
        self._kd_tree.build(target)

    @property
    def source(self):
        return self._source

    @property
    def target(self):
        return self._target

    @property
    def calculate_penalty(self) -> float:
        """
        Штрафная функция.

        Returns:
            float: квадратичное отклонение.
        """
        penalty = 0
        for p in self._source:
            dist, _ = self._kd_tree.find_closest(p)
            penalty += dist ** 2
        return penalty / self._source.length

    @staticmethod
    def find_rotation_matrix(source: PointCloud, target: PointCloud) -> np.array:
        """
        Найти матрицу поворота для перехода от облака точек source к target.
        source: [s1, s2, s3]
                 ^    ^   ^
                 |    |   |
        target: [t1, t2, t3]

        Args:
            source (PointCloud): смещаемое облако точек.
            target (PointCloud): облако точек, к которому смещаем.

        Raises:
            ValueError: если размеры облаков точек не совпадают.

        Returns:
            np.array: матрица поворота.
        """
        if source.length != target.length:
            raise ValueError("Point clouds length is not the same")

        source_matrix = source.get_matrix(by_rows=False)
        target_matrix = target.get_matrix(by_rows=True)

        s = np.matmul(source_matrix, target_matrix)
        u, _, vt = np.linalg.svd(s)
        return np.matmul(vt.T, u.T)

    def icp_step(self) -> PointCloud:
        """
        Шаг алгоритма.

        Returns:
            PointCloud: смещенное облако точек.
        """

        # Найти ближайших соседей
        correspondence = [(*self._kd_tree.find_closest(p), p) for p in self._source]
        neighbours = PointCloud([p[1] for p in correspondence])

        # Сдвиг
        self._source += neighbours.mass_center - self._source.mass_center

        # Поворот
        rotation_matrix = self.find_rotation_matrix(self._source, neighbours)
        self._source.rotate(rotation_matrix)

        return self._source

    def icp_algorithm(self, debug: bool = False, ax: Optional[Axes3D] = None, pause_time: float = 0.1):
        """
        Запуск алгоритма.

        Args:
            debug (bool): дебаг.
            ax (Optional[Axes3D]): matplotlib-оси, если None, то не рисуем.
            pause_time (float): время задержки между отрисовкой шагов.
        """
        for i in range(self._max_iter):
            penalty = self.calculate_penalty
            if penalty < self._penalty_bound:
                break
            if debug:
                print(penalty)

            # Шаг алгоритма
            self.icp_step()

            # Отрисовка
            if ax:
                self._source.draw(ax, 'red')
                self._target.draw(ax, 'green')
                plt.pause(pause_time)
                if i + 1 != self._max_iter:
                    ax.cla()
