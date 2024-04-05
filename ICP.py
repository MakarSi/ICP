import copy
import numpy as np
import random

from matplotlib import pyplot as plt

import kdt
from Point import Point

from PointCloud import PointCloud


class ICP:
    def __init__(self, source: PointCloud, target: PointCloud, penalty_bound: float = 1, max_iter: int = 10):
        self.source = source
        self.target = target
        self.penalty_bound = penalty_bound
        self.max_iter = max_iter

        self.kd_tree = kdt.KDTree()
        self.kd_tree.build(target)

    @staticmethod
    def find_rotation_matrix(source: PointCloud, target: PointCloud) -> np.array:
        """
        Source and target have the same number of points.
        Source points have to be in accordance with target ones.

        source: [s1, s2, s3]
                 ^    ^   ^
                 |    |   |
        target: [t1, t2, t3]
        """
        source_matrix = source.get_matrix(by_rows=False)
        target_matrix = target.get_matrix(by_rows=True)

        S = np.matmul(source_matrix, target_matrix)

        U, _, VT = np.linalg.svd(S)

        return np.matmul(VT.T, U.T)

    def iterative_closest_point(self, source) -> PointCloud:
        temp_source = copy.deepcopy(source)

        correspondence = [(*self.kd_tree.find_closest(p), p) for p in temp_source]
        new_source = PointCloud([p[1] for p in correspondence])

        temp_source += new_source.mass_center - temp_source.mass_center

        rotation_matrix = self.find_rotation_matrix(temp_source, new_source)

        temp_source.rotate(rotation_matrix)

        return temp_source

    def ICP_algorithm(self, iterations: int, show_penalty: bool = True):
        temp_source = self.source
        for i in range(iterations):
            if show_penalty:
                penalty = self.calculate_penalty(temp_source)
                print(penalty)
            temp_source = self.iterative_closest_point(temp_source)

    def draw_steps(self, ax, iterations: int, pause_time, show_penalty: bool = True):
        temp_source = self.source
        for i in range(iterations):
            if show_penalty:
                penalty = self.calculate_penalty(temp_source)
                print(penalty)

            temp_source = self.iterative_closest_point(temp_source)

            temp_source.draw(ax, 'red')
            self.target.draw(ax, 'green')

            plt.pause(pause_time)
            if i + 1 != iterations:
                ax.cla()

    def calculate_penalty(self, temp_source: PointCloud):
        penalty = 0
        for p in temp_source:
            dist, _ = self.kd_tree.find_closest(p)
            penalty += dist ** 2
        return penalty / temp_source.length()
