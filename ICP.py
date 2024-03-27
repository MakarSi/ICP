import copy

import numpy as np

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

    def _find_rotation_matrix(self, source: PointCloud, target: PointCloud) -> np.matrix:
        """
        Source and target have the same number of points.
        Source points have to be in accordance with target ones.

        source: [s1, s2, s3]
                 ^    ^   ^
                 |    |   |
        target: [t1, t2, t3]
        """

        pass

    def iterative_closest_point(self, source: PointCloud, target: PointCloud) -> PointCloud:
        # TODO: косяк со сдвигами
        temp_source = copy.deepcopy(source)
        temp_source += target.mass_center - source.mass_center

        # (dist, target_point, source_point)
        correspondence = [(*self.kd_tree.find_closest(p), p) for p in temp_source]
        new_correspondence = []
        correspondence.sort(key=lambda el: el[0])
        for el in correspondence:
            if not [n_el for n_el in new_correspondence if n_el[0] == el[1]]:
                new_correspondence.append((el[1], el[2]))

        new_source = PointCloud([p[0] for p in new_correspondence])
        new_source -= new_source.mass_center
        new_target = PointCloud([p[1] for p in new_correspondence])
        new_target -= new_target.mass_center

        rotation_matrix = self._find_rotation_matrix(new_source, new_target)

        temp_source -= temp_source.mass_center
        temp_source.rotate(rotation_matrix)
        temp_source += source.mass_center

        return temp_source


source = PointCloud([Point(-2, -2+5, 0), Point(-1, -1+5, 0), Point(0, 0+5, 0), Point(1, 1+5, 0), Point(2, 2+5, 0)])

target = PointCloud([Point(-2, -4, 0), Point(-1, -2, 0), Point(0, 0, 0), Point(1, 2, 0), Point(2, 4, 0)])

ICP(source, target).iterative_closest_point(source, target)
