import math
import sys
from typing import Tuple

import numpy as np
import random as rd

from Point import Point
from PointCloud import PointCloud


class Node:
    """
    Узел K-d дерева.
    """

    def __init__(self):
        """
        Инициализация узла K-d дерева, как boundary box.
        """
        self.min_x = self.min_y = self.min_z = sys.maxsize
        self.max_x = self.max_y = self.max_z = -sys.maxsize

        self.x_list = [self.min_x, self.max_x]
        self.y_list = [self.min_y, self.max_y]
        self.z_list = [self.min_z, self.max_z]

    def update(self, point: Point):
        """
        Обновить boundary_box.
        :param point:
        :return:
        """
        self.min_x = min(self.min_x, point.x)
        self.max_x = max(self.max_x, point.x)
        self.min_y = min(self.min_y, point.y)
        self.max_y = max(self.max_y, point.y)
        self.min_z = min(self.min_z, point.z)
        self.max_z = max(self.max_z, point.z)

        self.x_list = [self.min_x, self.max_x]
        self.y_list = [self.min_y, self.max_y]
        self.z_list = [self.min_z, self.max_z]

    def distance(self, point: Point) -> float:

        # TODO: как-то слишком сложно, можно проще
        if (self.min_x <= point.x <= self.max_x
                and self.min_y <= point.y <= self.max_y
                and self.min_z <= point.z <= self.max_z):
            return 0

        res = min(point.distance(Point(x, y, z)) for x in self.x_list for y in self.y_list for z in self.z_list)

        if self.min_x <= point.x <= self.max_x and self.min_z <= point.z <= self.max_z:
            res = min(res, abs(self.min_y - point.y), abs(self.max_y - point.y))

        if self.min_y <= point.y <= self.max_y and self.min_z <= point.z <= self.max_z:
            res = min(res, abs(self.min_x - point.x), abs(self.max_x - point.x))

        if self.min_y <= point.y <= self.max_y and self.min_x <= point.x <= self.max_x:
            res = min(res, abs(self.min_z - point.z), abs(self.max_z - point.x))
        return res


class KDTree:
    """
    K-d дерево.
    """

    def __init__(self):
        self.tree = [Node()] * 2
        self.l_tr, self.r_tr = [-1] * 2, [-1] * 2

    def build(self, cloud: PointCloud):
        """
        Построить K-d дерево по облаку точек.

        Args:
            cloud (PointCloud): облако точек.
        """
        self._build(cloud, 1, 0)

    def find_closest(self, point: Point) -> Tuple[float, Point]:
        """
        Найти ближайшего соседа к точке.

        Args:
            point (Point): точка.

        Returns:
             Tuple[float, Point] - (расстояние до ближайшей точки, ближайшая точка).
        """
        pnt = [point.x, point.y, point.z]
        min_dist = [sys.maxsize, 0]
        neib = [[0, 0, 0]]
        self._find_closest(point, min_dist, neib)
        return min_dist[0], Point(neib[0][0], neib[0][1], neib[0][2])

    def _build(self, point_cloud: PointCloud, tr_num, turn):
        for point in point_cloud:
            self.tree[tr_num].update(point)

        if point_cloud.length == 1:
            return

        if turn == 1:
            for p in point_cloud:
                p._x, p._y = p.y, p.x

        elif turn == 2:
            for p in point_cloud:
                p._x, p._z = p.z, p.x

        point_cloud.sort()

        if turn == 1:
            for p in point_cloud:
                p._x, p._y = p.y, p.x

        elif turn == 2:
            for p in point_cloud:
                p._x, p._z = p.z, p.x

        mid, left_points, right_points = point_cloud.length // 2, [], []
        for i in range(mid):
            left_points.append(point_cloud[i])
        for i in range(mid, point_cloud.length):
            right_points.append(point_cloud[i])

        if len(left_points) > 0:
            self.tree.append(Node())
            self.l_tr.append(-1)
            self.r_tr.append(-1)
            self.l_tr[tr_num] = len(self.tree) - 1
            self._build(PointCloud(left_points), self.l_tr[tr_num], (turn + 1) % 3)
        if len(right_points) > 0:
            self.tree.append(Node())
            self.l_tr.append(-1)
            self.r_tr.append(-1)
            self.r_tr[tr_num] = len(self.tree) - 1
            self._build(PointCloud(right_points), self.r_tr[tr_num], (turn + 1) % 3)

    def _find_closest(self, pnt, min_dist, res, tr_num=1):
        min_dist[1] += 1
        if self.l_tr[tr_num] == -1:
            min_dist[0] = self.tree[tr_num].distance(pnt)
            res[0] = [self.tree[tr_num].min_x, self.tree[tr_num].min_y, self.tree[tr_num].min_z]
            return
        if rd.randint(0, 1) == 0:
            if self.tree[self.l_tr[tr_num]].distance(pnt) < min_dist[0]:
                self._find_closest(pnt, min_dist, res, self.l_tr[tr_num])
            if self.tree[self.r_tr[tr_num]].distance(pnt) < min_dist[0]:
                self._find_closest(pnt, min_dist, res, self.r_tr[tr_num])
        else:
            if self.tree[self.r_tr[tr_num]].distance(pnt) < min_dist[0]:
                self._find_closest(pnt, min_dist, res, self.r_tr[tr_num])
            if self.tree[self.l_tr[tr_num]].distance(pnt) < min_dist[0]:
                self._find_closest(pnt, min_dist, res, self.l_tr[tr_num])
