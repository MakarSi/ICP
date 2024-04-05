from typing import Tuple

import numpy as np
import random as rd

from Point import Point
from PointCloud import PointCloud

inf = 1e14


class Node:
    def __init__(self):
        self.min_x = self.min_y = self.min_z = inf
        self.max_x = self.max_y = self.max_z = -inf

    def upd(self, pnt):
        self.min_x = min(self.min_x, pnt[0])
        self.max_x = max(self.max_x, pnt[0])
        self.min_y = min(self.min_y, pnt[1])
        self.max_y = max(self.max_y, pnt[1])
        self.min_z = min(self.min_z, pnt[2])
        self.max_z = max(self.max_z, pnt[2])


def p2p_dist(p1, p2):
    dx, dy, dz = p1[0] - p2[0], p1[1] - p2[1], p1[2] - p2[2]
    return np.sqrt(dx * dx + dy * dy + dz * dz)


def p2nd_dist(p, nd: Node):
    if nd.min_x <= p[0] <= nd.max_x and nd.min_y <= p[1] <= nd.max_y and nd.min_z <= p[2] <= nd.max_z:
        return 0
    mn = min(p2p_dist(p, [nd.max_x, nd.max_y, nd.min_z]), p2p_dist(p, [nd.min_x, nd.max_y, nd.min_z]),
             p2p_dist(p, [nd.max_x, nd.min_y, nd.min_z]), p2p_dist(p, [nd.min_x, nd.min_y, nd.min_z]),
             p2p_dist(p, [nd.max_x, nd.max_y, nd.max_z]), p2p_dist(p, [nd.min_x, nd.max_y, nd.max_z]),
             p2p_dist(p, [nd.max_x, nd.min_y, nd.max_z]), p2p_dist(p, [nd.min_x, nd.min_y, nd.max_z]))
    if nd.min_x <= p[0] <= nd.max_x and nd.min_z <= p[2] <= nd.max_z:
        mn = min(mn, abs(nd.min_y - p[1]), abs(nd.max_y - p[1]))
    if nd.min_y <= p[1] <= nd.max_y and nd.min_z <= p[2] <= nd.max_z:
        mn = min(mn, abs(nd.min_x - p[0]), abs(nd.max_x - p[0]))
    if nd.min_y <= p[1] <= nd.max_y and nd.min_x <= p[0] <= nd.max_x:
        mn = min(mn, abs(nd.min_z - p[2]), abs(nd.max_z - p[2]))
    return mn


class KDTree:
    """
    K-d дерево.
    """

    def __init__(self):
        self.tr = [Node()] * 2
        self.l_tr, self.r_tr = [-1] * 2, [-1] * 2

    def build(self, cloud: PointCloud):
        """
        Построить K-d дерево по облаку точек.

        Args:
            cloud (PointCloud): облако точек.
        """
        points = [[p.x, p.y, p.z] for p in cloud]
        self._build(points, 1, 0)

    def find_closest(self, point: Point) -> Tuple[float, Point]:
        """
        Найти ближайшего соседа к точке.

        Args:
            point (Point): точка.

        Returns:
             Tuple[float, Point] - (расстояние до ближайшей точки, ближайшая точка).
        """
        pnt = [point.x, point.y, point.z]
        min_dist = [inf, 0]
        neib = [[0, 0, 0]]
        self._find_closest(pnt, min_dist, neib)
        return min_dist[0], Point(neib[0][0], neib[0][1], neib[0][2])

    def _build(self, pnts, tr_num, turn):
        n = len(pnts)
        for p in pnts:
            self.tr[tr_num].upd(p)
        if n == 1:
            return
        if turn == 1:
            for i in range(n):
                pnts[i][0], pnts[i][1] = pnts[i][1], pnts[i][0]
        elif turn == 2:
            for i in range(n):
                pnts[i][0], pnts[i][2] = pnts[i][2], pnts[i][0]
        pnts.sort()
        if turn == 1:
            for i in range(n):
                pnts[i][0], pnts[i][1] = pnts[i][1], pnts[i][0]
        elif turn == 2:
            for i in range(n):
                pnts[i][0], pnts[i][2] = pnts[i][2], pnts[i][0]
        mid, l_pnts, r_pnts = n // 2, [], []
        for i in range(mid):
            l_pnts.append(pnts[i])
        for i in range(mid, n):
            r_pnts.append(pnts[i])
        if len(l_pnts) > 0:
            self.tr.append(Node())
            self.l_tr.append(-1)
            self.r_tr.append(-1)
            self.l_tr[tr_num] = len(self.tr) - 1
            self._build(l_pnts, self.l_tr[tr_num], (turn + 1) % 3)
        if len(r_pnts) > 0:
            self.tr.append(Node())
            self.l_tr.append(-1)
            self.r_tr.append(-1)
            self.r_tr[tr_num] = len(self.tr) - 1
            self._build(r_pnts, self.r_tr[tr_num], (turn + 1) % 3)

    def _find_closest(self, pnt, min_dist, res, tr_num=1):
        min_dist[1] += 1
        if self.l_tr[tr_num] == -1:
            min_dist[0] = p2nd_dist(pnt, self.tr[tr_num])
            res[0] = [self.tr[tr_num].min_x, self.tr[tr_num].min_y, self.tr[tr_num].min_z]
            return
        if rd.randint(0, 1) == 0:
            if p2nd_dist(pnt, self.tr[self.l_tr[tr_num]]) < min_dist[0]:
                self._find_closest(pnt, min_dist, res, self.l_tr[tr_num])
            if p2nd_dist(pnt, self.tr[self.r_tr[tr_num]]) < min_dist[0]:
                self._find_closest(pnt, min_dist, res, self.r_tr[tr_num])
        else:
            if p2nd_dist(pnt, self.tr[self.r_tr[tr_num]]) < min_dist[0]:
                self._find_closest(pnt, min_dist, res, self.r_tr[tr_num])
            if p2nd_dist(pnt, self.tr[self.l_tr[tr_num]]) < min_dist[0]:
                self._find_closest(pnt, min_dist, res, self.l_tr[tr_num])
