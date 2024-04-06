import itertools

from recordtype import recordtype
from typing import List, Tuple
from src.Metric import Metric, MetricContainer
from src.Point3D import Point3D
from src.PointCloud import PointCloud

MinMax = recordtype('MinMax', 'min max')


class BoundaryBox:
    def __init__(self, cls, inf: float = 10e8):
        if cls.dimension < 1:
            raise ValueError(f"Dimension {cls.dimension} have to be >= 1")
        self._dimension = cls.dimension
        self._bounds = [MinMax(inf, -inf) for _ in range(self._dimension)]
        self._cls = cls

    @property
    def vertexes(self) -> List[Metric]:
        lst = [[self._bounds[j].min if tool[j] else self._bounds[j].max for j in range(self._dimension)] for tool
               in itertools.product([0, 1], repeat=self._dimension)]
        return [self._cls(*obj) for obj in lst]

    def update(self, point: Metric):
        if type(point) != self._cls:
            raise AttributeError(f"Point have to be {self._cls}")

        for i, coordinate in enumerate(point):
            self._bounds[i].min = min(self._bounds[i].min, coordinate)
            self._bounds[i].max = max(self._bounds[i].max, coordinate)

    def distance(self, point: Metric) -> float:
        if type(point) != self._cls:
            raise AttributeError(f"Point have to be {self._cls}")

        # If point inside box then distance is 0
        if all(self._bounds[i].min <= coord <= self._bounds[i].max for i, coord in enumerate(point)):
            return 0

        # Find min distance to vertexes
        res = min(point.distance(vertex) for vertex in self.vertexes)

        # Find min distance to faces
        for i in range(self._dimension):
            if all(self._bounds[j].min <= coord <= self._bounds[j].max for j, coord in enumerate(point) if i != j):
                res = min(res, abs(self._bounds[i].min - point[i]), abs(self._bounds[i].max - point[i]))

        return res


class KDTree:
    """
    K-d дерево.
    """

    def __init__(self, cls):
        self._container_cls = cls
        self._cls = cls.metric_type
        self.tree = [BoundaryBox(self._cls)] * 2
        self.l_tr, self.r_tr = [-1] * 2, [-1] * 2

    def build(self, cloud: MetricContainer):
        """
        Построить K-d дерево по облаку точек.

        Args:
            cloud (PointCloud): облако точек.
        """
        if cloud.metric_type != self._cls:

        self._build(cloud, 1, 0)

    def _build(self, point_cloud: MetricContainer, tr_num, turn):
        if point_cloud.metric_type != self._cls:
            raise AttributeError(f"Container have to contain {self._cls} objects")

        for point in point_cloud:
            self.tree[tr_num].update(point)

        if len(point_cloud) == 1:
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

        mid, left_points, right_points = len(point_cloud) // 2, [], []
        for i in range(mid):
            left_points.append(point_cloud[i])
        for i in range(mid, len(point_cloud)):
            right_points.append(point_cloud[i])

        if len(left_points) > 0:
            self.tree.append(BoundaryBox(self._cls))
            self.l_tr.append(-1)
            self.r_tr.append(-1)
            self.l_tr[tr_num] = len(self.tree) - 1
            self._build(PointCloud(left_points), self.l_tr[tr_num], (turn + 1) % 3)
        if len(right_points) > 0:
            self.tree.append(BoundaryBox(self._cls))
            self.l_tr.append(-1)
            self.r_tr.append(-1)
            self.r_tr[tr_num] = len(self.tree) - 1
            self._build(PointCloud(right_points), self.r_tr[tr_num], (turn + 1) % 3)

    def find_closest(self, point: Metric) -> Tuple[float, Metric]:
        """
        Найти ближайшего соседа к точке.

        Args:
            point (Point3D): точка.

        Returns:
             Tuple[float, Point] - (расстояние до ближайшей точки, ближайшая точка).
        """
        if type(point) != self._cls:
            raise AttributeError(f"Point have to be {self._cls}")
        pnt = [point.x, point.y, point.z]
        min_dist = [sys.maxsize, 0]
        neib = [[0, 0, 0]]
        self._find_closest(point, min_dist, neib)
        return min_dist[0], Point3D(neib[0][0], neib[0][1], neib[0][2])

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
