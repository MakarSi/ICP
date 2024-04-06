import itertools
from recordtype import recordtype
from typing import List, Tuple
from Metric import Metric, MetricContainer

MinMax = recordtype('MinMax', 'min max')


# TODO: рефакторинг и переезд на python3.12, проблемы с typehint и архитектурой

class BoundaryBox:
    def __init__(self, cls, inf: float = 10e8):
        if cls.dimension() < 1:
            raise ValueError(f"Dimension {cls.dimension()} have to be >= 1")
        self._dimension = cls.dimension()
        self._bounds = [MinMax(inf, -inf) for _ in range(self._dimension)]
        self._cls = cls

    @property
    def dimension(self):
        return self._dimension

    @property
    def bounds(self):
        return self._bounds

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

    def __init__(self, cls, inf=10e8):
        self._container_cls = cls
        self._cls = cls.metric_type()
        self._dimension = cls.metric_type().dimension()
        self._inf = inf
        self.tree = [BoundaryBox(self._cls, self._inf)] * 2
        self.l_tr, self.r_tr = [-1] * 2, [-1] * 2

    def _add_node(self):
        self.tree.append(BoundaryBox(self._cls, self._inf))
        self.l_tr.append(-1)
        self.r_tr.append(-1)

    def build(self, point_cloud: MetricContainer, tr_num=1, coordinate=0):
        if point_cloud.metric_type() != self._cls:
            raise AttributeError(f"Container have to contain {self._cls} objects")

        for point in point_cloud:
            self.tree[tr_num].update(point)
        if len(point_cloud) <= 1:
            return

        point_cloud.sort(key=lambda p: (p[coordinate]))

        # Initialize left sub-tree
        self._add_node()
        self.l_tr[tr_num] = len(self.tree) - 1
        self.build(self._container_cls(point_cloud[len(point_cloud) // 2:]), self.l_tr[tr_num],
                   (coordinate + 1) % self._dimension)

        # Initialize right sub-tree
        self._add_node()
        self.r_tr[tr_num] = len(self.tree) - 1
        self.build(self._container_cls(point_cloud[:len(point_cloud) // 2]), self.r_tr[tr_num],
                   (coordinate + 1) % self._dimension)

    def find_closest(self, point: Metric) -> Tuple[float, Metric]:
        """
        Найти ближайшего соседа к точке.

        Args:
            point (Point3D): точка.

        Returns:
             Tuple[float, Point] - (расстояние до ближайшей точки, ближайшая точка).
        """

        def _find_closest(tr_num=1):
            nonlocal min_dist, res
            if self.l_tr[tr_num] == -1:
                min_dist = self.tree[tr_num].distance(point)
                res = self._cls(*[self.tree[tr_num].bounds[i].min for i in range(self._dimension)])
                return
            if self.tree[self.l_tr[tr_num]].distance(point) < min_dist:
                _find_closest(self.l_tr[tr_num])
            if self.tree[self.r_tr[tr_num]].distance(point) < min_dist:
                _find_closest(self.r_tr[tr_num])

        if type(point) != self._cls:
            raise AttributeError(f"Point have to be {self._cls}")
        min_dist = self._inf
        res = self._cls()
        _find_closest()
        return min_dist, res
