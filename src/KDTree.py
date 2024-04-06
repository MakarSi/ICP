import itertools
from recordtype import recordtype
from typing import List, Tuple, Type
from Point import Point, PointContainer
from Point3D import Point3D

MinMax = recordtype('MinMax', 'min max')


class BoundaryBox:
    def __init__(self, cls: Type[Point], inf: float = 10e8):
        """
        Инициализировать boundary box.

        Args:
            cls (Type[Point]): класс хранимых точек.
            inf (float): бесконечно большое число, заведомо больше любой координаты точки.
        """
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
    def vertexes(self) -> List[Point]:
        """
        Вершины, образующие boundary box.

        Returns:
            List[Point]: вершины.
        """
        lst = [
            [self._bounds[j].min if tool[j] else self._bounds[j].max for j in range(self._dimension)] for tool
            in itertools.product([0, 1], repeat=self._dimension)]
        return [self._cls(*obj) for obj in lst]

    def update(self, point: Point):
        """
        Обновить с учетом добавленной точки.

        Args:
            point (Point): точка.
        """
        if type(point) != self._cls:
            raise AttributeError(f"Point have to be {self._cls}")

        for i, coordinate in enumerate(point):
            self._bounds[i].min = min(self._bounds[i].min, coordinate)
            self._bounds[i].max = max(self._bounds[i].max, coordinate)

    def distance(self, point: Point) -> float:
        """
        Расстояние до точки

        Args:
            point (Point): точка.

        Returns:
            float- расстояние.
        """
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

    def __init__(self, container_cls: Type[PointContainer], inf=10e8):
        """
        Инициализировать K-d дерево на основе контейнера точек.

        Args:
            container_cls (Type[PointContainer]): класс контейнера точек.
            inf (float): бесконечно большое число, заведомо больше любой координаты точки.
        """

        """
        _container_cls - класс контейнера точек.
        _cls - класс точек.
        _dimension - размерность точек.
        _inf - бесконечно большое число.
        """
        self._container_cls = container_cls
        self._cls = container_cls.point_class()
        self._dimension = container_cls.point_class().dimension()
        self._inf = inf

        self._tree = [BoundaryBox(self._cls, self._inf)] * 2
        self._left_child_index, self._right_child_index = [-1] * 2, [-1] * 2

    def _add_node(self):
        """
        Добавить пустую вершину к дереву.
        """
        self._tree.append(BoundaryBox(self._cls, self._inf))
        self._left_child_index.append(-1)
        self._right_child_index.append(-1)

    def build(self, point_collection: PointContainer, tree_index: int = 1, coordinate: int = 0):
        """
        Построить дерево по коллекции точек

        Args:
            point_collection (PointContainer): коллекция точек.
            tree_index (PointContainer): текущий индекс элемента дерева.
            coordinate (int): координата точки на текущей итерации.
        """
        if point_collection.point_class() != self._cls:
            raise AttributeError(f"Container have to contain {self._cls} objects")

        for point in point_collection:
            self._tree[tree_index].update(point)
        if len(point_collection) <= 1:
            return

        point_collection.sort(key=lambda p: (p[coordinate]))

        # Initialize left sub-tree
        self._add_node()
        self._left_child_index[tree_index] = len(self._tree) - 1
        self.build(self._container_cls(point_collection[len(point_collection) // 2:]),
                   self._left_child_index[tree_index],
                   (coordinate + 1) % self._dimension)

        # Initialize right sub-tree
        self._add_node()
        self._right_child_index[tree_index] = len(self._tree) - 1
        self.build(self._container_cls(point_collection[:len(point_collection) // 2]),
                   self._right_child_index[tree_index],
                   (coordinate + 1) % self._dimension)

    def find_closest(self, point: Point) -> Tuple[float, Point]:
        """
        Найти ближайшего соседа к точке.

        Args:
            point (Point): точка.

        Returns:
             Tuple[float, Point] - (расстояние до ближайшей точки, ближайшая точка).
        """

        def _find_closest(tree_index=1):
            nonlocal min_dist, res
            if self._left_child_index[tree_index] == -1:
                min_dist = self._tree[tree_index].distance(point)
                res = self._cls(*[self._tree[tree_index].bounds[i].min for i in range(self._dimension)])
                return
            if self._tree[self._left_child_index[tree_index]].distance(point) < min_dist:
                _find_closest(self._left_child_index[tree_index])
            if self._tree[self._right_child_index[tree_index]].distance(point) < min_dist:
                _find_closest(self._right_child_index[tree_index])

        if type(point) != self._cls:
            raise AttributeError(f"Point have to be {self._cls}")
        min_dist = self._inf
        res = self._cls()
        _find_closest()
        return min_dist, res
