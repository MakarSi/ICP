from collections.abc import Iterable
from typing import Type, List, Optional
from abc import ABC


class Point(ABC):
    def __init__(self, coordinates: Optional[List[float]] = None):
        self._coordinates = coordinates

    @classmethod
    def dimension(cls):
        raise NotImplementedError

    def distance(self, other: 'Point') -> float:
        raise NotImplementedError

    def __iter__(self):
        raise NotImplementedError

    def __getitem__(self, item):
        raise NotImplementedError


class PointContainer:
    def __init__(self, points: List[Point]):
        self._points: Iterable = points

    @classmethod
    def point_class(cls) -> Type[Point]:
        raise NotImplementedError

    def sort(self, key):
        raise NotImplementedError

    def __len__(self):
        raise NotImplementedError

    def __iter__(self):
        raise NotImplementedError

    def __getitem__(self, item) -> Point | List[Point]:
        raise NotImplementedError
