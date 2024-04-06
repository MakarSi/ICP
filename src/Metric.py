from abc import ABCMeta


class Metric(metaclass=ABCMeta):
    def distance(self, other: 'Metric') -> float:
        raise NotImplementedError

    @classmethod
    def dimension(cls):
        raise NotImplementedError

    def __iter__(self):
        raise NotImplementedError

    def __getitem__(self, item):
        raise NotImplementedError


class MetricContainer(metaclass=ABCMeta):
    @property
    def metric_type(self) -> Metric:
        raise NotImplementedError

    def sort(self):
        raise NotImplementedError

    def __len__(self):
        raise NotImplementedError

    def __iter__(self):
        raise NotImplementedError

    def __getitem__(self, item) -> Metric:
        raise NotImplementedError
