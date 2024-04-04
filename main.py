import numpy as np

from matplotlib import pyplot as plt

from Point import Point
from PointCloud import PointCloud
from ICP import ICP

if __name__ == '__main__':
    ax = plt.axes(projection='3d')
    ax.set_xlim(-2, 2)
    ax.set_ylim(-2, 2)
    ax.set_zlim(-2, 2)

    target = PointCloud(
        [Point(-2, -4, 0), Point(-1, -2, 0), Point(0, 0, 0), Point(1, 2, 0), Point(2, 4, 0), Point(1, 1, 0),
         Point(-1, -1, 0)])

    source_array = np.random.permutation(target.get_matrix())
    source = PointCloud([Point(p[0], p[1], p[2]) for p in source_array]).do_perturbation()

    ICP(source, target).draw_steps(ax, 10, 1)

    # target.draw(ax, 'g')
    # source.draw(ax, 'k')
    plt.show()

# tests
# [Point(-2, -4, 0), Point(-1, -2, 0), Point(0, 0, 0), Point(1, 2, 0), Point(2, 4, 0)]
# [Point(1, 1, 0), Point(1, 0, 1), Point(0, 1, 1), Point(-2, -2, -2)]
