import numpy as np
from matplotlib import pyplot as plt
from plyfile import PlyData

from Point import Point
from PointCloud import PointCloud
from ICP import ICP


def read_ply_file(file_path):
    plydata = PlyData.read(file_path)

    vertices = plydata['vertex']

    x = vertices['x']
    y = vertices['y']
    z = vertices['z']

    points = np.vstack((x, y, z)).T

    return points


if __name__ == '__main__':
    A = read_ply_file('tests/bun000.ply')
    B = read_ply_file('tests/bun045.ply')

    fig = plt.figure(figsize=(15, 15))
    ax = plt.axes(projection='3d')
    ax.set_xlabel('X (m)')
    ax.set_ylabel('Y (m)')
    ax.set_zlabel('Z (m)')

    source = PointCloud([Point(p[0], p[1], p[2]) for p in A.tolist()[:40000]])
    target = PointCloud([Point(p[0], p[1], p[2]) for p in B.tolist()[:40000]])

    ax = plt.axes(projection='3d')

    target.draw(ax, 'g')
    source.draw(ax, 'k')
    plt.show()


    # target = PointCloud(
    #     [Point(-2, -4, 0), Point(-1, -2, 0), Point(0, 0, 0), Point(1, 2, 0), Point(2, 4, 0), Point(1, 1, 0),
    #      Point(-1, -1, 0)])
    #
    # source_array = np.random.permutation(target.get_matrix())
    # source = PointCloud([Point(p[0], p[1], p[2]) for p in source_array]).do_perturbation()
    #
    # ICP(source, target).draw_steps(ax, 10, 1)

# tests
# [Point(-2, -4, 0), Point(-1, -2, 0), Point(0, 0, 0), Point(1, 2, 0), Point(2, 4, 0)]
# [Point(1, 1, 0), Point(1, 0, 1), Point(0, 1, 1), Point(-2, -2, -2)]
