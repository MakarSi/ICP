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
    A = read_ply_file('tests/output.ply')
    B = read_ply_file('tests/output2.ply')

    source = PointCloud([Point(p[0], p[1], p[2]) for p in A.tolist()])
    target = PointCloud([Point(p[0], p[1], p[2]) for p in B.tolist()])

    fig = plt.figure(figsize=(15, 15))
    ax = plt.axes(projection='3d')

    ICP(source, target).draw_steps(ax, 30, 0.1)
    plt.show()

# tests
# [Point(-2, -4, 0), Point(-1, -2, 0), Point(0, 0, 0), Point(1, 2, 0), Point(2, 4, 0)]
# [Point(1, 1, 0), Point(1, 0, 1), Point(0, 1, 1), Point(-2, -2, -2)]
# [Point(0, 0, 0), Point(3, 0, 0), Point(0, 3, 0), Point(3, 3, 0), Point(1.5, 1.5, 4)]
