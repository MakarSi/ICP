import numpy as np
from matplotlib import pyplot as plt
from plyfile import PlyData

from Point import Point
from PointCloud import PointCloud
from ICP import ICP


def read_ply_file(file_path: str) -> PointCloud:
    """
    Конвертация ply файла в PointCloud.

    Args:
        file_path (str): путь к файлу

    Returns:
        PointCloud: облако точек.
    """
    plydata = PlyData.read(file_path)

    vertices = plydata['vertex']
    x = vertices['x']
    y = vertices['y']
    z = vertices['z']

    points = np.vstack((x, y, z)).T
    points = PointCloud([Point(p[0], p[1], p[2]) for p in points.tolist()])
    return points


if __name__ == '__main__':
    source = read_ply_file('./tests/bun000_small.ply')
    target = read_ply_file('./tests/bun045_small.ply')

    fig = plt.figure(figsize=(15, 15))
    ax = plt.axes(projection='3d')
    ICP(source, target).icp_algorithm(debug=True, ax=ax)
    plt.show()
