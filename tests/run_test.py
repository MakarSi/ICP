import numpy as np
from matplotlib import pyplot as plt
from plyfile import PlyData

from ICP import ICP
from Point3D import Point3D
from PointCloud import PointCloud


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
    points = PointCloud([Point3D(p[0], p[1], p[2]) for p in points.tolist()])
    return points


if __name__ == '__main__':
    source = read_ply_file('bun000_small.ply')
    target = read_ply_file('bun045_small.ply')

    fig = plt.figure(figsize=(15, 15))
    ax = plt.axes(projection='3d')
    ICP(source, target).icp_algorithm(debug=True, ax=ax)
    plt.show()
