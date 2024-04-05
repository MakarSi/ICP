# -*- coding: utf-8 -*-
import open3d as o3d

# Загружаем PointCloud из файла ply
pcd = o3d.io.read_point_cloud("tests/bun000.ply")

# Уменьшаем количество точек, выбирая рядом стоящие точки
downsampled_pcd = pcd.voxel_down_sample(voxel_size=0.01)

# Сохраняем сжатый PointCloud в новый файл
o3d.io.write_point_cloud("output2.ply", downsampled_pcd, write_ascii=True)
