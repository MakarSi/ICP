# -*- coding: utf-8 -*-
import open3d as o3d

filename = "bun000"

# Загружаем PointCloud из файла ply
pcd = o3d.io.read_point_cloud("data/ply_files/" + filename + ".ply")

# Уменьшаем количество точек
down_sampled_pcd = pcd.voxel_down_sample(voxel_size=0.01)

# Сохраняем сжатый PointCloud в новый файл
o3d.io.write_point_cloud(filename + "_small.ply", down_sampled_pcd, write_ascii=True)
