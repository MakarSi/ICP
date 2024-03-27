import numpy as np
import random as rd


def mult(pnt, mat):
    res = [0] * 3
    for i in range(3):
        for j in range(3):
            res[i] += pnt[j] * mat[i][j]
    return res


def mat_mult(mat1, mat2):
    res = []
    for i in range(3):
        res.append([0] * 3)
    for i in range(3):
        for j in range(3):
            for k in range(3):
                res[i][j] += mat1[i][k] * mat2[k][j]
    return res


def axis_rotation_mat(axis, ang):
    x, y, z = axis[0], axis[1], axis[2]
    cosa, sina = np.cos(ang), np.sin(ang)
    res = []
    for i in range(3):
        res.append([0] * 3)
    res[0][0] = cosa + (1 - cosa) * x * x
    res[0][1] = (1 - cosa) * x * y - sina * z
    res[0][2] = (1 - cosa) * x * z + sina * y
    res[1][0] = (1 - cosa) * y * x + sina * z
    res[1][1] = cosa + (1 - cosa) * y * y
    res[1][2] = (1 - cosa) * y * z - sina * x
    res[2][0] = (1 - cosa) * z * x - sina * y
    res[2][1] = (1 - cosa) * z * y + sina * x
    res[2][2] = cosa + (1 - cosa) * z * z
    return res


def slight_rotation():
    x_ang, y_ang, z_ang = np.random.uniform(0, np.pi / 20), np.random.uniform(0, np.pi / 20), np.random.uniform(0,
                                                                                                                np.pi / 20)
    mat1 = axis_rotation_mat([1, 0, 0], x_ang)
    mat2 = axis_rotation_mat([0, 1, 0], y_ang)
    mat3 = axis_rotation_mat([0, 0, 1], z_ang)
    return mat_mult(mat1, mat_mult(mat2, mat3))


def slight_shift():
    return [rd.randint(-5, 5), rd.randint(-5, 5), rd.randint(-5, 5)]


def no_rotation():
    mat = []
    for i in range(3):
        mat.append([0] * 3)
        mat[i][i] = 1
    return mat


def no_shift():
    return [0, 0, 0]


def vec_sum(vec1, vec2):
    res = vec1.copy()
    for i in range(3):
        res[i] += vec2[i]
    return res


def vec_dif(vec1, vec2):
    res = vec1.copy()
    for i in range(3):
        res[i] -= vec2[i]
    return res


def centroid(s):
    x, y, z = 0, 0, 0
    for p in s:
        x += p[0]
        y += p[1]
        z += p[2]
    x /= len(s)
    y /= len(s)
    z /= len(s)
    return [x, y, z]


def rotate_cloud(s, rot_mat):
    for i in range(len(s)):
        s[i] = mult(s[i], rot_mat)


def shift_cloud(s, shift, f):
    for i in range(len(s)):
        if f:
            s[i] = vec_sum(s[i], shift)
        else:
            s[i] = vec_dif(s[i], shift)


def norm(vec):
    x, y, z = vec[0], vec[1], vec[2]
    return np.sqrt(x * x + y * y + z * z)


def norm_cross(vec1, vec2):
    x1, y1, z1 = vec1[0], vec1[1], vec1[2]
    x2, y2, z2 = vec2[0], vec2[1], vec2[2]
    res = [0] * 3
    res[0] = y1 * z2 - z1 * y2
    res[1] = z1 * x2 - x1 * z2
    res[2] = x1 * y2 - y1 * x2
    nr = norm(res)
    res[0] /= nr
    res[1] /= nr
    res[2] /= nr
    return res


def ang_dif(vec1, vec2):
    x1, y1, z1 = vec1[0], vec1[1], vec1[2]
    x2, y2, z2 = vec2[0], vec2[1], vec2[2]
    sc = x1 * x2 + y1 * y2 + z1 * z2
    return np.arccos(sc / norm(vec1) / norm(vec2))
