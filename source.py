import mat as mt
import kdt
import numpy as np

n, s, t = int(input()), [], []
for i in range(n):
    s.append(list(map(float, input().split())))
t = s.copy()
t_cent = mt.centroid(t)

perturbation_rotation = mt.slight_rotation()
perturbation_shift = mt.slight_shift()

mt.shift_cloud(t, t_cent, False)
mt.rotate_cloud(t, perturbation_rotation)
mt.shift_cloud(t, mt.vec_sum(t_cent, perturbation_shift), True)

# cr = mt.norm_cross(pnt2, pnt1)
# mat = mt.axis_rotation_mat(cr, mt.ang_dif(pnt1, pnt2))
# pnt2 = mt.mult(pnt2, mat)

Tr = kdt.KDTree()
Tr.build(s, 1, 0)

penalty = 0

while True:
    s_cent, t_cent = [0, 0, 0], [0, 0, 0]
    penalty = 0
    for i in range(n):
        res = [kdt.inf, 0]
        neib = [[0, 0, 0]]
        Tr.find_closest(t[i], res, neib, 1)
        s_cent = mt.vec_sum(s_cent, neib[0])
        t_cent = mt.vec_sum(t_cent, t[i])
    for i in range(3):
        s_cent[i] /= n
        t_cent[i] /= n
    mt.shift_cloud(t, mt.vec_dif(s_cent, t_cent), True)

    sc = []
    s_cent = [0, 0, 0]
    t_cent = [0, 0, 0]
    for i in range(n):
        res = [kdt.inf, 0]
        neib = [[0, 0, 0]]
        Tr.find_closest(t[i], res, neib, 1)
        sc.append(neib[0])
        s_cent = mt.vec_sum(s_cent, sc[i])
        t_cent = mt.vec_sum(t_cent, t[i])
    s_cent[0] /= n
    s_cent[1] /= n
    s_cent[2] /= n
    t_cent[0] /= n
    t_cent[1] /= n
    t_cent[2] /= n
    mt.shift_cloud(sc, s_cent, False)
    mt.shift_cloud(t, t_cent, False)
    smat = []
    for i in range(3):
        smat.append([0] * 3)

    for i in range(3):
        for j in range(3):
            for k in range(n):
                smat[i][j] += t[k][i] * sc[k][j]

    nsmat = np.zeros((3, 3))
    for i in range(3):
        for j in range(3):
            nsmat[i][j] = smat[i][j]

    U, S, Vh = np.linalg.svd(nsmat, full_matrices=True)
    R = np.matmul(Vh.T, U.T)
    rot_mat = []
    for i in range(3):
        rot_mat.append([0] * 3)
        for j in range(3):
            rot_mat[i][j] = R[i][j]

    mt.rotate_cloud(t, R)
    mt.shift_cloud(t, t_cent, True)

    for i in range(n):
        res = [kdt.inf, 0]
        neib = [[0, 0, 0]]
        Tr.find_closest(t[i], res, neib, 1)
        penalty += res[0] * res[0]
    penalty /= n
    print(penalty)
    # draw(s, "red")
    # draw(t, "blue")
