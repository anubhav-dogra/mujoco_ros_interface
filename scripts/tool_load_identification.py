import numpy as np


def f_cross(F):
    F_cross = np.array([[0, -F[2], F[1]], [F[2], 0, -F[0]], [-F[1], F[0], 0]])
    return F_cross


# F1 = np.array([0.0, 0.0, 30.411])
# tau_1 = np.array([-0.1520, -0.45616, 0.0])
# F2 = np.array([0.0, 30.41, 0.0])
# tau_2 = np.array([1.6728, 0.0, 0.45616])
# F3 = np.array([-30.40, 0.0253, 0.4841])
# tau_3 = np.array([-0.00102, 1.6654, -0.1516])

F1 = np.array([0.0, -0.0017, -30.411])
tau_1 = np.array([-0.152, 0.4561, 0.0])
F2 = np.array([-0.0517, -30.50, 0.049])
tau_2 = np.array([0.1678, 0.00099, -0.369])
F3 = np.array([-30.40, 0.411, 0.0])
tau_3 = np.array([-0.0020, -0.152, 0.158])

# arange tau as a single column using tau_1, tau_2, tau_3
tau = np.array([tau_1, tau_2, tau_3])
tau = np.reshape(tau, (-1, 1))

F_mat1 = f_cross(F1)
F_mat2 = f_cross(F2)
F_mat3 = f_cross(F3)

F_mat = np.array([F_mat1,F_mat2,F_mat3])
F_mat = np.reshape(F_mat, (9, 3))
pinv = np.linalg.pinv(-F_mat)
com = np.linalg.lstsq(F_mat, tau, rcond=None)[0]
com = pinv.dot(tau)
print(com)