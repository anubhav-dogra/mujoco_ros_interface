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

# F1 = np.array([0.0, -0.0017, -30.411])
# tau_1 = np.array([-0.152, 0.4561, 0.0])
# F2 = np.array([-0.0517, -30.50, 0.049])
# tau_2 = np.array([0.1678, 0.00099, -0.369])
# F3 = np.array([-30.40, 0.411, 0.0])
# tau_3 = np.array([-0.0020, -0.152, 0.158])

# on lbr
F1 = np.array([-2.35, 1.37, 33.65])
tau_1 = np.array([0.01, -0.35, -0.0365])
F2 = np.array([-3.71, 34.07, -0.3848])
tau_2 = np.array([-2.59, 0.0369, 0.6782])
F3 = np.array([30.80, -0.003087, -0.6079])
tau_3 = np.array([0.0808, 2.476, -0.041005])
F4 = np.array([5.22, -14.194, 28.066])
tau_4 = np.array([1.250, 0.2188, -0.1850])
F5 = np.array([-2.8, -32.9, 0.155])
tau_5 = np.array([2.678, -0.098, -0.478])
F6 = np.array([30.16, 0.2782, 1.4277])
tau_6 = np.array([0.0706, 2.388, 0.003645])
F7 = np.array([-2.368, 0.7310, 33.87])
tau_7 = np.array([0.0715, -0.3616, -0.09729])
F8 = np.array([-16.82, -9.086, 28.813])
tau_8 = np.array([0.8712, -1.535, -0.133])
F9 = np.array([-1.93709, 2.025, 33.47])
tau_9 = np.array([0.0485, -0.3392, -0.0987])
F10 = np.array([-2.987, -31.949, 0.9028])
tau_10 = np.array([2.608, -0.0558, -0.468])
F11 = np.array([-2.076, 0.21573, 33.914])
tau_11 = np.array([0.06516, -0.2395, -0.0624])

# auto arange tau as a single column using tau_1, tau_2, tau_3 ,... tau_n

tau = np.array([tau_1, tau_2, tau_3, tau_4, tau_5, tau_6, tau_7, tau_8, tau_9, tau_10, tau_11])
tau = np.reshape(tau, (-1, 1))

F_mat1 = f_cross(F1)
F_mat2 = f_cross(F2)
F_mat3 = f_cross(F3)
F_mat4 = f_cross(F4)
F_mat5 = f_cross(F5)
F_mat6 = f_cross(F6)
F_mat7 = f_cross(F7)
F_mat8 = f_cross(F8)
F_mat9 = f_cross(F9)
F_mat10 = f_cross(F10)
F_mat11 = f_cross(F11)

F_mat = -np.array([F_mat1,F_mat2,F_mat3,F_mat4,F_mat5,F_mat6,F_mat7,F_mat8,F_mat9,F_mat10,F_mat11])
F_mat = np.reshape(F_mat, (33, 3))
# pinv = np.linalg.pinv(F_mat)
com = np.linalg.lstsq(F_mat, tau, rcond=None)[0]
# com = pinv.dot(tau)
print(com)