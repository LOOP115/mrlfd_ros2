import numpy as np

def get_tf_mat(i, dh):
    a = dh[i][0]
    d = dh[i][1]
    alpha = dh[i][2]
    theta = dh[i][3]
    q = theta

    return np.array([[np.cos(q), -np.sin(q), 0, a],
                     [np.sin(q) * np.cos(alpha), np.cos(q) * np.cos(alpha), -np.sin(alpha), -np.sin(alpha) * d],
                     [np.sin(q) * np.sin(alpha), np.cos(q) * np.sin(alpha), np.cos(alpha), np.cos(alpha) * d],
                     [0, 0, 0, 1]])


def get_jacobian(joint_angles):
    dh_params = np.array([[0, 0.333, 0, joint_angles[0]],
                 [0, 0, -np.pi / 2, joint_angles[1]],
                 [0, 0.316, np.pi / 2, joint_angles[2]],
                 [0.0825, 0, np.pi / 2, joint_angles[3]],
                 [-0.0825, 0.384, -np.pi / 2, joint_angles[4]],
                 [0, 0, np.pi / 2, joint_angles[5]],
                 [0.088, 0, np.pi / 2, joint_angles[6]],
                 [0, 0.107, 0, 0],
                 [0, 0, 0, -np.pi / 4],
                 [0.0, 0.1034, 0, 0]], dtype=np.float64)

    T_EE = np.identity(4)
    for i in range(7 + 3):
        T_EE = T_EE @ get_tf_mat(i, dh_params)

    J = np.zeros((6, 10))
    T = np.identity(4)
    for i in range(7 + 3):
        T = T @ get_tf_mat(i, dh_params)

        p = T_EE[:3, 3] - T[:3, 3]
        z = T[:3, 2]

        J[:3, i] = np.cross(z, p)
        J[3:, i] = z

    return J[:, :7]
