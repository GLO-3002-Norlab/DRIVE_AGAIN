import numpy as np


Pose = np.ndarray  # x, y, z, roll, pitch, yaw

Command = np.ndarray  # v_x, omega_z


def is_same_command(a: Command, b: Command):
    tol = 1e-6
    return abs(a[0] - b[0]) <= tol and abs(a[1] - b[1]) <= tol
