import numpy as np

def dh_transform(a, alpha, d, theta):
    ca, sa = np.cos(alpha), np.sin(alpha)
    ct, st = np.cos(theta), np.sin(theta)
    return np.array([
        [ct, -st*ca,  st*sa, a*ct],
        [st,  ct*ca, -ct*sa, a*st],
        [0.0,    sa,     ca,    d],
        [0.0,   0.0,    0.0,  1.0]
    ])

def ur5e_dh_params():
    # UR5-like DH parameters (a_{i-1}, alpha_{i-1}, d_i)
    a = [0.0, -0.425, -0.3922, 0.0, 0.0, 0.0]
    alpha = [np.pi/2, 0.0, 0.0, np.pi/2, -np.pi/2, 0.0]
    d = [0.1625, 0.0, 0.0, 0.1333, 0.0997, 0.0996]
    return a, alpha, d

def forward_kinematics(q):
    """Return list of transforms T1..T6 and the T_end (4x4)."""
    if len(q) != 6:
        raise ValueError("q must be length 6")
    a, alpha, d = ur5e_dh_params()
    T = np.eye(4)
    Ts = []
    for i in range(6):
        A = dh_transform(a[i], alpha[i], d[i], q[i])
        T = T @ A
        Ts.append(T.copy())
    return Ts, T

def frames_origins_and_axes(q):
    """Return origins O0..O6 and z axes Z0..Z6 in base frame"""
    Ts, Tn = forward_kinematics(q)
    origins = [np.array([0.,0.,0.])]
    zs = [np.array([0.,0.,1.])]
    for T in Ts:
        origins.append(T[:3,3].copy())
        zs.append(T[:3,2].copy())
    return origins, zs

