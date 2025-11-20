import numpy as np
from .dh_fk import forward_kinematics
from .jacobian import analytic_jacobian

def rotation_vector_from_matrix(R):
    angle = np.arccos(max(-1.0, min(1.0, (np.trace(R)-1.0)/2.0)))
    if abs(angle) < 1e-8:
        return np.zeros(3)
    rx = (R[2,1] - R[1,2])/(2*np.sin(angle))
    ry = (R[0,2] - R[2,0])/(2*np.sin(angle))
    rz = (R[1,0] - R[0,1])/(2*np.sin(angle))
    axis = np.array([rx, ry, rz])
    return axis * angle

def pose_error(T_cur, T_des):
    p_err = T_des[:3,3] - T_cur[:3,3]
    R_err = T_des[:3,:3] @ T_cur[:3,:3].T
    rot_err = rotation_vector_from_matrix(R_err)
    return np.hstack((p_err, rot_err))

def damped_pinv(J, lam=1e-6):
    m,n = J.shape
    if m >= n:
        return np.linalg.inv(J.T @ J + lam*np.eye(n)) @ J.T
    else:
        return J.T @ np.linalg.inv(J @ J.T + lam*np.eye(m))

def ik_newton(T_des, q0=None, max_iters=200, tol_pos=1e-4, tol_ori=1e-3):
    if q0 is None:
        q = np.zeros(6)
    else:
        q = q0.copy()
    for k in range(max_iters):
        Ts, Tcur = forward_kinematics(q)
        err = pose_error(Tcur, T_des)
        if np.linalg.norm(err[:3]) < tol_pos and np.linalg.norm(err[3:]) < tol_ori:
            return q, True, k
        J = analytic_jacobian(q)
        Jp = damped_pinv(J, lam=1e-6)
        dq = Jp @ err
        q += dq
    return q, False, max_iters

