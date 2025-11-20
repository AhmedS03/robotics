import numpy as np
from mujoco_ros2.dh_fk import forward_kinematics
from mujoco_ros2.jacobian import analytic_jacobian
from mujoco_ros2.ik_newton import ik_newton

def main():
    q = np.array([0.0, -0.5, 0.4, 0.0, 0.0, 0.0])
    Ts, T = forward_kinematics(q)
    print("EE pos:", T[:3,3])
    J = analytic_jacobian(q)
    print("Jacobian shape:", J.shape)
    # IK test
    q0 = np.zeros(6)
    qsol, ok, iters = ik_newton(T, q0=q0)
    print("IK ok:", ok, "iters:", iters, "qsol:", qsol)

if __name__ == "__main__":
    main()
