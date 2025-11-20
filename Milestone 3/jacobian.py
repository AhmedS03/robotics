import numpy as np
from .dh_fk import frames_origins_and_axes

def analytic_jacobian(q):
    origins, zs = frames_origins_and_axes(q)
    o_n = origins[-1]
    Jv = np.zeros((3,6))
    Jw = np.zeros((3,6))
    for i in range(6):
        z_i = zs[i]
        o_i = origins[i]
        Jv[:,i] = np.cross(z_i, (o_n - o_i))
        Jw[:,i] = z_i
    J = np.vstack((Jv, Jw))
    return J

