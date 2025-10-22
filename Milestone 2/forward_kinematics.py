import numpy as np

def dh_matrix(alpha, a, d, theta):
    """
    Calculate the Denavit-Hartenberg transformation matrix.
    """
    return np.array([
        [np.cos(theta), -np.sin(theta) * np.cos(alpha),  np.sin(theta) * np.sin(alpha), a * np.cos(theta)],
        [np.sin(theta),  np.cos(theta) * np.cos(alpha), -np.cos(theta) * np.sin(alpha), a * np.sin(theta)],
        [0,             np.sin(alpha),                  np.cos(alpha),                  d],
        [0,             0,                              0,                              1]
    ])

def forward_kinematics_ur5e(joint_angles):
    """
    Calculate the forward kinematics for the UR5e robot.
    joint_angles: a list or array of 6 joint angles in radians.
    """
    # DH Parameters for UR5e
    # alpha_{i-1}, a_{i-1}, d_i
    dh_params = [
        [0,          0,        0.1625],
        [np.pi/2,    0,        0],
        [0,         -0.425,    0],
        [0,         -0.3922,   0.1333],
        [np.pi/2,    0,        0.0997],
        [-np.pi/2,   0,        0.0996]
    ]

    T_0_6 = np.identity(4)

    for i in range(6):
        alpha = dh_params[i][0]
        a = dh_params[i][1]
        d = dh_params[i][2]
        theta = joint_angles[i]
        
        A_i = dh_matrix(alpha, a, d, theta)
        T_0_6 = np.dot(T_0_6, A_i)

    return T_0_6

if __name__ == '__main__':
    # Example usage: joint angles (in radians)
    # e.g., all joints at 0 degrees, except joint 2 at -90 deg and joint 4 at -90 deg
    q = [0, -np.pi/2, 0, -np.pi/2, 0, 0] 

    # Calculate the final transformation matrix
    end_effector_matrix = forward_kinematics_ur5e(q)

    # Extract position (x, y, z)
    position = end_effector_matrix[:3, 3]

    # Extract rotation matrix
    rotation = end_effector_matrix[:3, :3]
    
    print("--- Forward Kinematics UR5e ---")
    print(f"Joint Angles (rad): {q}\n")
    print(f"End-Effector Position (x, y, z): {position}\n")
    print(f"End-Effector Rotation Matrix:\n{rotation}\n")
    print(f"Full Transformation Matrix:\n{end_effector_matrix}\n")
