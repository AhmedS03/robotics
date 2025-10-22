import ikpy.chain
import numpy as np

# Create the robot chain from the URDF file.
# You might need to find a UR5e URDF file or create one from your XML.
# Many are available online. For now, we define it via DH params.
ur5e_chain = ikpy.chain.Chain.from_urdf_file("path_to_your_ur5e.urdf")


if __name__ == '__main__':
    # Target position (x, y, z)
    target_position = [0.3, 0.2, 0.4]

    # Optional: Target orientation
    # By default, it tries to keep the end-effector orientation.
    # You can specify a target orientation matrix (3x3).
    # target_orientation = np.eye(3) 

    print(f"--- Inverse Kinematics UR5e ---")
    print(f"Target Position: {target_position}\n")

    # Calculate inverse kinematics
    ik_solution = ur5e_chain.inverse_kinematics(
        target_position,
        # target_orientation=target_orientation,
        # orientation_mode="all" # Can be 'x', 'y', 'z', 'all' or None
    )

    # Convert radians to degrees for readability
    ik_solution_deg = np.rad2deg(ik_solution)

    print(f"IK Solution (rad): {ik_solution.tolist()}\n")
    print(f"IK Solution (deg): {ik_solution_deg.tolist()}\n")

    # --- Verification using Forward Kinematics ---
    # You can use the fk from ikpy to verify the result
    fk_matrix = ur5e_chain.forward_kinematics(ik_solution)
    fk_position = fk_matrix[:3, 3]

    print("--- Verification ---")
    print(f"Position from IK solution: {fk_position}")
    print(f"This should be very close to your target position: {target_position}")
