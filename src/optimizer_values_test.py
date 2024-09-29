from scipy.spatial.transform import Rotation as R
import numpy as np


def z_rot_offset_calc(z_rot):
    return -0.001641 * z_rot + 0.255080

def rotation_matrix_from_euler_angles(x, y, z):
    # Convert Euler angles to rotation matrix
    R_x = R.from_euler('xyz', [x, 0, 0], degrees=True)
    R_y = R.from_euler('xyz', [0, y, 0], degrees=True)
    R_z = R.from_euler('xyz', [0, 0, z], degrees=True)

    # Compose the rotation matrices
    Rot = R_x * R_y * R_z
    return Rot.as_matrix()

def solve_for_x(Y, R):
    # Define the identity matrix
    I = np.eye(3)
    
    # Compute (R + I)
    R_plus_I = R + I

    # Check if the matrix is invertible
    if np.linalg.det(R_plus_I) == 0:
        raise ValueError("The matrix R + I is not invertible.")
    
    # Compute the inverse of (R + I)
    R_plus_I_inv = np.linalg.inv(R_plus_I)

    # Solve for x
    x = np.dot(R_plus_I_inv, Y)

    return x

C = np.array([[ 0.34301724, -0.93932911,  0,          0.03097672],
 [ 0.93932911 , 0.34301724 , 0 ,        -0.65846769],
 [ 0        , 0      ,   1    ,    0.092     ],
 [ 0      ,   0      ,    0        , 1     ]])

C_rot_matrix = C[:3, :3]
C_trans_vec = C[:3, 3]

# extracting the z-axis rotation (yaw) from the rotation matrix
euler_angles = R.from_matrix(C_rot_matrix).as_euler('xyz', degrees=True)
z_rot= euler_angles[2]  # Yaw angle
print(f"z_rot: {z_rot:.7f}")
z_rot_offset = z_rot_offset_calc(z_rot)
print(f"z_rot_offset: {z_rot_offset:.7f}")
z_rot_offset_rot_matrix = rotation_matrix_from_euler_angles(0, 0, z_rot_offset)
C_fixed_rot_matrix = C_rot_matrix @ z_rot_offset_rot_matrix

C_pure_trans_vec = solve_for_x(C_trans_vec, C_rot_matrix)
C_fixed_trans_vec = C_fixed_rot_matrix @ C_pure_trans_vec + C_pure_trans_vec

C_fixed = C
C_fixed[:3, :3] = C_fixed_rot_matrix
C_fixed[:3, 3] = C_fixed_trans_vec

print("The fixed transformation matrix is:")
print(C_fixed)