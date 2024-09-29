import numpy as np

# Given matrices E and C
E = np.array([
    [0.338624, -0.940922, -9.8951e-06, 0.0320787],
    [0.940922, 0.338624, -1.62226e-05, -0.651648],
    [1.8615e-05, -3.81713e-06, 1, 0.0963264],
    [0, 0, 0, 1]
])

C = np.array([
    [0.34301724, -0.93932911, 0, 0.03097672],
    [0.93932911, 0.34301724, 0, -0.65846769],
    [0, 0, 1, 0.092],
    [0, 0, 0, 1]
])

# Compute the inverse of C
C_inv = np.linalg.inv(C)

# Compute T
T = np.dot(C_inv, E)

# Print the result
print(T)


import numpy as np
from scipy.spatial.transform import Rotation as R

# Given transformation matrix
transformation_matrix = np.array([
    [0.338624, -0.940922, -9.8951e-06, 0.0320787],
    [0.940922, 0.338624, -1.62226e-05, -0.651648],
    [1.8615e-05, -3.81713e-06, 1, 0.0963264],
    [0, 0, 0, 1]
])

# Extract rotation matrix (3x3 upper left part of the transformation matrix)
rotation_matrix = transformation_matrix[:3, :3]

# Extract translation vector (last column of the transformation matrix, except the last element)
translation_vector = transformation_matrix[:3, 3]

# Create a rotation object from the rotation matrix
rotation = R.from_matrix(rotation_matrix)

# Convert the rotation to Euler angles (specify the order you want, e.g., 'xyz')
euler_angles = rotation.as_euler('xyz', degrees=True)

print("Euler angles for 1 [roll, pitch, yaw]:", euler_angles)
print("Translation vector for 1[x, y, z]:", translation_vector)


# Given transformation matrix
transformation_matrix_2 = np.array([
    [0.34301724, -0.93932911, 0, 0.03097672],
    [0.93932911, 0.34301724, 0, -0.65846769],
    [0, 0, 1, 0.092],
    [0, 0, 0, 1]
])

# Extract rotation matrix (3x3 upper left part of the transformation matrix)
rotation_matrix_2 = transformation_matrix_2[:3, :3]

# Extract translation vector (last column of the transformation matrix, except the last element)
translation_vector_2 = transformation_matrix_2[:3, 3]

# Create a rotation object from the rotation matrix
rotation_2 = R.from_matrix(rotation_matrix_2)

# Convert the rotation to Euler angles (specify the order you want, e.g., 'xyz')
euler_angles_2 = rotation_2.as_euler('xyz', degrees=True)

print("Euler angles [roll, pitch, yaw] for second matrix:", euler_angles_2)
print("Translation vector [x, y, z] for second matrix:", translation_vector_2)

