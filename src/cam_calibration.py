
from scipy.spatial.transform import Rotation as R
import numpy as np
import json

# base to task board transformation matrix - from touch detection
tb = np.array([
    [-0.575884, -0.817531, -1.11993e-05, 0.125211],
    [0.817531, -0.575884, 3.54378e-05, -0.633125],
    [-3.5421e-05, 1.12523e-05, 1, 0.0960402],
    [0, 0, 0, 1]
])

# camera to task board transformation matrix - from board detection
M = np.array([
    [0.5847149, 0.81123695, -0.00176267, -0.15764446],
    [0.81103886, -0.58461761, -0.02093377, -0.04993484],
    [-0.01801274, 0.0108107, -0.99977931, 0.84401977],
    [0, 0, 0, 1]
])

R_tb = tb[:3, :3]
t_tb = tb[:3, 3]

R_M = M[:3, :3]
t_M = M[:3, 3]

q_tb = R.from_matrix(R_tb).as_quat()  # (x, y, z, w)
q_M = R.from_matrix(R_M).as_quat()    # (x, y, z, w)

q_M_inv = R.from_quat(q_M).inv().as_quat()
t_M_inv = -R.from_quat(q_M_inv).apply(t_M)

R_combined = R.from_quat(q_tb) * R.from_quat(q_M_inv)
q_combined = R_combined.as_quat()
q_norm = q_combined / np.linalg.norm(q_combined)
t_combined = R.from_quat(q_tb).apply(t_M_inv) + t_tb

final_transform = np.eye(4)
final_transform[:3, :3] = R.from_quat(q_norm).as_matrix()
final_transform[:3, 3] = t_combined

print("Final Transformation Matrix:")
print(final_transform)

final_transform_list = final_transform.tolist()
final_transform_json = json.dumps(final_transform_list, indent=4)
print("Final Transformation Matrix in JSON format:")
print(final_transform_json)