import numpy as np
# starting from a rotation matrix
R = np.array([
    [1, 0, 0],    
    [0, 0, -1],
    [0, 1, 0]])
# it is possible to obtain the roll, pitch and yaw angles from inverse formulas
pitch = np.arcsin(-R[2][0])
roll = np.arctan2(R[2][1] / np.cos(pitch), R[2][2] / np.cos(pitch))
yaw = np.arctan2(R[1][0] / np.cos(pitch), R[0][0] / np.cos(pitch))
# the angles are in radians, to convert them in degrees we use the np.degrees() function
roll_deg = np.degrees(roll)
pitch_deg = np.degrees(pitch)
yaw_deg = np.degrees(yaw)
print("Roll:", roll_deg)
print("Pitch:", pitch_deg)
print("Yaw:", yaw_deg)