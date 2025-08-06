import math

def rotation_matrix_to_euler_angles(R):
    """
    Convert a rotation matrix to Euler angles.

    Parameters:
    R (numpy.ndarray): A 3x3 rotation matrix.

    Returns:
    tuple: A tuple containing the Euler angles (roll, pitch, yaw) in radians.
    """
    assert R.shape == (3, 3), "Rotation matrix must be 3x3"

    sy = math.sqrt(R[0, 0] * R[0, 0] + R[1, 0] * R[1, 0])

    singular = sy < 1e-6

    if not singular:
        x = math.atan2(R[2, 1], R[2, 2])  # Roll
        y = math.atan2(-R[2, 0], sy)  # Pitch
        z = math.atan2(R[1, 0], R[0, 0])  # Yaw
    else:
        x = math.atan2(-R[1, 2], R[1, 1])
        y = math.atan2(-R[2, 0], sy)
        z = 0

    return x, y, z