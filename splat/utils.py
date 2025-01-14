
import numpy as np
import pytorch3d.transforms
import torch

def quaternion_multiply(q1, q2):
    """
    Multiplies two quaternions.
    """
    w1, x1, y1, z1 = q1
    w2, x2, y2, z2 = q2
    return np.array([
        w1*w2 - x1*x2 - y1*y2 - z1*z2,
        w1*x2 + x1*w2 + y1*z2 - z1*y2,
        w1*y2 - x1*z2 + y1*w2 + z1*x2,
        w1*z2 + x1*y2 - y1*x2 + z1*w2,
    ])

def quaternion_to_rot_matrix(quat):
    """
    Convert a quaternion into a rotation matrix.
    """
    x, y, z, w = quat
    return np.array([[1 - 2*y**2 - 2*z**2, 2*x*y - 2*z*w, 2*x*z + 2*y*w],
                     [2*x*y + 2*z*w, 1 - 2*x**2 - 2*z**2, 2*y*z - 2*x*w],
                     [2*x*z - 2*y*w, 2*y*z + 2*x*w, 1 - 2*x**2 - 2*y**2]])

def quaternion_conjugate(q):
    """
    Computes the conjugate of a quaternion.
    """
    w, x, y, z = q
    return np.array([w, -x, -y, -z])

def compute_transformation(pose_a, pose_b):
    (x1, y1, z1, q1) = pose_a
    (x2, y2, z2, q2) = pose_b

    r1 = quaternion_to_rot_matrix(q1)
    r2 = quaternion_to_rot_matrix(q2)

    r_rel = np.matmul(r2, r1.T)

    T = np.array([x2, y2, z2]) - np.matmul(r_rel, np.array([x1, y1, z1]))
    
    return r_rel, T

def apply_transformation(p, q_p, q_rel, T):
    # Apply rotation (using quaternion multiplication)
    p_quat = np.array([0, *p])
    p_rotated = quaternion_multiply(quaternion_multiply(q_rel, p_quat), quaternion_conjugate(q_rel))
    
    # Extract the vector part and apply translation
    p_transformed = p_rotated[1:] + T
    
    # Apply rotation to the orientation
    q_p_transformed = quaternion_multiply(q_rel, q_p)
    
    return p_transformed, q_p_transformed


def get_euler_from_matrix(matrix):
    """
    Extract Euler angles from a rotation matrix.
    """
    if matrix[2, 0] != 1 and matrix[2, 0] != -1:
        y1 = -np.arcsin(matrix[2, 0])
        y2 = np.pi - y1
        x1 = np.arctan2(matrix[2, 1] / np.cos(y1), matrix[2, 2] / np.cos(y1))
        x2 = np.arctan2(matrix[2, 1] / np.cos(y2), matrix[2, 2] / np.cos(y2))
        z1 = np.arctan2(matrix[1, 0] / np.cos(y1), matrix[0, 0] / np.cos(y1))
        z2 = np.arctan2(matrix[1, 0] / np.cos(y2), matrix[0, 0] / np.cos(y2))
    else:
        z = 0
        if matrix[2, 0] == -1:
            y = np.pi / 2
            x = z + np.arctan2(matrix[0, 1], matrix[0, 2])
        else:
            y = -np.pi / 2
            x = -z + np.arctan2(-matrix[0, 1], -matrix[0, 2])
    return np.array([x1, y1, z1])


def get_quaternion_from_matrix(matrix):
    """
    Extract quaternion from a rotation matrix.
    """
    t = matrix.trace()
    if t > 0:
        s = 0.5 / np.sqrt(t + 1)
        w = 0.25 / s
        x = (matrix[2, 1] - matrix[1, 2]) * s
        y = (matrix[0, 2] - matrix[2, 0]) * s
        z = (matrix[1, 0] - matrix[0, 1]) * s
    elif matrix[0, 0] > matrix[1, 1] and matrix[0, 0] > matrix[2, 2]:
        s = 2 * np.sqrt(1 + matrix[0, 0] - matrix[1, 1] - matrix[2, 2])
        w = (matrix[2, 1] - matrix[1, 2]) / s
        x = 0.25 * s
        y = (matrix[0, 1] + matrix[1, 0]) / s
        z = (matrix[0, 2] + matrix[2, 0]) / s
    elif matrix[1, 1] > matrix[2, 2]:
        s = 2 * np.sqrt(1 + matrix[1, 1] - matrix[0, 0] - matrix[2, 2])
        w = (matrix[0, 2] - matrix[2, 0]) / s
        x = (matrix[0, 1] + matrix[1, 0]) / s
        y = 0.25 * s
        z = (matrix[1, 2] + matrix[2, 1]) / s
    else:
        s = 2 * np.sqrt(1 + matrix[2, 2] - matrix[0, 0] - matrix[1, 1])
        w = (matrix[1, 0] - matrix[0, 1]) / s
        x = (matrix[0, 2] + matrix[2, 0]) / s
        y = (matrix[1, 2] + matrix[2, 1]) / s
        z = 0.25 * s

    return np.array([ x, y, z, w])

if __name__ == '__main__':
    # Define poses
    pose_a = (0, 0, 0, np.array([1, 0, 0, 0]))  # Initial pose
    pose_b = (1, 1, 1, np.array([0, 0, 1, 0]))  # Final pose
    r_rel, T = compute_transformation(pose_a, pose_b)

    # Apply transformation
    p = np.array([2, 2, 2])  # Position of another point
    q_p = np.array([1, 0, 0, 0])  # Orientation of another point
    p_transformed, q_p_transformed = apply_transformation(p, q_p, q_rel, T)

    p_transformed, q_p_transformed