import numpy as np

def save_point_cloud_as_ply(points, filename):
    """Save Nx3 numpy array as a .ply file."""
    with open(filename, 'w') as f:
        f.write("ply\nformat ascii 1.0\nelement vertex {}\n".format(len(points)))
        f.write("property float x\nproperty float y\nproperty float z\nend_header\n")
        for pt in points:
            f.write("{} {} {}\n".format(pt[0], pt[1], pt[2]))

def get_point_cloud_from_camera(pybullet_client, width=640, height=480):
    # Camera parameters (adjust as needed)
    view_matrix = pybullet_client.computeViewMatrix(
        cameraEyePosition=[1, 0, 1],
        cameraTargetPosition=[0, 0, 0],
        cameraUpVector=[0, 0, 1]
    )
    proj_matrix = pybullet_client.computeProjectionMatrixFOV(
        fov=60, aspect=float(width)/height, nearVal=0.01, farVal=10
    )
    _, _, _, depth, _ = pybullet_client.getCameraImage(
        width, height, view_matrix, proj_matrix, renderer=pybullet_client.ER_BULLET_HARDWARE_OPENGL
    )

    depth = np.array(depth)
    far = 10
    near = 0.01
    depth = far * near / (far - (far - near) * depth)

    x, y = np.meshgrid(np.arange(width), np.arange(height))
    x = x.flatten()
    y = y.flatten()
    z = depth.flatten()
    fx = width / (2 * np.tan(np.deg2rad(60) / 2))
    fy = fx
    cx = width / 2
    cy = height / 2
    X = (x - cx) * z / fx
    Y = (y - cy) * z / fy
    Z = z
    points = np.stack([X, Y, Z], axis=1)
    points = points[np.isfinite(points).all(axis=1)]
    return points