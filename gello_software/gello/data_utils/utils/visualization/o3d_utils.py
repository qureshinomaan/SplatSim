import numpy as np, open3d as o3d
from ..lib3d import np2pcd, to_o3d


def visualize_3d(objects, show_frame=True, frame_size=1.0, frame_origin=(0, 0, 0)):
    if not isinstance(objects, (list, tuple)):
        objects = [objects]
    objects = [to_o3d(obj) for obj in objects]
    if show_frame:
        objects.append(o3d.geometry.TriangleMesh.create_coordinate_frame(size=frame_size, origin=frame_origin))
    return o3d.visualization.draw_geometries(objects)


def visualize_pcd(points, colors=None, normals=None, bbox=None, show_frame=False, frame_size=1.0, frame_origin=(0, 0, 0), use_o3d=True):
    """Visualize a point cloud."""
    if points.shape[0] == 3:
        points = points.T
    if colors is not None and colors.shape[0] == 3:
        colors = colors.T
    if normals is not None and normals.shape[0] == 3:
        normals = normals.T
        
    pcd = np2pcd(points, colors, normals)
    geometries = [pcd]
    if show_frame:
        coord_frame = o3d.geometry.TriangleMesh.create_coordinate_frame(size=frame_size, origin=frame_origin)
        geometries.append(coord_frame)
    else:
        coord_frame = None
    if bbox is None:
        bbox = []
    elif not isinstance(bbox, (tuple, list)):
        bbox = [bbox]

    if use_o3d:
        o3d.visualization.draw_geometries(geometries + bbox)
    else:
        from pyrl.utils.lib3d import to_trimesh
        import trimesh
        scene = trimesh.scene.scene.Scene()
        pcd = to_trimesh(pcd)
        coord_frame = to_trimesh(coord_frame)
        scene.add_geometry(pcd)
        scene.add_geometry(coord_frame)
        scene.show()
        