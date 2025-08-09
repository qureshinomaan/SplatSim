import trimesh
import numpy as np
mesh = trimesh.load_mesh('828d1b43814fc98c93b89bfd41acaad0.obj')
# mesh.is_watertight)
# mesh.fill_holes()
# mesh.convex_hull
# if not mesh.is_watertight:
#     if not mesh.fill_holes():
#         mesh = mesh.convex_hull
# mesh.show()
mesh.apply_scale(0.003)

points, faceIndices = trimesh.sample.sample_surface_even(mesh, 3000)
faceNormals = mesh.face_normals
print(np.size(points, 0))
# for index in range(np.size(points, 0)):
#     point = points[index]
#     faceIndex = faceIndices[index]
#     normalVector = faceNormals[faceIndex]
#     print(point)
#     print(faceIndex)
#     print(normalVector)
#     print('-----------------')

# ray test
ray_origins = np.array([ points[0] + faceNormals[ faceIndices[0]] * 0.005 ])
ray_directions = np.array([ faceNormals[ faceIndices[0]] * -1 ])
locations, index_ray, index_tri = mesh.ray.intersects_location(ray_origins=ray_origins,
        ray_directions=ray_directions)
print(ray_origins)
print('The rays hit the mesh at coordinates:\n', locations)
ray_length = 0.080
ray_visualize = trimesh.load_path(np.hstack((ray_origins,
                                             ray_origins + ray_directions * ray_length)).reshape(-1, 2, 3))

# unmerge so viewer doesn't smooth
mesh.unmerge_vertices()
# make mesh white- ish
mesh.visual.face_colors = [155,155,155,255]
mesh.visual.face_colors[index_tri] = [255, 0, 0, 255]
# create a visualization scene with rays, hits, and mesh
scene = trimesh.Scene([mesh,
                       ray_visualize])
# show the visualization
scene.show()


# sudo apt install python3-rtree
# pip install rtree

