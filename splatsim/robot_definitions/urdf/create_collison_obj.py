import coacd
import trimesh
import numpy as np

input_file = "./pusher.stl"
mesh = trimesh.load(input_file, force="mesh")
mesh = coacd.Mesh(mesh.vertices, mesh.faces)
result = coacd.run_coacd(mesh)

output_file = "./pusher_collision.obj"

mesh_parts = []
for vs, fs in result:
    mesh_parts.append(trimesh.Trimesh(vs, fs))
scene = trimesh.Scene()
np.random.seed(0)
for p in mesh_parts:
    p.visual.vertex_colors[:, :3] = (np.random.rand(3) * 255).astype(np.uint8)
    scene.add_geometry(p)
scene.export(output_file)