import trimesh
import numpy as np

# SCALE = 0.004
# MAX_OPENING_WIDTH = 0.085
# MARGIN = 0.005
# SAMPLE_POINT_NUM = 3000
#
# mesh = trimesh.load_mesh('828d1b43814fc98c93b89bfd41acaad0.obj')
# mesh.apply_scale(SCALE)
#
# points, faceIndices = trimesh.sample.sample_surface_even(mesh, SAMPLE_POINT_NUM)
# faceNormals = mesh.face_normals
#
# pointNum = np.size(points, 0)
#
# ray_origins = np.empty([pointNum, 3], dtype=float)
# ray_directions = np.empty([pointNum, 3], dtype=float)
#
# for index in range(pointNum):
#     point = points[index]
#     normal_inward = faceNormals[faceIndices[index]] * -1
#     ray_origins[index] = point - normal_inward * MARGIN
#     ray_directions[index] = normal_inward
#
# locations, index_ray, index_tri = mesh.ray.intersects_location(ray_origins=ray_origins, ray_directions=ray_directions)
#
# ray_visualize = trimesh.load_path(np.hstack((ray_origins,
#                                              ray_origins + ray_directions * MAX_OPENING_WIDTH)).reshape(-1, 2, 3))
#
# mesh.visual.face_colors = [155,155,155,255]
# mesh.visual.face_colors[index_tri] = [255, 0, 0, 255]
# print(np.size(locations, 0))
# scene = trimesh.Scene([mesh,
#                        ray_visualize])
# # show the visualization
# scene.show()


"""
    sample points from a mesh file
"""
class AntipodalSampler:

    """
        input:
            mesh: mesh of object
            scale: ratio to rescale the mesh
            maxOpeningWidth: maximum gripper opening width
            margin: the min distance of planned gripper tips to the surface of object
            samplePointNum: num of points sampled from the object surface
    """
    def __init__(self, meshFile, scale=1, maxOpeningWidth=0.085, minOpeningWidth=0.003, samplePointNum=1000, mu=0.1):
        self.mesh = trimesh.load_mesh(meshFile)
        self.faceNormals = self.mesh.face_normals
        # rescale
        self.mesh.apply_scale(scale)
        self.maxOpeningWidth = maxOpeningWidth
        self.minOpeningWidth = minOpeningWidth
        self.samplePointNum = samplePointNum
        self.surfacePoints, self.faceIndices = trimesh.sample.sample_surface_even(self.mesh, self.samplePointNum)
        self.mu = mu

    """
        return:
            list of antipodal point pairs : ndarray num * 2 * 3
    """
    def sample_antipodal_points(self):
        pointNum = np.size(self.surfacePoints, 0)

        # generate the ray
        rayOrigins = np.empty([pointNum, 3], dtype=float)
        rayDirections = np.empty([pointNum, 3], dtype=float)

        for index in range(pointNum):
            point = self.surfacePoints[index]
            inwardNormal = self.faceNormals[self.faceIndices[index]] * -1
            rayOrigins[index] = point + inwardNormal * self.minOpeningWidth  # avoid intersect_points that are too close
            rayDirections[index] = inwardNormal

        intersectPoints, indicesOfRay, indicesOfFace = self.mesh.ray.intersects_location(ray_origins=rayOrigins,
                                                                            ray_directions=rayDirections)

        processedPairIndices = self.__removeRedundantPairs(intersectPoints, indicesOfRay, indicesOfFace)
        processedPairNum = len(processedPairIndices)
        processedPair = np.empty([processedPairNum, 2, 3])
        count = 0
        for index in processedPairIndices:
            processedPair[count][0] = rayOrigins[ indicesOfRay[index] ]
            processedPair[count][1] = intersectPoints[index]

        return processedPair

    def __removeRedundantPairs(self, intersectPoints, indicesOfRay, indicesOfFace):
        pairNum = np.size(intersectPoints, 0)
        processedIndices = []
        for index in range(pairNum):
            if 1:
                # TODO dis too large / direction not match
                processedIndices.append(index)

        return processedIndices

    def visualization(self, processedPair):
        processedRayOrigin = processedPair[:, 0, :]
        # TODO visualization for test
        # processedRayDirectionUnNormalized = (processedRayOrigin - processedPair[:, 1, :])

    def _direction_normalize(self, directionUnNormalized):
        # TODO normalize the list of vector
        for vec in directionUnNormalized:
            pass


