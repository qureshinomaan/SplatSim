import numpy as np

#R_gaussian_camera = torch.from_numpy(np.array([[-0.97686609, -0.14716841, -0.15515831],
                        # [-0.19480104,  0.91172397,  0.36167935],
                        # [ 0.08823378,  0.38353729, -0.91930079]])).float()
        
# T_gaussian_camera = torch.Tensor([ 0.09347542, -0.74648806,  5.57444971] ).float()

H_gaussian_camera = np.array([[ -0.97686609, -0.14716841, -0.15515831, 0.09347542],
                              [-0.19480104,  0.91172397,  0.36167935, -0.74648806],
                              [ 0.08823378,  0.38353729, -0.91930079, 5.57444971],
                              [0, 0, 0, 1]])

H_robotincorrect_gaussian = np.array([[0.144820347428, 0.019638715312, -0.143120646477, -0.240426957607],
                            [0.141510024667, 0.021471565589, 0.146136879921, 0.408296585083],
                            [0.029053352773, -0.202473223209, 0.001615444897, 0.492784976959],
                            [0, 0, 0, 1]])

H_robotincorrect_camera = np.matmul(H_robotincorrect_gaussian, H_gaussian_camera)

print(H_robotincorrect_camera)

z_angle = 43
z_angle = np.radians(z_angle)
H_robotincorrect_R = np.array([[np.cos(z_angle), -np.sin(z_angle), 0, 0],
              [np.sin(z_angle), np.cos(z_angle), 0, 0],
              [0, 0, 1, 0], 
              [0, 0, 0, 1]])

H_R_camera = np.matmul(np.linalg.inv(H_robotincorrect_R), H_robotincorrect_camera)

print(H_R_camera)


H_C_T = np.array([[1, 0, 0, 0], [0, 1, 0, 0], [0, 0, 1, 0.44], [0, 0, 0, 1]])

H_R_T = np.array([[-1, 0, 0, 0.711], [0, 1, 0, 0.564], [0, 0, -1, 0.], [0, 0, 0, 1]])

H_T_camera = np.matmul(np.linalg.inv(H_R_T), H_R_camera)

H_camera_T = np.matmul(np.linalg.inv(H_R_camera), H_R_T)

print(H_T_camera)