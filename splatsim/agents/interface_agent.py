import pygame
import numpy as np
from typing import Dict
import cv2
import open3d as o3d
import os
from datetime import datetime
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

class InterfaceAgent:
    def __init__(self):
        pygame.init()
        self.screen = pygame.display.set_mode((400, 400))
        pygame.display.set_caption("6-DOF Camera Control Interface")
        
        # Track mouse state
        self.cv2_mouse_down = False
        self.cv2_last_mouse_pos = None
        self.window_name = 'image'
        self.z_speed = 0
        
        # Control parameters (adjusted for better feel)
        self.position_sensitivity = 0.003  # For translation
        self.rotation_sensitivity = 0.009  # For rotation
        self.z_sensitivity = 0.01  # Increased for more noticeable effect
        
        # Initialize other variables
        self.clicked_points = []
        self.clicked_points_3d = []
        self.mouse_clicked = False
        
        # Remove z_accumulated and z_decay as we won't need them anymore
        # self.z_accumulated = 0
        # self.z_decay = 0.95
        
        # Add new image saving related attributes
        self.is_saving = False
        self.save_counter = 0
        self.save_dir = None
        
        # Add trajectory visualization setup
        plt.ion()  # Enable interactive mode
        self.fig = plt.figure(figsize=(8, 8))
        self.ax = self.fig.add_subplot(111, projection='3d')
        self.camera_positions = []  # Store camera positions
        self.camera_orientations = []  # Store camera orientations
        
        # Setup initial plot
        self.setup_3d_plot()
        
    def setup_3d_plot(self):
        """Initialize the 3D plot with better formatting and interactivity"""
        self.ax.clear()
        
        # Set equal aspect ratio and better viewing angle
        self.ax.view_init(elev=30, azim=45)
        
        # Plot base coordinate frame
        self.plot_coordinate_frame(np.eye(3), np.zeros(3), scale=0.5, label='base')
        
        # Better formatting
        self.ax.set_xlabel('X', fontsize=12, labelpad=10)
        self.ax.set_ylabel('Y', fontsize=12, labelpad=10)
        self.ax.set_zlabel('Z', fontsize=12, labelpad=10)
        
        # Set consistent scale
        self.ax.set_box_aspect([1, 1, 1])
        self.ax.set_xlim([-2, 2])
        self.ax.set_ylim([-2, 2])
        self.ax.set_zlim([-2, 2])
        
        # Add grid
        self.ax.grid(True, linestyle='--', alpha=0.3)
        
        # Add title
        self.ax.set_title('Camera Trajectory Visualization', fontsize=14, pad=20)
        
        # Customize ticks
        self.ax.tick_params(labelsize=10)
        
        # Add legend
        self.ax.legend(['Trajectory'])
        
        # Enable mouse interaction
        self.fig.canvas.mpl_connect('button_press_event', self.on_mouse_click)
        self.fig.canvas.mpl_connect('scroll_event', self.on_scroll)
        self.fig.canvas.mpl_connect('motion_notify_event', self.on_mouse_move)
        
        # Adjust layout
        plt.tight_layout()

    def on_mouse_click(self, event):
        """Handle mouse click events for rotation"""
        if event.inaxes == self.ax:
            self.last_mouse_pos = (event.xdata, event.ydata)

    def on_scroll(self, event):
        """Handle scroll events for zooming"""
        if event.inaxes == self.ax:
            # Get current axis limits
            x_min, x_max = self.ax.get_xlim()
            y_min, y_max = self.ax.get_ylim()
            z_min, z_max = self.ax.get_zlim()
            
            # Set zoom factor
            zoom_factor = 0.9 if event.button == 'up' else 1.1
            
            # Apply zoom
            self.ax.set_xlim([x_min * zoom_factor, x_max * zoom_factor])
            self.ax.set_ylim([y_min * zoom_factor, y_max * zoom_factor])
            self.ax.set_zlim([z_min * zoom_factor, z_max * zoom_factor])
            
            plt.draw()

    def on_mouse_move(self, event):
        """Handle mouse movement for rotation"""
        if event.inaxes == self.ax and hasattr(self, 'last_mouse_pos') and event.button == 1:
            if self.last_mouse_pos:
                dx = event.xdata - self.last_mouse_pos[0]
                dy = event.ydata - self.last_mouse_pos[1]
                
                # Update view angle
                curr_elev = self.ax.elev
                curr_azim = self.ax.azim
                
                self.ax.view_init(elev=curr_elev - dy*0.5,
                                  azim=curr_azim - dx*0.5)
                
                plt.draw()
            
            self.last_mouse_pos = (event.xdata, event.ydata)

    def plot_coordinate_frame(self, R, t, scale=0.2, label=None):
        """Plot coordinate frame with rotation R and translation t"""
        colors = ['r', 'g', 'b']
        for i, color in enumerate(colors):
            axis = scale * R[:, i]
            self.ax.quiver(t[0], t[1], t[2],
                         axis[0], axis[1], axis[2],
                         color=color, alpha=0.8)
        if label:
            self.ax.text(t[0], t[1], t[2], label)

    def update_trajectory_plot(self, camera_info):
        """Update the trajectory plot with new camera pose and better styling"""
        T = camera_info["T"].detach().cpu().numpy()
        R = camera_info["R"].detach().cpu().numpy()
        
        # Store camera pose
        self.camera_positions.append(T)
        self.camera_orientations.append(R)
        
        # Clear and redraw
        self.ax.clear()
        self.setup_3d_plot()
        
        # Plot trajectory with better styling
        positions = np.array(self.camera_positions)
        if len(positions) > 1:
            self.ax.plot3D(positions[:, 0], positions[:, 1], positions[:, 2], 
                          'k--', alpha=0.5, linewidth=2, label='Camera Path')
            
            # Add points along trajectory
            self.ax.scatter(positions[:, 0], positions[:, 1], positions[:, 2], 
                           c='blue', s=30, alpha=0.5)
        
        # Plot current camera frame
        self.plot_coordinate_frame(R, T, scale=0.2, label='camera')
        
        # Maintain view angle during updates
        if hasattr(self, 'ax_elev') and hasattr(self, 'ax_azim'):
            self.ax.view_init(elev=self.ax_elev, azim=self.ax_azim)
        
        # Store current view angle
        self.ax_elev = self.ax.elev
        self.ax_azim = self.ax.azim
        
        # Update plot
        plt.draw()
        plt.pause(0.001)

    def act(self, obs_dict: Dict[str, np.ndarray]) -> np.ndarray:
        current_image = obs_dict["wrist_rgb"]
        depth_image = obs_dict["depth"]
        camera_info = obs_dict["camera_info"]

        # Handle Pygame events first
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                pygame.quit()
                cv2.destroyAllWindows()
                exit()
            # Add specific key event handling
            elif event.type == pygame.KEYDOWN:
                print(f"Key pressed: {event.key}")  # Debug print
                if event.key == pygame.K_s:
                    if not self.is_saving:
                        timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
                        self.save_dir = os.path.join('servoing_data', timestamp)
                        os.makedirs(self.save_dir, exist_ok=True)
                        print(f"Started saving images to {self.save_dir}")
                        self.is_saving = True
                        self.save_counter = 0
                elif event.key == pygame.K_q:
                    if self.is_saving:
                        print(f"Stopped saving images. Saved {self.save_counter} images.")
                        self.is_saving = False

        # Show the image
        cv2.imshow(self.window_name, cv2.cvtColor(current_image, cv2.COLOR_BGR2RGB))
        cv2.waitKey(1)

        # Initialize velocity and angular velocity
        vel = np.zeros(3)
        ang_vel = np.zeros(3)

        # Update mouse callback
        cv2.setMouseCallback(self.window_name, self.handle_cv2_mouse, 
                           param=(current_image, depth_image, camera_info))

        # Handle CV2 mouse movement
        if self.cv2_mouse_down and self.cv2_last_mouse_pos is not None:
            current_pos = np.array([self.cv2_current_x, self.cv2_current_y])
            delta = current_pos - self.cv2_last_mouse_pos
            
            if hasattr(self, 'z_mode') and self.z_mode:
                # Z-axis control (Shift + drag)
                vel[2] = -delta[1] * self.z_sensitivity  # Direct z velocity instead of accumulation
            elif hasattr(self, 'rotation_mode') and self.rotation_mode:
                # Rotation control (Ctrl + drag)
                ang_vel[0] = -delta[1] * self.rotation_sensitivity
                ang_vel[1] = delta[0] * self.rotation_sensitivity
            else:
                # Position control (drag)
                vel[0] = delta[0] * self.position_sensitivity
                vel[1] = -delta[1] * self.position_sensitivity
            
            self.cv2_last_mouse_pos = current_pos

        # Update visualization
        self.screen.fill((200, 255, 200) if self.is_saving else (255, 255, 255))
        
        # Draw mode indicator
        mode_text = "ROTATION" if pygame.key.get_mods() & pygame.KMOD_CTRL else "POSITION"
        if pygame.key.get_mods() & pygame.KMOD_ALT:
            mode_text = "POINT SELECT"
        if self.is_saving:
            mode_text += " (SAVING)"
        font = pygame.font.Font(None, 36)
        text = font.render(mode_text, True, (0, 0, 0))
        self.screen.blit(text, (10, 10))
        
        # Draw Z-axis indicator
        z_color = (0, 255, 0) if vel[2] > 0 else (255, 0, 0)
        pygame.draw.rect(self.screen, z_color, (350, 200, 20, int(vel[2] * 1000)))
        
        # Draw current velocities
        vel_text = f"Vel: {vel[0]:.3f}, {vel[1]:.3f}, {vel[2]:.3f}"
        ang_text = f"Ang: {ang_vel[0]:.3f}, {ang_vel[1]:.3f}, {ang_vel[2]:.3f}"
        vel_surface = font.render(vel_text, True, (0, 0, 0))
        ang_surface = font.render(ang_text, True, (0, 0, 0))
        self.screen.blit(vel_surface, (10, 50))
        self.screen.blit(ang_surface, (10, 90))

        # Update trajectory visualization
        self.update_trajectory_plot(obs_dict["camera_info"])
        
        pygame.display.flip()

        # Save image if saving is enabled
        if self.is_saving:
            img_path = os.path.join(self.save_dir, f'frame_{self.save_counter:06d}.png')
            cv2.imwrite(img_path, cv2.cvtColor(current_image, cv2.COLOR_BGR2RGB))
            self.save_counter += 1

        return [vel[0], -vel[1], -vel[2], 
                        ang_vel[0], ang_vel[1], ang_vel[2], self.clicked_points_3d]
        # return [0.0, 0.0, 0.0, 0.0, 0.0, 0.05, self.clicked_points_3d]

    def handle_cv2_mouse(self, event, x, y, flags, param):
        """Combined mouse handler for both movement control and point selection"""
        self.cv2_current_x = x
        self.cv2_current_y = y
        
        # Use CV2's flags instead of Pygame's key modifiers
        ctrl_pressed = flags & cv2.EVENT_FLAG_CTRLKEY
        alt_pressed = flags & cv2.EVENT_FLAG_ALTKEY
        shift_pressed = flags & cv2.EVENT_FLAG_SHIFTKEY
        
        if event == cv2.EVENT_LBUTTONDOWN:
            if alt_pressed:
                # Point selection mode
                print(f"Selected point at: {x}, {y}")
                self.clicked_points.append((x, y))
                self.mouse_clicked = True
                current_image, depth_image, camera_info = param
                self.get_point_cloud(depth_image, current_image, camera_info, self.clicked_points)
            else:
                # Movement control mode
                self.cv2_mouse_down = True
                self.cv2_last_mouse_pos = np.array([x, y])
                self.rotation_mode = ctrl_pressed
                self.z_mode = shift_pressed
        
        elif event == cv2.EVENT_LBUTTONUP:
            self.cv2_mouse_down = False
            self.cv2_last_mouse_pos = None

    def get_point_cloud(self, depth_image, current_image, camera_info, clicked_points):

        print('in get_point_cloud function')
        # Unproject the image to 3D space
        depth = depth_image.squeeze(0)
        # depth = depth.detach().cpu().numpy()

        # Get camera parameters
        T = camera_info["T"].detach().cpu().numpy()  # Translation vector
        R = camera_info["R"].detach().cpu().numpy()  # Rotation matrix
        FoVx = camera_info["FoVx"]
        FoVy = camera_info["FoVy"]
        image_width = camera_info["image_width"]
        image_height = camera_info["image_height"]
        camera_center = camera_info["camera_center"].detach().cpu().numpy()
        world_view_transform = camera_info["world_view_transform"].detach().cpu().numpy()

        # Convert to numpy and set types
        T = T.astype(np.float32)
        R = R.astype(np.float32)
        camera_center = camera_center.astype(np.float32)
        world_view_transform = world_view_transform.astype(np.float32)

        # Calculate intrinsic parameters from FOV and image dimensions
        fx = (image_width / 2) / np.tan(FoVx / 2)
        fy = (image_height / 2) / np.tan(FoVy / 2)
        cx = image_width / 2
        cy = image_height / 2

        # Create meshgrid of pixel coordinates
        y, x = np.meshgrid(np.arange(image_height), np.arange(image_width), indexing='ij')
        
        # Calculate 3D points in camera space
        z = depth
        x = (x - cx) * z / fx
        y = (y - cy) * z / fy

        # Stack coordinates
        points = np.stack([x, y, z], axis=-1)
        
        # Reshape to (N, 3)
        points = points.reshape(-1, 3)

        # Transform points to world coordinates
        homogeneous_points = np.concatenate([points, np.ones((points.shape[0], 1))], axis=1)
        print('world_view_transform', world_view_transform.shape)
        print('np.linalg.inv(world_view_transform)', np.linalg.inv(world_view_transform))
        points = (homogeneous_points @ np.linalg.inv(world_view_transform))[:, :3]

        # Create Open3D point cloud
        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(points)

        # Add colors if RGB image is available
        if current_image is not None:
            colors = current_image.reshape(-1, 3) / 255.0  # Normalize to [0, 1]
            pcd.colors = o3d.utility.Vector3dVector(colors)

        # Only calculate 3D position for the newest clicked point
        if self.mouse_clicked:
            x, y = clicked_points[-1]  # Get the latest clicked point
            depth_val = depth[y, x]
            
            # Calculate 3D position of clicked point in camera space
            X = (x - cx) * depth_val / fx
            Y = (y - cy) * depth_val / fy
            Z = depth_val
            
            # Transform to world coordinates
            click_point = np.array([X, Y, Z, 1])  # Make homogeneous
            world_point = (click_point @ np.linalg.inv(world_view_transform))[:3]  # Transform and get xyz
            self.clicked_points_3d.append(world_point)  # Store the world coordinates


        #remove points that with depth > 10
        points = pcd.points
        colors = pcd.colors
        points = np.array(points)
        colors = np.array(colors)
        colors = colors[(points[:, 2] < 10) & (points[:, 2] > -10)]
        points = points[(points[:, 2] < 10) & (points[:, 2] > -10)]
        pcd.points = o3d.utility.Vector3dVector(points)
        pcd.colors = o3d.utility.Vector3dVector(colors)


        # Create spheres for all stored 3D points
        geometries = [pcd]
        for world_point in self.clicked_points_3d:
            sphere = o3d.geometry.TriangleMesh.create_sphere(radius=0.1)
            sphere.translate(world_point)
            sphere.paint_uniform_color([1, 0, 0])
            geometries.append(sphere)

        # Visualize point cloud with spheres
        o3d.visualization.draw_geometries(geometries)

        return pcd


if __name__ == "__main__":
    demo_agent = InterfaceAgent()
    obs = {"wrist_rgb": np.zeros((1, 1, 3, 240, 320))}  # Dummy image data for testing

    running = True
    while running:
        action = demo_agent.act(obs)
        print("Action:", action)
