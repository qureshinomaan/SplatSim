# Importing required libraries
import open3d as o3d
import numpy as np
from sklearn.neighbors import KNeighborsClassifier
from sklearn.model_selection import train_test_split
from sklearn.metrics import accuracy_score
import matplotlib.pyplot as plt

# Step 1: Import your function
from get_segmented import get_segmented_pcd

# Step 2: Get segmented points and labels
segmented_points, _, idxs = get_segmented_pcd()

# Step 3: Combine all points into a single array and create labels
X = np.vstack(segmented_points)
y = np.hstack([[i] * len(points) for i, points in enumerate(segmented_points)])


# Step 4: Split the data into training and testing sets
X_train, X_test, y_train, y_test = train_test_split(X, y, test_size=0.01, random_state=42)

# Step 5: Create and train the KNN classifier
knn = KNeighborsClassifier(n_neighbors=10)  # You can adjust the number of neighbors
knn.fit(X_train, y_train)

# Step 6: Make predictions on the test set
y_pred = knn.predict(X_test)

#save knn model
import joblib
joblib.dump(knn, 'knn_model.pkl')

# Step 7: Evaluate the classifier
accuracy = accuracy_score(y_test, y_pred)
print(f"Accuracy: {accuracy}")

# Optionally: Print predictions and actual labels
print("Predictions:", y_pred)
print("Actual labels:", y_test)

# Step 8: Assign random colors to each label
num_classes = len(np.unique(y))
colors = [[0, 0, 0], [0.9, 0.9, 0.9], [0.0, 0.4, 0.9], [0.4, 0.9, 0.0], [0.9, 0.0, 0.4], [0.5, 0.5, 0.5], [0.0, 0.9, 0.5], [0.5, 0.0, 0.9], [0.9, 0.5, 0.0], [0.0, 0.5, 0.1], [0.1, 0.0, 0.5], [0.4, 0.8, 0.9], [0.5, 0.1, 0.0], [0.8, 0.9, 0.4], [0.9, 0.4, 0.8], [0.0, 0.0, 0.9]]
# random colors
# colors = [[np.random.rand() for _ in range(3)] for _ in range(19)]

label_colors = colors
y_pred = knn.predict(X)
y = y_pred

# Step 9: Create an Open3D point cloud
pcd = o3d.geometry.PointCloud()
pcd.points = o3d.utility.Vector3dVector(X)

# Step 10: Color the points according to their labels
pcd_colors = np.array([label_colors[int(label)] for label in y])
print('pcd_colors:', pcd_colors.shape)
pcd.colors = o3d.utility.Vector3dVector(pcd_colors)

# Step 11: Visualize the point cloud
o3d.visualization.draw_geometries([pcd], window_name="Point Cloud", width=800, height=600)
