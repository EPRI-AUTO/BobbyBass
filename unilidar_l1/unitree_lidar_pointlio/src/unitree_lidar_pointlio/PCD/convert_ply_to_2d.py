import open3d as o3d
import numpy as np
import cv2

# Load the 3D PLY file
pcd = o3d.io.read_point_cloud("scans.ply")

# Convert to numpy array
points = np.asarray(pcd.points)

# Project points onto the XY plane (ignoring Z)
x_coords = points[:, 0]
y_coords = points[:, 1]

# Normalize coordinates to fit in an image
x_min, x_max = np.min(x_coords), np.max(x_coords)
y_min, y_max = np.min(y_coords), np.max(y_coords)

img_size = 500  # Size of the output image
scale_x = (img_size - 1) / (x_max - x_min)  # -1 to keep within bounds
scale_y = (img_size - 1) / (y_max - y_min)

# Create a blank white image
img = np.ones((img_size, img_size), dtype=np.uint8) * 255

# Map points to image coordinates (clipping to bounds)
for x, y in zip(x_coords, y_coords):
    px = np.clip(int((x - x_min) * scale_x), 0, img_size - 1)
    py = np.clip(int((y - y_min) * scale_y), 0, img_size - 1)
    img[py, px] = 0  # Black for obstacles

# Save as PNG
cv2.imwrite("scans_2d.png", img)
print("2D Map saved as scans_2d.png")

