import open3d as o3d
import numpy as np
import cv2
import yaml

# Load the point cloud
pcd = o3d.io.read_point_cloud("downsampled.pcd")  # change filename

# Convert to numpy
points = np.asarray(pcd.points)
xy = points[:, :2]  # drop Z

# Set resolution and image bounds
resolution = 0.05  # meters/pixel
margin = 10  # extra padding in pixels

min_xy = np.min(xy, axis=0)
max_xy = np.max(xy, axis=0)

dims = ((max_xy - min_xy) / resolution).astype(int) + margin
grid = np.ones((dims[1], dims[0]), dtype=np.uint8) * 255  # height, width

# Map points to grid
for x, y in xy:
    ix = int((x - min_xy[0]) / resolution)
    iy = int((y - min_xy[1]) / resolution)
    if 0 <= iy < grid.shape[0] and 0 <= ix < grid.shape[1]:
        grid[iy, ix] = 0  # black = occupied

# Save the map
cv2.imwrite("map.pgm", grid)

# Write YAML metadata
map_yaml = {
    "image": "map.pgm",
    "resolution": resolution,
    "origin": [float(min_xy[0]), float(min_xy[1]), 0.0],
    "negate": 0,
    "occupied_thresh": 0.5,
    "free_thresh": 0.2
}

with open("map.yaml", "w") as f:
    yaml.dump(map_yaml, f)

print("map.pgm and map.yaml created.")
