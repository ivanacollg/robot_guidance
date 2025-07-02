import cv2
import numpy as np
import math


def meters_to_pixels(pt_m, map_origin, resolution, map_height_m, border_thickness_m):
    """
    Convert real-world coordinates (in meters) to image pixel coordinates.
    
    Args:
        x_m (float): X coordinate in meters.
        y_m (float): Y coordinate in meters.
        map_origin (tuple): (x_origin, y_origin, yaw) in meters. Yaw is ignored.
        resolution (float): Map resolution in meters per pixel.
        height_px (int): Total height of the map in pixels (including borders).
        border_px (int): Border thickness in pixels.

    Returns:
        (int, int): (x_px, y_px) coordinates in pixel space.
    """
    x_px = math.floor((pt_m[0] - map_origin[0]) / resolution)
    y_px = math.floor((map_height_m + 2*border_thickness_m + map_origin[1] - pt_m[1]) / resolution)
    
    return (x_px, y_px)

# ----------------------------
# Map Parameters (in meters)
# ----------------------------
map_width_m = 29.10
map_height_m = 4.95
border_thickness_m = 0.3
resolution = 0.05  # meters per pixel
map_origin = (-5.3, -4.75, 0.0)  # user-defined map origin: (x, y, yaw) # Yaw is not taken into account

# ----------------------------
# Obstacle Parameters (in meters)
# ----------------------------
obstacle_width_m = 0.6
obstacle_height_m = 0.6
obstacle_origin_m = (19.5, -1.5)  # (x, y) bottom-right corner

# ----------------------------
# Fence Parameters (in meters)
# ----------------------------
fence_width_m = 1.8288 # Single fence panel
fence_offset = 0.45
fence_angle = 20
fence1_origin_m = (0.0, -4.45)  # (x, y) bottom-right corner
fence2_origin_m = (10.0, -4.45)  # (x, y) bottom-right corner

# ----------------------------
# Compute pixel dimensions
# ----------------------------
width_px = math.ceil(map_width_m / resolution)
height_px = math.ceil(map_height_m / resolution)
border_px = math.ceil(border_thickness_m / resolution)

print(f"Map width: {map_width_m} m => {width_px} px")
print(f"Map height: {map_height_m} m => {height_px} px")
print(f"Border thickness: {border_thickness_m} m => {border_px} px")

# Add border to full size
width_px += (border_px*2)
height_px += (border_px*2)

print(f"Map width with border: {width_px} px")
print(f"Map height with border: {height_px} px")

# ----------------------------
# Create base map
# ----------------------------
image = np.ones((height_px, width_px), dtype=np.uint8) * 255  # white background
# Add border
# Top border
image[0:border_px, :] = 0
# Bottom border
image[-border_px:, :] = 0
# Left border
image[:, 0:border_px] = 0
# Right border
image[:, -border_px:] = 0

# ----------------------------
# Rotate map 180 degrees to get approriate orientation-> y back, x right, z up 
# ----------------------------
image = cv2.rotate(image, cv2.ROTATE_180)

# ----------------------------
# Convert obstacle dimensions to pixels
# ----------------------------
ob_w_px = math.ceil(obstacle_width_m / resolution)
ob_h_px = math.ceil(obstacle_height_m / resolution)
ob_x_m, ob_y_m = obstacle_origin_m


ob_x_px = math.floor((ob_x_m-map_origin[0]) / resolution)
ob_y_px = math.floor((map_height_m + 2*border_thickness_m + map_origin[1] - ob_y_m) / resolution)

# Bottom right corner
ob_top_left = (ob_x_px + ob_w_px, ob_y_px + ob_h_px) 
if ob_x_m > 1.0:
    ob_bottom_right = (ob_x_px, ob_y_px+1) # plus one is an opencv offset 
else:
    ob_bottom_right = (ob_x_px+1, ob_y_px+1) # plus one is an opencv offset 
print(f"Drawing obstacle at: top-left {ob_top_left}, bottom-right {ob_bottom_right} (in px)")

# ----------------------------
# Draw obstacle (black rectangle)
# ----------------------------
cv2.rectangle(image, ob_top_left, ob_bottom_right, color=0, thickness=-1)


#------------------------------
# Draw Irregular polynomials
# -----------------------------
fence_angle = math.radians(fence_angle) # turn to radians
dx = fence_width_m*math.cos(fence_angle)
dy = fence_width_m*math.sin(fence_angle)

# Corners
p2 = (fence1_origin_m[0] + 2*dx, fence1_origin_m[1])
p3 = (fence1_origin_m[0] + 2*dx, fence1_origin_m[1] + fence_offset )
p4 = (fence1_origin_m[0] + dx, fence1_origin_m[1] + fence_offset + dy)
p5 = (fence1_origin_m[0], fence1_origin_m[1] + fence_offset )

p1_px = meters_to_pixels(fence1_origin_m, map_origin, resolution, map_height_m, border_thickness_m)
p2_px = meters_to_pixels(p2, map_origin, resolution, map_height_m, border_thickness_m)
p3_px = meters_to_pixels(p3, map_origin, resolution, map_height_m, border_thickness_m)
p4_px = meters_to_pixels(p4, map_origin, resolution, map_height_m, border_thickness_m)
p5_px = meters_to_pixels(p5, map_origin, resolution, map_height_m, border_thickness_m)

print(f"Drawing fence 1: origin {fence1_origin_m}, p2 {p2}, p3 {p3}, p4 {p4}, p5 {p5}")
print(f"Drawing fence 1 at: origin {p1_px}, p2 {p2_px}, p3 {p3_px}, p4 {p4_px}, p5 {p5_px}")

# Define the vertices of a triangle
fence_courners = np.array([p1_px, p2_px, p3_px, p4_px, p5_px], dtype=np.int32)

# Fill the triangle with blue color
cv2.fillPoly(image, pts=[fence_courners], color=0)

# Corners
p2 = (fence2_origin_m[0] + 2*dx, fence2_origin_m[1])
p3 = (fence2_origin_m[0] + 2*dx, fence2_origin_m[1] + fence_offset )
p4 = (fence2_origin_m[0] + dx, fence2_origin_m[1] + fence_offset + dy)
p5 = (fence2_origin_m[0], fence2_origin_m[1] + fence_offset )

p1_px = meters_to_pixels(fence2_origin_m, map_origin, resolution, map_height_m, border_thickness_m)
p2_px = meters_to_pixels(p2, map_origin, resolution, map_height_m, border_thickness_m)
p3_px = meters_to_pixels(p3, map_origin, resolution, map_height_m, border_thickness_m)
p4_px = meters_to_pixels(p4, map_origin, resolution, map_height_m, border_thickness_m)
p5_px = meters_to_pixels(p5, map_origin, resolution, map_height_m, border_thickness_m)

print(f"Drawing fence 1: origin {fence2_origin_m}, p2 {p2}, p3 {p3}, p4 {p4}, p5 {p5}")
print(f"Drawing fence 1 at: origin {p1_px}, p2 {p2_px}, p3 {p3_px}, p4 {p4_px}, p5 {p5_px}")

# Define the vertices of a triangle
fence_courners = np.array([p1_px, p2_px, p3_px, p4_px, p5_px], dtype=np.int32)

# Fill the triangle with blue color
cv2.fillPoly(image, pts=[fence_courners], color=0)


# ----------------------------
# Save image and metadata
# ----------------------------
cv2.imwrite("../maps/map.pgm", image)

cv2.imwrite("../maps/map.png", image)

origin_str = f"[{map_origin[0]}, {map_origin[1]}, {map_origin[2]}]"
yaml_content = f"""image: map.pgm
resolution: {resolution}
origin: {origin_str}
negate: 0
occupied_thresh: 0.65
free_thresh: 0.196
"""

with open("../maps/map.yaml", "w") as f:
    f.write(yaml_content)

print("Map image with obstacle and YAML file (with origin) generated.")