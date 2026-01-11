"""
Key bindings:
    'p': Pause/Resume automatic map updating
    's': Save map data and measurements
    'u/r': Undo/Redo the last update (only available when paused)
    'q': Quit the program

Processing Flow:
1. The program starts in a PAUSED state, continuously scanning and displaying LiDAR data.
2. Press 'p' to switch to RUNNING state. The first scan is captured as the initial map.
3. In RUNNING state, the program automatically captures a new scan at a set interval,
   estimates the robot's pose using DE, and updates the map.
4. Press 'p' again to switch to PAUSED state. Automatic updates stop.
5. In PAUSED state, 'u' and 'r' can be used to undo/redo map updates.
"""
import sys
import time
import os
import numpy as np
import cv2
import copy
from math import pi, cos, sin
from numba import jit

import Lidar

# --- Configuration ---
SENSOR_DIRECTION_DEGREE = 0 # 180.0  # Set sensor's forward direction (0: right, 90: up, 180: left)
UPDATE_INTERVAL_SEC = 0.5 # Interval for automatic map updates
MAP_EXPANSION_MARGIN_M = 5.0 # Margin in meters to expand the map when boundaries are reached
MAX_UNDO_HISTORY_STEPS = 10 # Limit undo history to this many steps (e.g., 100 seconds)

STATE_INITIAL = "INITIAL"       # Initial state
STATE_TARGET_CAPTURED = "TARGET_CAPTURED" # First scan (target) is captured

FONT = cv2.FONT_HERSHEY_SIMPLEX
FONT_SCALE = 1.7
FONT_COLOR = (0, 0, 0)
THICKNESS = 2

# --- Application State ---
app_state = STATE_INITIAL
map_gmap = None
robot_pose = np.identity(3)
# --- History for Undo/Redo and Saving ---
pose_history = []
map_scans_in_world = []
raw_scans_history = []
timestamps_history = []
map_gmap_history = [] # New: Store map_gmap copies
map_width_history = [] # New: Store map width
map_height_history = [] # New: Store map height
map_origin_x_history = [] # New: Store map origin x
map_origin_y_history = [] # New: Store map origin y

# --- Undo/Redo Stacks ---
redo_pose_stack = []
redo_scans_stack = []
redo_raw_scan_stack = []
redo_timestamp_stack = []
redo_map_gmap_stack = [] # New: Redo stack for map_gmap
redo_map_width_stack = [] # New: Redo stack for map width
redo_map_height_stack = [] # New: Redo stack for map height
redo_map_origin_x_stack = [] # New: Redo stack for map origin x
redo_map_origin_y_stack = [] # New: Redo stack for map origin y

class GaussianKernel:
    """Port of GaussianKernel class from slam.cpp"""
    def __init__(self, sigma, kernel_radius):
        self.radius = kernel_radius
        size = 2 * self.radius + 1
        self.kernel = np.zeros((size, size), dtype=np.float64)

        sum_val = 0.0
        sigma2 = sigma * sigma
        for y in range(-self.radius, self.radius + 1):
            for x in range(-self.radius, self.radius + 1):
                distance_sq = x*x + y*y
                weight = np.exp(-distance_sq / (2.0 * sigma2))
                self.kernel[y + self.radius, x + self.radius] = weight
                sum_val += weight
        
        self.kernel /= sum_val

def create_occupancy_grid(points, csize, width, height, origin_x, origin_y):
    """Creates a simple binary occupancy grid map from a point cloud."""
    gmap = np.zeros((height, width), dtype=np.float64)
    if points is None or len(points) == 0:
        return gmap
    #for p in points:
    #    ix = int(p[0] / csize) + origin_x
    #    iy = int(-p[1] / csize) + origin_y
    #    if 0 <= ix < width and 0 <= iy < height:
    #        gmap[iy, ix] = 1.0 # Occupied

    # 以下は上記ループ処理をnumpyで処理する
    ix_arr = ( points[:, 0] / csize).astype(int) + origin_x
    iy_arr = (-points[:, 1] / csize).astype(int) + origin_y
   
    mask_x = (ix_arr >= 0) & (ix_arr < width)
    mask_y = (iy_arr >= 0) & (iy_arr < height)
    final_mask = mask_x & mask_y
   
    valid_ix = ix_arr[final_mask]
    valid_iy = iy_arr[final_mask]
   
    gmap[valid_iy, valid_ix] = 1.0

    return gmap

@jit(nopython=True)
def gaussian_match_count(gmap, points, pose, csize, origin_x, origin_y, kernel_array, kernel_radius):
    """Port of gaussian_match_count from slam.cpp, optimized for Numba."""
    total_score = 0.0
    R = pose[:2, :2]
    t = pose[:2, 2]
    height, width = gmap.shape

    # transformed_points = (R @ points.T).T + t
    # Numba compatible transformation
    transformed_points = np.empty_like(points)
    for i in range(points.shape[0]):
        transformed_points[i, 0] = points[i, 0] * R[0, 0] + points[i, 1] * R[0, 1] + t[0]
        transformed_points[i, 1] = points[i, 0] * R[1, 0] + points[i, 1] * R[1, 1] + t[1]

    for p in transformed_points:
        gx = int(p[0] / csize) + origin_x
        gy = int(-p[1] / csize) + origin_y

        point_score = 0.0
        radius = kernel_radius
        for dy in range(-radius, radius + 1):
            for dx in range(-radius, radius + 1):
                map_x = gx + dx
                map_y = gy + dy

                if 0 <= map_x < width and 0 <= map_y < height:
                    if gmap[map_y, map_x] > 0.0:
                        point_score += kernel_array[dy + radius, dx + radius]
        total_score += point_score
    return total_score

# Note: The main DE logic is complex for Numba to optimize fully without
# more restructuring. We will JIT compile the most expensive part, which is the
# evaluation function gaussian_match_count.

def optimize_de(gmap, points, csize, origin_x, origin_y, kernel_obj, initial_pose_params=np.zeros(3)):
    """Port of optimize_de from slam.cpp"""
    # DE Parameters
    Wxy = 0.8  # Search range for x, y [m]
    Wa = np.deg2rad(30) # Search range for angle [rad]
    population_size = 200 # Increased for better search
    generations = 100     # Increased for better search
    F = 0.5  # Mutation factor
    CR = 0.2  # Crossover rate

    height, width = gmap.shape

    # 1. Initialize population around initial_pose_params
    population = np.zeros((population_size, 3))
    population[:, 0] = np.random.uniform(-Wxy, Wxy, size=population_size) + initial_pose_params[0]
    population[:, 1] = np.random.uniform(-Wxy, Wxy, size=population_size) + initial_pose_params[1]
    population[:, 2] = np.random.uniform(-Wa, Wa, size=population_size) + initial_pose_params[2]

    # Evaluate initial population
    scores = np.zeros(population_size)
    for i in range(population_size):
        R = np.array([[cos(population[i, 2]), -sin(population[i, 2])], [sin(population[i, 2]), cos(population[i, 2])]])
        t = population[i, :2]
        pose = np.vstack([np.hstack([R, t.reshape(2,1)]), [0,0,1]])
        scores[i] = gaussian_match_count(gmap, points, pose, csize, origin_x, origin_y, kernel_obj.kernel, kernel_obj.radius)

    best_idx = np.argmax(scores)
    best_pose_params = population[best_idx]
    best_eval = scores[best_idx]

    # 2. Generation loop
    for gen in range(generations):
        for i in range(population_size):
            # Mutation
            idxs = [idx for idx in range(population_size) if idx != i]
            r1, r2, r3 = np.random.choice(idxs, 3, replace=False)
            
            x1, y1, a1 = population[r1]
            x2, y2, a2 = population[r2]
            x3, y3, a3 = population[r3]

            # v = p_r1 + F * (p_r2 - p_r3)
            vx = x1 + F * (x2 - x3)
            vy = y1 + F * (y2 - y3)
            
            ax1, ay1 = cos(a1), sin(a1)
            ax2, ay2 = cos(a2), sin(a2)
            ax3, ay3 = cos(a3), sin(a3)
            vax = ax1 + F * (ax2 - ax3)
            vay = ay1 + F * (ay2 - ay3)
            va = np.arctan2(vay, vax)

            # Crossover
            trial_pose = population[i].copy()
            j_rand = np.random.randint(3)
            
            if np.random.rand() < CR or j_rand == 0: trial_pose[0] = vx
            if np.random.rand() < CR or j_rand == 1: trial_pose[1] = vy
            if np.random.rand() < CR or j_rand == 2: trial_pose[2] = va

            # Selection
            R_trial = np.array([[cos(trial_pose[2]), -sin(trial_pose[2])], [sin(trial_pose[2]), cos(trial_pose[2])]])
            t_trial = trial_pose[:2]
            pose_trial = np.vstack([np.hstack([R_trial, t_trial.reshape(2,1)]), [0,0,1]])
            eval_trial = gaussian_match_count(gmap, points, pose_trial, csize, origin_x, origin_y, kernel_obj.kernel, kernel_obj.radius)

            if eval_trial > scores[i]:
                population[i] = trial_pose
                scores[i] = eval_trial
                if eval_trial > best_eval:
                    best_eval = eval_trial
                    best_pose_params = trial_pose

    # Create final transformation matrix
    x, y, a = best_pose_params
    R_best = np.array([[cos(a), -sin(a)], [sin(a), cos(a)]])
    t_best = np.array([x, y])
    best_transform = np.vstack([np.hstack([R_best, t_best.reshape(2,1)]), [0,0,1]])
    
    return best_transform, best_eval

def hex_to_rgb(hex_color):
    hex_color = hex_color.lstrip('#')
    return tuple(int(hex_color[i:i+2], 16) for i in (0, 2, 4))

def draw_points(img, points, color, csize, origin_x, origin_y, width, height):
    rot_rad = np.deg2rad(SENSOR_DIRECTION_DEGREE)
    c, s = cos(rot_rad), sin(rot_rad)
    for px, py in points:
        px_rot = px * c - py * s
        py_rot = px * s + py * c
        ix = int(px_rot / csize + origin_x)
        iy = int(-py_rot / csize + origin_y)
        if 0 <= ix < width and 0 <= iy < height:
            cv2.circle(img, (ix, iy), 1, color, -1)

def draw_pose(img, pose, color, csize, origin_x, origin_y, radius=8, line_length=20, thickness=2):
    """Draws a pose (position and orientation) on the image."""
    rot_rad = np.deg2rad(SENSOR_DIRECTION_DEGREE)
    c, s = cos(rot_rad), sin(rot_rad)

    # Extract position and orientation from the pose matrix
    x_world = pose[0, 2]
    y_world = pose[1, 2]
    theta_rad = np.arctan2(pose[1, 0], pose[0, 0])

    # Rotate position for display
    x_rot = x_world * c - y_world * s
    y_rot = x_world * s + y_world * c

    # Rotate orientation for display
    theta_rot = theta_rad + rot_rad

    # Convert world coordinates to image coordinates
    ix = int(x_rot / csize + origin_x)
    iy = int(-y_rot / csize + origin_y)

    # Draw the center of the pose
    cv2.circle(img, (ix, iy), radius, color, -1)

    # Draw the orientation line
    line_end_x = int(ix + line_length * np.cos(theta_rot))
    line_end_y = int(iy - line_length * np.sin(theta_rot)) # Image y-axis is inverted
    cv2.line(img, (ix, iy), (line_end_x, line_end_y), color, thickness)

def save_slam_data(csize, save_dir_base="slam_save"):
    """Saves the current SLAM state to a new directory."""
    global pose_history, raw_scans_history, timestamps_history, map_gmap, map_scans_in_world, \
           map_width_history, map_height_history, map_origin_x_history, map_origin_y_history

    if not pose_history:
        print("No data to save.")
        return

    # Create unique directory
    save_dir = f"{save_dir_base}_{time.strftime('%Y%m%d-%H%M%S')}"
    try:
        os.makedirs(save_dir, exist_ok=True)
        print(f"Saving data to {save_dir}...")
    except OSError as e:
        print(f"Error creating directory {save_dir}: {e}", file=sys.stderr)
        return

    # 1. Save robot_poses.txt
    try:
        with open(os.path.join(save_dir, "robot_poses.txt"), "w") as f:
            for i in range(len(pose_history)):
                ts = int(timestamps_history[i] * 1000)
                pose_matrix = pose_history[i]
                x = pose_matrix[0, 2]
                y = pose_matrix[1, 2]
                a = np.arctan2(pose_matrix[1, 0], pose_matrix[0, 0])
                f.write(f"{ts} {x} {y} {a}\n")
    except IOError as e:
        print(f"Error writing robot_poses.txt: {e}", file=sys.stderr)

    # 2. Save laser_data.txt
    start_angle_deg = -135.0
    step_angle_deg = 0.25
    try:
        with open(os.path.join(save_dir, "laser_data.txt"), "w") as f:
            for i in range(len(raw_scans_history)):
                ts = int(timestamps_history[i] * 1000)
                raw_data = raw_scans_history[i]
                
                count = len(raw_data)
                end_angle_deg = start_angle_deg + (count - 1) * step_angle_deg
                max_echo_size = 1 # Assumed
                
                header = f"LASERSCANRT {ts} {count} {start_angle_deg} {end_angle_deg} {step_angle_deg} {max_echo_size}"
                ranges = " ".join([str(d[1]) for d in raw_data])
                
                f.write(f"{header} {ranges} {ts}\n")
    except IOError as e:
        print(f"Error writing laser_data.txt: {e}", file=sys.stderr)

    # 3. Save mapInfo.yaml
    if map_scans_in_world:
        all_points = np.vstack(map_scans_in_world)
        min_x, min_y = np.min(all_points, axis=0)
        max_x, max_y = np.max(all_points, axis=0)
        
        try:
            with open(os.path.join(save_dir, "mapInfo.yaml"), "w") as f:
                f.write(f"submap_id: 0\n") # Not using submaps
                f.write(f"csize: {csize}\n")
                f.write(f"bounds:\n")
                f.write(f"  min_x: {min_x}\n")
                f.write(f"  max_x: {max_x}\n")
                f.write(f"  min_y: {min_y}\n")
                f.write(f"  max_y: {max_y}\n")
                f.write(f"frame_count: {len(raw_scans_history)}\n")
        except IOError as e:
            print(f"Error writing mapInfo.yaml: {e}", file=sys.stderr)

    # 4. Save integrated_occMap.png
    if map_gmap is not None:
        height, width = map_gmap.shape
        occ_map_img = np.full((height, width), 128, dtype=np.uint8) # Gray for unknown
        occ_map_img[map_gmap > 0.0] = 0 # Black for occupied
        
        try:
            cv2.imwrite(os.path.join(save_dir, "integrated_occMap.png"), occ_map_img)
        except Exception as e:
            print(f"Error writing integrated_occMap.png: {e}", file=sys.stderr)

    print(f"Data saved successfully to {save_dir}")

def check_and_resize_map(current_map_gmap, all_scans_in_world, csize, current_img_origin_x, current_img_origin_y, current_width, current_height):
    """
    Checks if the map needs to be resized based on the extent of all_scans_in_world.
    If resizing is needed, creates a new larger map, copies old data, and updates origin.
    Returns (new_map_gmap, new_img_origin_x, new_img_origin_y, new_width, new_height)
    """
    if not all_scans_in_world:
        return current_map_gmap, current_img_origin_x, current_img_origin_y, current_width, current_height

    # Combine all points to find the true extent
    all_points_combined = np.vstack(all_scans_in_world)
    min_x_world, min_y_world = np.min(all_points_combined, axis=0)
    max_x_world, max_y_world = np.max(all_points_combined, axis=0)

    # Convert world coordinates to current map pixel coordinates
    min_ix = int(min_x_world / csize) + current_img_origin_x
    max_ix = int(max_x_world / csize) + current_img_origin_x
    min_iy = int(-max_y_world / csize) + current_img_origin_y # Note: y-axis is inverted in image
    max_iy = int(-min_y_world / csize) + current_img_origin_y

    # Check if any point is outside current map boundaries with a margin
    # We want to expand if points are within MAP_EXPANSION_MARGIN_M from the edge
    margin_pixels = int(MAP_EXPANSION_MARGIN_M / csize)

    expand_left = min_ix < margin_pixels
    expand_right = max_ix >= (current_width - margin_pixels)
    expand_up = min_iy < margin_pixels
    expand_down = max_iy >= (current_height - margin_pixels)

    if not (expand_left or expand_right or expand_up or expand_down):
        return current_map_gmap, current_img_origin_x, current_img_origin_y, current_width, current_height

    print("Map boundary reached, resizing map...")

    # Calculate new dimensions and origin shift
    new_width = current_width
    new_height = current_height
    offset_x = 0 # How much the old map shifts in the new map
    offset_y = 0

    # Determine expansion amount for each side
    if expand_left:
        needed_expansion = margin_pixels - min_ix
        new_width += needed_expansion
        offset_x += needed_expansion
    if expand_right:
        needed_expansion = max_ix - (current_width - margin_pixels) + 1
        new_width += needed_expansion
    if expand_up:
        needed_expansion = margin_pixels - min_iy
        new_height += needed_expansion
        offset_y += needed_expansion
    if expand_down:
        needed_expansion = max_iy - (current_height - margin_pixels) + 1
        new_height += needed_expansion

    # Update origin for the new map
    new_img_origin_x = current_img_origin_x + offset_x
    new_img_origin_y = current_img_origin_y + offset_y

    # Re-create occupancy grid from all points with the new dimensions and origin
    # This is more robust than copying and ensures all points are correctly placed.
    final_map_gmap = create_occupancy_grid(all_points_combined, csize, new_width, new_height, new_img_origin_x, new_img_origin_y)

    return final_map_gmap, new_img_origin_x, new_img_origin_y, new_width, new_height


def main():
    # Use new variables for SLAM state
    global app_state, map_gmap, robot_pose, pose_history, map_scans_in_world, \
           raw_scans_history, timestamps_history, map_gmap_history, \
           map_width_history, map_height_history, map_origin_x_history, map_origin_y_history, \
           redo_pose_stack, redo_scans_stack, redo_raw_scan_stack, redo_timestamp_stack, \
           redo_map_gmap_stack, redo_map_width_stack, redo_map_height_stack, \
           redo_map_origin_x_stack, redo_map_origin_y_stack

    urg = None
    try:
        urg = Lidar.Urg('/dev/cu.usbmodem1101', 115200)
    except Exception as e:
        print(f"Error opening LiDAR: {e}", file=sys.stderr)
        return

    start_angle, step_angle = -135.0, 0.25
    csize = 0.025
    # Map dimensions are now determined by the first scan.
    # Initialize with placeholder values until the first scan is processed.
    current_width, current_height = 400, 400
    img_origin_x, img_origin_y = 200, 200

    # Prepare Gaussian Kernel for scoring
    gaussian_kernel = GaussianKernel(sigma=0.8, kernel_radius=2)
    current_scan = None
    map_points = None # This is now a derived local variable

    # --- SLAM State Initialization ---
    robot_pose = np.identity(3)
    pose_history = []
    map_scans_in_world = []
    raw_scans_history = []
    timestamps_history = []
    map_gmap_history = []
    map_width_history = []
    map_height_history = []
    map_origin_x_history = []
    map_origin_y_history = []

    # --- Undo/Redo Stacks ---
    redo_pose_stack = []
    redo_scans_stack = []
    redo_raw_scan_stack = []
    redo_timestamp_stack = []
    redo_map_gmap_stack = []
    redo_map_width_stack = []
    redo_map_height_stack = []
    redo_map_origin_x_stack = []
    redo_map_origin_y_stack = []
    
    # --- New state for continuous mode ---
    is_paused = True # Start in paused mode
    last_update_time = 0.0

    try:
        while True:
            # --- Always get a new scan ---
            success, urg_data = urg.one_shot()
            if success:
                points = []
                for index, d in enumerate(urg_data):
                    if d[1] < 20: continue
                    angle = (index * step_angle + start_angle) * pi / 180
                    points.append([d[1] * cos(angle) / 1000.0, d[1] * sin(angle) / 1000.0])
                current_scan = np.array(points)
            
            # --- Automatic Map Update Logic ---
            time_now = time.time()
            if not is_paused and (time_now - last_update_time) > UPDATE_INTERVAL_SEC:
                if current_scan is not None and len(current_scan) > 0:
                    timestamp = time_now
                    last_update_time = time_now

                    # Clear redo stacks on new action
                    redo_pose_stack.clear()
                    redo_scans_stack.clear()
                    redo_raw_scan_stack.clear()
                    redo_timestamp_stack.clear()
                    redo_map_gmap_stack.clear()
                    redo_map_width_stack.clear()
                    redo_map_height_stack.clear()
                    redo_map_origin_x_stack.clear()
                    redo_map_origin_y_stack.clear()

                    if app_state == STATE_INITIAL:
                        print("First scan captured. Initializing map...")
                        first_scan = current_scan

                        # Calculate bounding box of the first scan
                        min_x_world, min_y_world = np.min(first_scan, axis=0)
                        max_x_world, max_y_world = np.max(first_scan, axis=0)

                        # Add margin
                        margin_m = MAP_EXPANSION_MARGIN_M
                        min_x_world -= margin_m
                        min_y_world -= margin_m
                        max_x_world += margin_m
                        max_y_world += margin_m

                        # Calculate new dimensions and origin
                        new_width = int((max_x_world - min_x_world) / csize)
                        new_height = int((max_y_world - min_y_world) / csize)
                        
                        new_origin_x = int(margin_m / csize) - int(min_x_world / csize)
                        new_origin_y = int(margin_m / csize) - int(-max_y_world / csize)

                        # Update map variables
                        current_width = new_width
                        current_height = new_height
                        img_origin_x = new_origin_x
                        img_origin_y = new_origin_y

                        # Initialize history and state
                        pose_history.append(np.identity(3))
                        map_scans_in_world.append(first_scan)
                        raw_scans_history.append(urg_data)
                        timestamps_history.append(timestamp)
                        
                        robot_pose = pose_history[-1]
                        map_points = np.vstack(map_scans_in_world)
                        map_gmap = create_occupancy_grid(map_points, csize, current_width, current_height, img_origin_x, img_origin_y)
                        
                        # Store initial map state in history
                        map_gmap_history.append(map_gmap.copy())
                        map_width_history.append(current_width)
                        map_height_history.append(current_height)
                        map_origin_x_history.append(img_origin_x)
                        map_origin_y_history.append(img_origin_y)

                        app_state = STATE_TARGET_CAPTURED
                        print(f"Initial map created with size {current_width}x{current_height}. RUNNING...")
                    
                    elif app_state == STATE_TARGET_CAPTURED:
                        source_scan = current_scan
                        print("Auto-updating... Running DE...")
                        
                        last_x, last_y, last_a = robot_pose[0, 2], robot_pose[1, 2], np.arctan2(robot_pose[1, 0], robot_pose[0, 0])
                        initial_guess = np.array([last_x, last_y, last_a])

                        best_pose_matrix, best_score = optimize_de(map_gmap, source_scan, csize, img_origin_x, img_origin_y, gaussian_kernel, initial_guess)
                        
                        transformed_source = (best_pose_matrix[:2, :2] @ source_scan.T).T + best_pose_matrix[:2, 2]
                        
                        # --- Check and Resize Map ---
                        old_height, old_width = current_height, current_width
                        
                        new_map_gmap, new_img_origin_x, new_img_origin_y, new_width, new_height = \
                            check_and_resize_map(map_gmap, map_scans_in_world + [transformed_source], csize, img_origin_x, img_origin_y, current_width, current_height)
                        
                        if new_map_gmap is not map_gmap: # Map was resized
                            # Display "Resizing..." message before blocking operation
                            temp_img = np.full((old_height, old_width, 3), 220, dtype=np.uint8)
                            cv2.putText(temp_img, "MAP RESIZING... PLEASE WAIT", (30, old_height // 2), FONT, FONT_SCALE, (0, 0, 255), THICKNESS * 2)
                            cv2.imshow("Differential Evolution Scan Matching", temp_img)
                            cv2.waitKey(1) # Ensure message is displayed

                            map_gmap = new_map_gmap
                            img_origin_x = new_img_origin_x
                            img_origin_y = new_img_origin_y
                            current_width = new_width
                            current_height = new_height
                            print(f"Map resized to {current_width}x{current_height}. New origin: ({img_origin_x}, {img_origin_y})")
                        
                        else: # No resize happened, so update current map_gmap
                            all_points_for_grid = np.vstack(map_scans_in_world + [transformed_source])
                            map_gmap = create_occupancy_grid(all_points_for_grid, csize, current_width, current_height, img_origin_x, img_origin_y)
                        # Commit new state to history
                        pose_history.append(best_pose_matrix)
                        map_scans_in_world.append(transformed_source)
                        raw_scans_history.append(urg_data)
                        timestamps_history.append(timestamp)
                        map_gmap_history.append(map_gmap.copy())
                        map_width_history.append(current_width)
                        map_height_history.append(current_height)
                        map_origin_x_history.append(img_origin_x)
                        map_origin_y_history.append(img_origin_y)

                        # Apply history limit to all stacks
                        # if len(pose_history) > MAX_UNDO_HISTORY_STEPS:
                        #     pose_history.pop(0)
                        #     map_scans_in_world.pop(0)
                        #     raw_scans_history.pop(0)
                        #     timestamps_history.pop(0)
                        #     map_gmap_history.pop(0)
                        #     map_width_history.pop(0)
                        #     map_height_history.pop(0)
                        #     map_origin_x_history.pop(0)
                        #     map_origin_y_history.pop(0)

                        robot_pose = pose_history[-1]
                        map_points = np.vstack(map_scans_in_world) # Update map_points for drawing current scan
                        
                        print(f"DE finished. Map updated. Score: {best_score:.2f}")

            # --- Drawing ---
            img = np.full((current_height, current_width, 3), 220, dtype=np.uint8)
            cv2.line(img, (0, img_origin_y), (current_width, img_origin_y), (128,128,128), 1)
            cv2.line(img, (img_origin_x, 0), (img_origin_x, current_height), (128,128,128), 1)

            # --- Drawing based on state ---
            if is_paused:
                if app_state == STATE_INITIAL:
                    status_text = "PAUSED: Press 'p' to start mapping"
                else:
                    status_text = "PAUSED: p:Resume, u:Undo, r:Redo, s:Save"
                cv2.putText(img, status_text, (30, 50), FONT, FONT_SCALE, FONT_COLOR, THICKNESS)
            else: # Running
                cv2.putText(img, "RUNNING: Press 'p' to pause", (30, 50), FONT, FONT_SCALE, FONT_COLOR, THICKNESS)

            if map_points is not None:
                draw_points(img, map_points, hex_to_rgb('#555555'), csize, img_origin_x, img_origin_y, current_width, current_height)
            
            if current_scan is not None and len(current_scan) > 0:
                # Always show current scan relative to the latest estimated pose
                transformed_current = (robot_pose[:2, :2] @ current_scan.T).T + robot_pose[:2, 2]
                draw_points(img, transformed_current, hex_to_rgb('#ff0000'), csize, img_origin_x, img_origin_y, current_width, current_height)

            for pose in pose_history:
                draw_pose(img, pose, hex_to_rgb('#00cc00'), csize, img_origin_x, img_origin_y, radius=3, line_length=8, thickness=1)

            if app_state == STATE_TARGET_CAPTURED:
                draw_pose(img, robot_pose, hex_to_rgb('#ff0000'), csize, img_origin_x, img_origin_y)
                draw_pose(img, np.identity(3), hex_to_rgb('#0000ff'), csize, img_origin_x, img_origin_y)

                x, y, a = robot_pose[0,2], robot_pose[1,2], np.rad2deg(np.arctan2(robot_pose[1,0], robot_pose[0,0]))
                pose_text = f"Robot Pose: x={x:.2f}m, y={y:.2f}m, theta={a:.1f}deg"
                cv2.putText(img, pose_text, (30, current_height - 30), FONT, FONT_SCALE, FONT_COLOR, THICKNESS)

            cv2.imshow("Differential Evolution Scan Matching", img)
            key = cv2.waitKey(10) & 0xFF

            if key == ord('q'): break
            
            # --- Key Handlers ---
            if key == ord('p'):
                is_paused = not is_paused
                if not is_paused:
                    last_update_time = time.time() # Reset timer on resume
                    print("Resuming auto-update...")
                else:
                    print("Pausing auto-update. u/r enabled.")

            elif key == ord('u') and is_paused: # UNDO (only when paused)
                if len(pose_history) > 1: # Need at least one previous state to revert to
                    # Pop from history and push to redo stacks
                    redo_pose_stack.append(pose_history.pop())
                    redo_scans_stack.append(map_scans_in_world.pop())
                    redo_raw_scan_stack.append(raw_scans_history.pop())
                    redo_timestamp_stack.append(timestamps_history.pop())
                    redo_map_gmap_stack.append(map_gmap_history.pop())
                    redo_map_width_stack.append(map_width_history.pop())
                    redo_map_height_stack.append(map_height_history.pop())
                    redo_map_origin_x_stack.append(map_origin_x_history.pop())
                    redo_map_origin_y_stack.append(map_origin_y_history.pop())

                    # Restore state from history
                    robot_pose = pose_history[-1]
                    map_scans_in_world_current = map_scans_in_world[-1] # For map_points below
                    map_gmap = map_gmap_history[-1].copy()
                    current_width = map_width_history[-1]
                    current_height = map_height_history[-1]
                    img_origin_x = map_origin_x_history[-1]
                    img_origin_y = map_origin_y_history[-1]

                    map_points = np.vstack(map_scans_in_world) # Update map_points for drawing current scan
                    
                    print("Undo successful.")
                else:
                    print("Nothing to undo.")

            elif key == ord('r') and is_paused: # REDO (only when paused)
                if redo_pose_stack: # Check if redo stack has entries
                    # Pop from redo and push to history stacks
                    pose_history.append(redo_pose_stack.pop())
                    map_scans_in_world.append(redo_scans_stack.pop())
                    raw_scans_history.append(redo_raw_scan_stack.pop())
                    timestamps_history.append(redo_timestamp_stack.pop())
                    map_gmap_history.append(redo_map_gmap_stack.pop())
                    map_width_history.append(redo_map_width_stack.pop())
                    map_height_history.append(redo_map_height_stack.pop())
                    map_origin_x_history.append(redo_map_origin_x_stack.pop())
                    map_origin_y_history.append(redo_map_origin_y_stack.pop())

                    # Restore state from history (which just had new items appended)
                    robot_pose = pose_history[-1]
                    map_scans_in_world_current = map_scans_in_world[-1] # For map_points below
                    map_gmap = map_gmap_history[-1].copy()
                    current_width = map_width_history[-1]
                    current_height = map_height_history[-1]
                    img_origin_x = map_origin_x_history[-1]
                    img_origin_y = map_origin_y_history[-1]

                    map_points = np.vstack(map_scans_in_world) # Update map_points for drawing current scan
                    
                    print("Redo successful.")
                else:
                    print("Nothing to redo.")
            
            elif key == ord('s'): # SAVE
                if app_state == STATE_TARGET_CAPTURED:
                    save_slam_data(csize=csize)
                else:
                    print("No map to save yet.")

    except KeyboardInterrupt:
        print("\nPressed Ctrl-C")
    finally:
        if urg is not None: urg.close()
        cv2.destroyAllWindows()
        print("Exiting program.")

if __name__ == '__main__':
    main()
