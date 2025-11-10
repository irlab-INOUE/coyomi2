"""
Key bindings:
    'c': LiDARデータを取り込み，地図との照合，自己位置推定，地図の更新を行う
    's': 地図データ，計測データを保存
    'u/r': 直前データのundo/redo
    'q': プログラムの終了

Processing Flow:
1. 'c' を押すと，スキャンデータをキャプチャーする．初回はとのまま地図とする
2. 再び，'c' を押すと，スキャンデータをキャプチャーし，DEによる自己位置推定を行う．
3. 推定した自己位置でスキャンデータを地図に追加する
4. 推定座標を地図上に表示する
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
# --- Undo/Redo Stacks ---
redo_pose_stack = []
redo_scans_stack = []
redo_raw_scan_stack = []
redo_timestamp_stack = []

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
    for p in points:
        ix = int(p[0] / csize) + origin_x
        iy = int(-p[1] / csize) + origin_y
        if 0 <= ix < width and 0 <= iy < height:
            gmap[iy, ix] = 1.0 # Occupied
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
    Wa = np.deg2rad(22.5) # Search range for angle [rad]
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
            cv2.circle(img, (ix, iy), 2, color, -1)

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
    global pose_history, raw_scans_history, timestamps_history, map_gmap, map_scans_in_world

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

def main():
    # Use new variables for SLAM state
    global app_state, map_gmap, robot_pose, pose_history, map_scans_in_world, \
           raw_scans_history, timestamps_history, redo_pose_stack, redo_scans_stack, \
           redo_raw_scan_stack, redo_timestamp_stack

    urg = None
    try:
        urg = Lidar.Urg('/dev/cu.usbmodem1101', 115200)
    except Exception as e:
        print(f"Error opening LiDAR: {e}", file=sys.stderr)
        return

    start_angle, step_angle = -135.0, 0.25
    height, width, csize = int(1000*1.5), int(1500*1.5), 0.025
    img_origin_x, img_origin_y = width // 2, height // 2

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
    redo_pose_stack = []
    redo_scans_stack = []
    redo_raw_scan_stack = []
    redo_timestamp_stack = []


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
            
            # --- Drawing ---
            img = np.full((height, width, 3), 220, dtype=np.uint8)
            cv2.line(img, (0, img_origin_y), (width, img_origin_y), (128,128,128), 1)
            cv2.line(img, (img_origin_x, 0), (img_origin_x, height), (128,128,128), 1)

            # --- Drawing based on state ---
            if app_state == STATE_INITIAL:
                cv2.putText(img, "Press 'c' to capture INITIAL map", (30, 50), FONT, FONT_SCALE, FONT_COLOR, THICKNESS)
                if current_scan is not None:
                    draw_points(img, current_scan, hex_to_rgb('#ff0000'), csize, img_origin_x, img_origin_y, width, height)
            
            elif app_state == STATE_TARGET_CAPTURED:
                cv2.putText(img, "c: Update, u: Undo, r: Redo, s: Save", (30, 50), FONT, FONT_SCALE, FONT_COLOR, THICKNESS)
                
                if map_points is not None:
                    draw_points(img, map_points, hex_to_rgb('#aaaaaa'), csize, img_origin_x, img_origin_y, width, height)
                
                if current_scan is not None and len(current_scan) > 0:
                    transformed_current = (robot_pose[:2, :2] @ current_scan.T).T + robot_pose[:2, 2]
                    draw_points(img, transformed_current, hex_to_rgb('#ff0000'), csize, img_origin_x, img_origin_y, width, height)

                for pose in pose_history:
                    draw_pose(img, pose, hex_to_rgb('#00cc00'), csize, img_origin_x, img_origin_y, radius=3, line_length=8, thickness=1)

                draw_pose(img, robot_pose, hex_to_rgb('#ff0000'), csize, img_origin_x, img_origin_y)
                draw_pose(img, np.identity(3), hex_to_rgb('#0000ff'), csize, img_origin_x, img_origin_y)

                x, y, a = robot_pose[0,2], robot_pose[1,2], np.rad2deg(np.arctan2(robot_pose[1,0], robot_pose[0,0]))
                pose_text = f"Robot Pose: x={x:.2f}m, y={y:.2f}m, theta={a:.1f}deg"
                cv2.putText(img, pose_text, (30, height - 30), FONT, FONT_SCALE, FONT_COLOR, THICKNESS)

            cv2.imshow("Differential Evolution Scan Matching", img)
            key = cv2.waitKey(10) & 0xFF

            if key == ord('q'): break
            
            # --- Key Handlers ---
            if key == ord('c'):
                if current_scan is None or len(current_scan) == 0: continue

                timestamp = time.time()

                if app_state == STATE_INITIAL:
                    pose_history = [np.identity(3)]
                    map_scans_in_world = [current_scan]
                    raw_scans_history = [urg_data]
                    timestamps_history = [timestamp]

                    redo_pose_stack.clear()
                    redo_scans_stack.clear()
                    redo_raw_scan_stack.clear()
                    redo_timestamp_stack.clear()
                    
                    robot_pose = pose_history[-1]
                    map_points = np.vstack(map_scans_in_world)
                    map_gmap = create_occupancy_grid(map_points, csize, width, height, img_origin_x, img_origin_y)
                    
                    app_state = STATE_TARGET_CAPTURED
                    print("Initial map captured.")
                
                elif app_state == STATE_TARGET_CAPTURED:
                    source_scan = current_scan
                    print("New scan captured. Running DE...")
                    
                    last_x, last_y, last_a = robot_pose[0, 2], robot_pose[1, 2], np.arctan2(robot_pose[1, 0], robot_pose[0, 0])
                    initial_guess = np.array([last_x, last_y, last_a])

                    best_pose_matrix, best_score = optimize_de(map_gmap, source_scan, csize, img_origin_x, img_origin_y, gaussian_kernel, initial_guess)
                    
                    transformed_source = (best_pose_matrix[:2, :2] @ source_scan.T).T + best_pose_matrix[:2, 2]
                    
                    pose_history.append(best_pose_matrix)
                    map_scans_in_world.append(transformed_source)
                    raw_scans_history.append(urg_data)
                    timestamps_history.append(timestamp)

                    redo_pose_stack.clear()
                    redo_scans_stack.clear()
                    redo_raw_scan_stack.clear()
                    redo_timestamp_stack.clear()

                    robot_pose = pose_history[-1]
                    map_points = np.vstack(map_scans_in_world)
                    map_gmap = create_occupancy_grid(map_points, csize, width, height, img_origin_x, img_origin_y)
                    
                    print(f"DE finished. Map updated. Score: {best_score:.2f}")

            elif key == ord('u'): # UNDO
                if len(pose_history) > 1:
                    redo_pose_stack.append(pose_history.pop())
                    redo_scans_stack.append(map_scans_in_world.pop())
                    redo_raw_scan_stack.append(raw_scans_history.pop())
                    redo_timestamp_stack.append(timestamps_history.pop())

                    robot_pose = pose_history[-1]
                    map_points = np.vstack(map_scans_in_world)
                    map_gmap = create_occupancy_grid(map_points, csize, width, height, img_origin_x, img_origin_y)
                    print("Undo successful.")
                else:
                    print("Nothing to undo.")

            elif key == ord('r'): # REDO
                if redo_pose_stack:
                    pose_history.append(redo_pose_stack.pop())
                    map_scans_in_world.append(redo_scans_stack.pop())
                    raw_scans_history.append(redo_raw_scan_stack.pop())
                    timestamps_history.append(redo_timestamp_stack.pop())

                    robot_pose = pose_history[-1]
                    map_points = np.vstack(map_scans_in_world)
                    map_gmap = create_occupancy_grid(map_points, csize, width, height, img_origin_x, img_origin_y)
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
