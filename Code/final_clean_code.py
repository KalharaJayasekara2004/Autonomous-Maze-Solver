from controller import Robot
from collections import deque
import math

# ==============================================================================
# 1. ROBOT SETUP & DEVICE INITIALIZATION
# ==============================================================================
robot = Robot()
timestep = int(robot.getBasicTimeStep())

# --- Motors ---
left_motor = robot.getDevice("left wheel motor")
right_motor = robot.getDevice("right wheel motor")
left_motor.setPosition(float('inf'))
right_motor.setPosition(float('inf'))
left_motor.setVelocity(0)
right_motor.setVelocity(0)

# --- Encoders ---
left_encoder = robot.getDevice("left wheel sensor")
right_encoder = robot.getDevice("right wheel sensor")
left_encoder.enable(timestep)
right_encoder.enable(timestep)

# --- Gyro ---
gyro = robot.getDevice("gyro")
gyro.enable(timestep)

# --- Distance Sensors (ps0 - ps7) ---
ps = []
for i in range(8):
    sensor = robot.getDevice(f"ps{i}")
    sensor.enable(timestep)
    ps.append(sensor)

# --- Camera ---
cam = robot.getDevice("camera")
cam.enable(timestep)

# --- LEDs ---
# Helper function defined early to allow usage in other functions if needed
def set_led(index, state):
    """
    Control an LED on the e-puck.
    index: Integer (0-9), state: Integer (1 for ON, 0 for OFF)
    """
    led = robot.getDevice(f"led{index}")
    if led:
        led.set(state)

# ==============================================================================
# 2. CONSTANTS & GLOBAL VARIABLES
# ==============================================================================
# Physical Constants
WHEEL_RADIUS = 0.0205
CELL_DISTANCE = 0.25
GYRO_SCALE = 7513.0
SPEED = 4.5

# Direction Constants
NORTH, EAST, SOUTH, WEST = 0, 1, 2, 3
dir_vectors = {
    NORTH: (0, 1),
    EAST: (-1, 0),
    SOUTH: (0, -1),
    WEST: (1, 0)
}

# State Tracking
green_wall_log = set()  # Using a set to avoid duplicate entries
maze_graph = {}         # (x,y) -> list of neighbor cells
visited = set()
path_history = []       # Breadcrumb trail

# Initial Position
current_x, current_y = 0, 0
current_dir = NORTH
visited.add((current_x, current_y))

# ==============================================================================
# 3. LOW LEVEL CONTROL & HELPERS
# ==============================================================================
def stop():
    left_motor.setVelocity(0)
    right_motor.setVelocity(0)

def wait(seconds):
    """
    Pauses robot logic for 'seconds' while keeping simulation running.
    """
    start_time = robot.getTime()
    while robot.step(timestep) != -1:
        if robot.getTime() - start_time >= seconds:
            break

def get_wall_centering_correction():
    # Define a threshold to determine if a wall is actually there
    # User logic: > 1345 is "Free", so < 1200 means a wall is likely present
    WALL_EXIST_THRESHOLD = 510
    
    # Read raw sensor values
    left_dist = ps[5].getValue()
    right_dist = ps[2].getValue()

    # Only correct if BOTH walls are visible (Corridor Mode)
    if left_dist < WALL_EXIST_THRESHOLD or right_dist < WALL_EXIST_THRESHOLD:
        # Calculate Error: Difference between left and right spacing
        # Note: Lower Value = Closer Wall
        error = left_dist - right_dist
        
        # P-Controller Gain for Walls
        Kp_wall = 0.000003
        
        # Calculate correction
        correction = -Kp_wall * error
        return correction
        
    return 0

# ==============================================================================
# 4. SENSOR CHECKS
# ==============================================================================
def front_free():
    return (ps[0].getValue() + ps[7].getValue()) / 2 > 1345

def left_free():
    return ps[5].getValue() > 1345

def right_free():
    return ps[2].getValue() > 1345

def back_free():
    return (ps[3].getValue() + ps[4].getValue()) / 2 > 1345

# ==============================================================================
# 5. CORE MOVEMENT FUNCTIONS
# ==============================================================================
def move_forward_cell():
    rotation_needed = CELL_DISTANCE / WHEEL_RADIUS
    left_start = left_encoder.getValue()
    right_start = right_encoder.getValue()

    # Gyro Gain
    Kp_gyro = 1.25 
    
    left_motor.setVelocity(SPEED)
    right_motor.setVelocity(SPEED)

    while robot.step(timestep) != -1:
        left_travel = left_encoder.getValue() - left_start
        right_travel = right_encoder.getValue() - right_start
        avg_travel = (left_travel + right_travel) / 2

        if avg_travel >= rotation_needed:
            break

        # 1. Gyro Correction (Keep Heading Straight)
        raw_gyro = gyro.getValues()[2]
        rad_s = raw_gyro / GYRO_SCALE
        gyro_correction = -Kp_gyro * rad_s
        
        # 2. Wall Correction (Keep Centered in Lane)
        wall_correction = get_wall_centering_correction()

        # Combine corrections
        total_correction = gyro_correction + wall_correction

        # Apply to motors
        left_motor.setVelocity(SPEED + total_correction)
        right_motor.setVelocity(SPEED - total_correction)

    stop()
    # Stabilization pause
    for _ in range(5):
        robot.step(timestep)

def rotate_left_90(speed=2.0):
    angle = 0
    target = math.pi / 2
    left_motor.setVelocity(-speed)
    right_motor.setVelocity(speed)

    last_time = robot.getTime()
    while robot.step(timestep) != -1:
        dt = robot.getTime() - last_time
        last_time = robot.getTime()
        angle += abs(gyro.getValues()[2] / GYRO_SCALE) * dt
       
        if angle >= target:
            break

    stop()

def rotate_right_90(speed=2.0):
    angle = 0
    target = math.pi / 2
    left_motor.setVelocity(speed)
    right_motor.setVelocity(-speed)
    
    # Capture sensor state at start of turn
    left_dist = ps[7].getValue()
    left02_dist = ps[6].getValue()
    right_dist = ps[0].getValue()
    right02_dist = ps[2].getValue()

    last_time = robot.getTime()
    while robot.step(timestep) != -1:
        dt = robot.getTime() - last_time
        last_time = robot.getTime()
        angle += abs(gyro.getValues()[2] / GYRO_SCALE) * dt
        
        # --- Camera Logic during Right Rotation ---
        img = cam.getImage()
        if img:
            w, h = cam.getWidth(), cam.getHeight()
            
            # Using specific pixel coordinates
            r = cam.imageGetRed(img, w, w-8, h-9)
            g = cam.imageGetGreen(img, w, w-8, h-9)
            b = cam.imageGetBlue(img, w, w-8, h-9)
            
            # Trigger exactly when g > 180 (calibrated distance of 0.2m)
            # And checking specific wall distance condition
            if (g > 180 and g > r*3) and g > b*4 and ((left_dist + right02_dist) / 2 > 1150):
                
                # Calculate where the wall is relative to current position
                wall_x = current_x
                wall_y = current_y 
                set_led(0, 1)
                
                # Store and Print if new
                if (wall_x, wall_y) not in green_wall_log:
                    green_wall_log.add((wall_x, wall_y))
                    print(f"!!! GREEN WALL DETECTED !!!")
                    print(f"<<<< While Rotating Right>>>>")
                    print(f"Wall Location:  ({wall_x}, {wall_y})")
                    print(f"Sensor Values:  R:{r} G:{g} B:{b}")
                    print("-" * 30)
            else:
                set_led(0, 0)
        
        if angle >= target:
            break

    stop()

def turn_to(target_dir, current_dir):
    diff = (target_dir - current_dir) % 4
    if diff == 1:
        rotate_right_90()
    elif diff == 3:
        rotate_left_90()
    elif diff == 2:
        rotate_right_90()
        rotate_right_90()

# ==============================================================================
# 6. NAVIGATION & PATHFINDING
# ==============================================================================
def bfs_path(start, goal, graph):
    queue = deque([start])
    came_from = {start: None}

    while queue:
        current = queue.popleft()
        if current == goal:
            break

        for neighbor in graph.get(current, []):
            if neighbor not in came_from:
                came_from[neighbor] = current
                queue.append(neighbor)

    if goal not in came_from:
        return None  # No path

    # Reconstruct path
    path = []
    cur = goal
    while cur:
        path.append(cur)
        cur = came_from[cur]
    path.reverse()
    return path

def direction_from_to(a, b):
    x1, y1 = a
    x2, y2 = b
    if x2 == x1 and y2 == y1 + 1:
        return NORTH
    if x2 == x1 and y2 == y1 - 1:
        return SOUTH
    if x2 == x1 - 1 and y2 == y1:
        return EAST
    if x2 == x1 + 1 and y2 == y1:
        return WEST
    return None

def navigate_to_target(final_x, final_y):
    global current_x, current_y, current_dir

    start = (current_x, current_y)
    goal = (final_x, final_y)

    path = bfs_path(start, goal, maze_graph)

    if not path:
        print("No valid path to target!")
        return

    print("Planned Path:", path)

    for i in range(len(path) - 1):
        cur_cell = path[i]
        next_cell = path[i + 1]

        target_dir = direction_from_to(cur_cell, next_cell)
        if target_dir is None:
            print("Direction error!")
            return

        turn_to(target_dir, current_dir)
        move_forward_cell()

        current_x, current_y = next_cell
        current_dir = target_dir

        print(f"Reached {current_x}, {current_y}")

# ==============================================================================
# 7. MAIN EXECUTION: DFS EXPLORATION
# ==============================================================================
while True:
    robot.step(timestep)
    
    # --- Main Loop Camera Check ---
    img = cam.getImage()
    if img:
        w, h = cam.getWidth(), cam.getHeight()
        
        # Using specific pixel coordinates
        r = cam.imageGetRed(img, w, w // 16, h // 16)
        g = cam.imageGetGreen(img, w, w // 16, h // 16)
        b = cam.imageGetBlue(img, w, w // 16, h // 16)
        
        # Trigger exactly when g > 180 (calibrated distance)
        if (g > 180 and g > r*4) and g > b*4:
            dx, dy = dir_vectors[current_dir]
            wall_x = current_x
            wall_y = current_y 
            set_led(0, 1)
            
            # Store and Print if new
            if (wall_x, wall_y) not in green_wall_log:
                green_wall_log.add((wall_x, wall_y))
                print(f"!!! GREEN WALL DETECTED !!!")
                print(f"Wall Location:  ({wall_x}, {wall_y})")
                print(f"Sensor Values:  R:{r} G:{g} B:{b}")
                print("-" * 30)
        else:
            set_led(0, 0)
    
    # --- DFS Strategy ---
    # 1. Look for a valid, UNVISITED neighbor
    best_next_step = None
    
    # Priority: Front, Left, Right, Back
    potential_moves = [
        (0, front_free),  
        (3, left_free),   
        (1, right_free),  
        (2, back_free)    
    ]

    for rel, check_func in potential_moves:
        if check_func():
            abs_dir = (current_dir + rel) % 4
            dx, dy = dir_vectors[abs_dir]
            nx, ny = current_x + dx, current_y + dy
            
            # CRITICAL: Only move if the cell has NEVER been visited
            if (nx, ny) not in visited:
                best_next_step = (abs_dir, nx, ny)
                break 

    # 2. Execution Logic
    if best_next_step:
        # EXPLORE FORWARD
        next_dir, nx, ny = best_next_step
        
        # Save current state to history BEFORE moving
        path_history.append((current_x, current_y, current_dir))
            
        turn_to(next_dir, current_dir)
        
        move_forward_cell()
        current_x, current_y = nx, ny
        current_dir = next_dir
        visited.add((current_x, current_y))
        print(f"Moving to new cell: {current_x}, {current_y}")
        
        # Register bidirectional connection in graph
        maze_graph.setdefault((current_x, current_y), [])
        
        px, py, _ = path_history[-1]
        maze_graph.setdefault((px, py), [])
        
        maze_graph[(px, py)].append((current_x, current_y))
        maze_graph[(current_x, current_y)].append((px, py))
        
    elif path_history:
        # BACKTRACKING LOGIC
        # 1. Retrieve the state of the cell we are moving BACK TO
        prev_x, prev_y, prev_dir = path_history.pop()
        
        print(f"Dead end! Backtracking from ({current_x},{current_y}) to ({prev_x},{prev_y})")
        
        # 2. Calculate direction needed to move backward
        move_back_dir = (current_dir + 2) % 4 
        
        # 3. Physically move
        turn_to(move_back_dir, current_dir) 
        move_forward_cell()
        
        # --- Backtracking Camera Check ---
        img = cam.getImage()
        if img:
            w, h = cam.getWidth(), cam.getHeight()
            
            # Using specific pixel coordinates for backtracking
            r = cam.imageGetRed(img, w, w//2 , h//2 )
            g = cam.imageGetGreen(img, w, w//2, h//2 )
            b = cam.imageGetBlue(img, w, w//2 , h//2)
            
            # Trigger with specific thresholds
            if (g > 180 and g > r*3 ) and g > b*4 and r > 27 and g > 38:
                wall_x = current_x
                wall_y = current_y 
                set_led(0, 1)
                
                # Store and Print if new
                if (wall_x, wall_y) not in green_wall_log:
                    green_wall_log.add((wall_x, wall_y))
                    print(f"!!! GREEN WALL DETECTED !!!")
                    print(f"<<<< Back tracking>>>>")
                    print(f"Wall Location:  ({wall_x}, {wall_y})")
                    print("-" * 30)
            else:
                set_led(0, 0)
        
        # 4. COORDINATE FIX: Reset the robot's internal state
        current_x, current_y = prev_x, prev_y
        
        # Turn robot back to its ORIGINAL orientation for this cell
        turn_to(prev_dir, move_back_dir) 
        current_dir = prev_dir
        
        print(f"Back at ({current_x},{current_y}), orientation reset to {current_dir}")
        
    else:
        # No unvisited neighbors and no more history = Back at start and finished
        break

# ==============================================================================
# 8. FINAL TARGET CALCULATION & NAVIGATION
# ==============================================================================
print("EXPLORATION COMPLETE")

sum_x = sum(coord[0] for coord in green_wall_log)
sum_y = sum(coord[1] for coord in green_wall_log)
final_x = sum_x % 12
final_y = sum_y % 12

final_target_x = final_x
final_target_y = final_y

print("="*40)
print(f"Target: ({final_x}, {final_y})")
print("="*40)

print("Navigating to Final Target...")
navigate_to_target(final_target_x, final_target_y)

print("FINAL TARGET REACHED")
set_led(0, 1)
wait(5.0)

stop()