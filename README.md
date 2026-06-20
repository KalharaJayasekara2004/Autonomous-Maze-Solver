# ROARZ 2025 Autonomous Maze Solver

## Project Overview

This project was developed for the ROARZ 2025 robotics challenge using the Webots simulation environment.

The objective was to autonomously explore an unknown maze, identify and record the coordinates of green walls, calculate a final destination based on the detected wall coordinates, and navigate to that destination without any prior knowledge of the maze layout.

The robot performs complete maze exploration using the Depth First Search (DFS) algorithm while simultaneously building an internal map of the environment. After exploration is completed, the robot calculates the target location and navigates to it using shortest-path planning.

---

## Key Features

* Autonomous maze exploration
* Complete maze coverage using DFS
* Green wall detection using camera-based color recognition
* Coordinate mapping of detected green walls
* Automatic maze graph generation
* Target coordinate calculation
* Shortest path planning using BFS
* Autonomous navigation to the final destination
* Wall-centering and heading correction
* Webots simulation implementation

---

## System Workflow

### Phase 1: Maze Exploration

The robot starts from the initial position and explores the maze using a Depth First Search (DFS) strategy.

During exploration:

* Unvisited neighboring cells are prioritized.
* The robot records visited locations.
* Dead ends are handled through backtracking.
* A graph representation of the maze is created.
* Coordinate positions are continuously updated.

### Phase 2: Green Wall Detection

A camera mounted on the robot continuously scans the environment.

When a green wall is detected:

* RGB values are analyzed.
* Green color thresholds are applied.
* The wall coordinate is recorded.
* Duplicate detections are ignored.

Detected wall locations are stored for later processing.

### Phase 3: Target Calculation

After full maze exploration, all detected green wall coordinates are processed.

The target location is calculated using the recorded wall coordinates according to the competition requirements.

The final destination coordinate is then generated.

### Phase 4: Path Planning

The generated maze graph is used for path planning.

Breadth First Search (BFS) is used to:

* Determine the shortest route
* Avoid unnecessary traversal
* Generate a sequence of navigation steps

### Phase 5: Autonomous Navigation

The robot follows the planned path and navigates to the final target location.

Upon reaching the destination, the mission is successfully completed.

---

## Algorithms Used

### Depth First Search (DFS)

Used for:

* Complete maze exploration
* Visiting all reachable cells
* Building the maze map
* Recording traversal history

### Breadth First Search (BFS)

Used for:

* Shortest path calculation
* Efficient navigation to the target

---

## Sensor Systems

### Distance Sensors

Used for:

* Wall detection
* Obstacle avoidance
* Corridor navigation

### Camera

Used for:

* Green wall detection
* RGB color analysis

### Wheel Encoders

Used for:

* Position estimation
* Cell-to-cell movement control

### Gyroscope

Used for:

* Heading stabilization
* Rotation measurement
* Direction correction

---

## Motion Control

The robot incorporates:

### Gyroscope-Based Correction

Maintains straight movement by minimizing heading errors.

### Wall-Centering Control

Maintains equal distance from surrounding walls while moving through corridors.

### Cell-Based Navigation

Movement is performed one maze cell at a time for accurate localization.

---

## Software and Tools

* Webots Simulator
* Python
* E-puck Robot Model
* DFS Algorithm
* BFS Algorithm

---

## Results

The developed system successfully:

* Explored the complete maze
* Detected green walls
* Recorded wall coordinates
* Constructed a maze graph
* Calculated the target location
* Generated the shortest path
* Reached the final destination autonomously

---

## Author

Kalhara Jayasekara

Department of Electrical and Electronic Engineering

University of Peradeniya

---

