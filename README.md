# RAS 545 — Vision-Guided Maze Solving with the MyCobot Pro 600

**Arizona State University | Robotics I (RAS 545)**

**Project Type:** Graduate Robotics Engineering — Team Final Project

**Contributors:** Phillip Hogan, Yogesh Salvi, Ashwat Rajbhandari

**Technologies:** Python, OpenCV, NumPy, MATLAB, Simulink, Computer Vision, Graph Search, Inverse Kinematics, Robot Motion Planning

---

## Project Overview

This project develops a vision-guided robotic system capable of identifying a maze, computing a collision-free path, transforming image coordinates into robot workspace coordinates, and commanding a MyCobot Pro 600 robotic manipulator to traverse the maze.

The original course project integrated computer vision, A* search, MATLAB, a Simulink digital twin, and physical robot execution.

Following the course demonstration, the Python solver was independently extended with a Dijkstra-based path-planning implementation.

The repository preserves the engineering artifacts and distinguishes the original physically demonstrated system from the subsequently updated offline solver.

## Project Demonstration

### Physical Robot Validation

The original team implementation successfully demonstrated maze traversal using the MyCobot Pro 600.

**[Watch the physical robot demonstration](media/mycobot_pro600_physical_validation.mp4)**

### Final Engineering Report

**[Read the RAS 545 Final Project Report](docs/RAS545_Final_Project_Report.pdf)**

The report documents the original A*-based implementation, methodology, simulation, physical execution, and reported experimental results.

### Updated Solver Visualization

![Maze path planning with ten numbered waypoints](ras545-final-maze-segments.png)

The visualization shows the subsequently updated solver's ten waypoints on the bundled maze image.

---

## Engineering Objectives

The project addresses five interconnected robotics engineering problems:

1. Detect maze boundaries and start/goal locations using computer vision.
2. Construct a traversable representation of the maze.
3. Calculate a path between the start and goal.
4. Transform image-space waypoints into physical robot coordinates.
5. Validate and execute robot movement using MATLAB, Simulink, inverse kinematics, and the MyCobot Pro 600.

## System Architecture

The original project followed this general workflow:

```text
Camera / Maze Image
        |
        v
OpenCV Image Processing
        |
        v
Start / Goal Detection
        |
        v
Maze Representation
        |
        v
A* Path Planning
        |
        v
Pixel-to-Workspace Coordinate Mapping
        |
        v
MATLAB Integration
        |
        v
Simulink Digital Twin
        |
        v
Inverse Kinematics
        |
        v
MyCobot Pro 600
        |
        v
Physical Maze Traversal
```

The updated standalone Python solver uses Dijkstra-based path planning in place of the original A* search.

## Computer Vision

The vision-processing workflow identifies maze geometry and navigation targets from a camera image.

Relevant techniques include:

- OpenCV image processing.
- HSV-based color segmentation.
- Detection of red start and green goal markers.
- Wall and free-space classification.
- Pixel-coordinate extraction.
- Conversion of image coordinates into physical workspace coordinates.

The resulting representation supports graph-based path planning and robot waypoint generation.

## Path Planning

### Original Implementation — A* Search

The graded course project used A* search with Manhattan distance as its heuristic.

The original implementation computed a path through the maze and supplied coordinates for downstream robot execution.

The team validated this workflow through simulation and a physical robot demonstration.

### Subsequent Implementation — Dijkstra Search

Following the course project, Phillip Hogan independently updated the Python solver to use Dijkstra-based graph search.

The updated implementation includes:

- Priority-queue-based search.
- Accumulated traversal costs.
- Four-connected grid navigation.
- Clearance-aware traversal penalties.
- Obstacle processing.
- Waypoint interpolation.
- Generation of ten XY waypoints.

This implementation is represented by the current `maze_solver.py` file.

### Validation Scope

The updated solver was tested offline using the bundled maze image.

The verification confirmed that:

- Ten finite XY waypoints were generated.
- The waypoints remained within image bounds.
- The visualized path connected the detected start and goal.
- No wall crossings were visible in the inspected overlay.

These checks do not establish comprehensive robustness across different mazes.

**The updated Dijkstra implementation has not been confirmed through a new physical robot demonstration.**

## MATLAB, Simulink, and Robot Control

The original project integrated Python-generated maze waypoints with MATLAB and a Simulink digital twin.

The engineering workflow included:

- Importing path-planning outputs into MATLAB.
- Mapping image coordinates to the robot workspace.
- Evaluating robot motion in the digital twin.
- Applying inverse kinematics.
- Checking motion feasibility.
- Sending commands to the physical robot.

The repository includes the MATLAB control script, Simulink model, model parameters, and CAD assets.

The current MATLAB integration and physical execution have not been revalidated against the subsequently updated Python solver.

## Reported Experimental Results

The original team report records the following results:

| Metric | Reported Result |
|---|---|
| Path-planning method | A* search |
| Planned path | 32 segments |
| Simulink execution | 28 seconds |
| Cartesian endpoint error | ≤ 1.8 mm |
| Total path length | 450 mm |
| Physical robot traversal | Completed without reported wall contact |
| Maze samples | Three |
| Reported trial duration | Under 60 seconds per sample |

These figures are **results reported in the original team project report**, not measurements independently reproduced for the updated Dijkstra implementation.

## Team Contributions

### Phillip Hogan

Contributed to the computer-vision and robotics-integration workflow, including:

- Researching and developing OpenCV-based maze and marker detection.
- Working with HSV segmentation, thresholding, and image coordinates.
- Investigating pixel-to-workspace coordinate transformations.
- Debugging waypoint generation and coordinate mapping.
- Supporting Python-to-MATLAB integration.
- Assisting with inverse kinematics, digital-twin testing, and robot execution.
- Participating in physical testing, troubleshooting, and report development.
- Independently developing the subsequent Dijkstra-based solver update.

### Yogesh Salvi

Contributed extensively to the team's robotics engineering and integration work, including the digital twin, inverse kinematics, MATLAB integration, robot control, testing, and troubleshooting.

### Ashwat Rajbhandari

Contributed to code development and troubleshooting during the team project.

The original course deliverable was a collaborative team effort.

## Repository Structure

```text
ras545-final-mycobot-maze-solver/
|
|-- README.md
|-- maze_solver.py
|-- maze_generator_9.py
|-- final.m
|-- cobot600.slx
|-- cobot600_DataFile.m
|-- cobot600.xml
|-- unsolved_maze.png
|-- unsolved_maze2.png
|-- ras545-final-maze-segments.png
|
|-- docs/
|   `-- RAS545_Final_Project_Report.pdf
|
|-- media/
|   `-- mycobot_pro600_physical_validation.mp4
|
`-- CAD and SolidWorks geometry assets
```

The repository also contains the original SolidWorks assembly, component files, and STEP geometry exports.

## Running the Updated Python Solver

### Requirements

- Python 3
- OpenCV
- NumPy

Pillow is also required for the separate maze generator.

### Installation

From the repository root:

```bash
python3 -m pip install opencv-python numpy pillow
```

### Execute the Solver

```bash
python3 maze_solver.py
```

The standalone demonstration uses the bundled maze image and opens an OpenCV visualization window.

A graphical desktop environment is required for the OpenCV display.

### Generate a Maze

```bash
python3 maze_generator_9.py
```

The generator and solver are separate scripts.

## Engineering Limitations

- Offline verification of the updated solver covers one bundled maze image.
- The updated Dijkstra implementation has not been physically retested.
- The current MATLAB-to-Python integration has not been revalidated after the solver update.
- The original report describes the A*-based system, not the subsequent Dijkstra implementation.
- The project is a research and educational demonstration, not a production-certified robotic navigation system.

## Engineering Significance

This project demonstrates the integration of several robotics engineering disciplines:

- Computer vision and perception.
- Graph-based path planning.
- Coordinate transformations.
- Robotic manipulation.
- Inverse kinematics.
- Digital-twin simulation.
- Hardware-software integration.
- Experimental validation.

The project also illustrates the distinction between validating a robotics algorithm offline and demonstrating the complete perception-to-action pipeline on physical hardware.

---

**Arizona State University — RAS 545 Robotics I**

**Project Contributor:** Phillip Hogan

**GitHub:** TheTechGuy-PH4
