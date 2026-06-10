# Directional Guidance System (DGS)

## Overview

Directional Guidance System (DGS) is a simple navigation and guidance simulation developed in C++. The system calculates the direction of a target relative to a drone, determines the heading error, and generates movement commands that guide the drone toward its destination.

This project demonstrates fundamental guidance and control concepts commonly used in autonomous aerial vehicles, robotics, and navigation systems.

---

## Features

- Target azimuth calculation
- Heading error computation
- Shortest-angle correction logic
- Adaptive turning commands
- Distance-based speed control
- Modular and extensible architecture
- Mission command generation

---

## Project Structure

```text
.
├── main.cpp
├── Guidance.h
├── Guidance.cpp
├── Position.h
├── MissionCommand.h
└── README.md
```

### File Descriptions

| File | Description |
|--------|------------|
| `main.cpp` | Runs the guidance simulation |
| `Guidance.cpp` | Contains the core navigation and guidance algorithms |
| `Guidance.h` | Function declarations |
| `Position.h` | Position data structure using North-East coordinates |
| `MissionCommand.h` | Mission command output structure |

---

## How It Works

The guidance system follows these steps:

1. Calculates the azimuth angle from the drone to the target.
2. Computes the heading error between the current yaw and target direction.
3. Finds the shortest angular difference.
4. Generates a turning command based on the heading error.
5. Calculates the distance to the target.
6. Adjusts forward speed according to distance and alignment.
7. Produces a mission command containing:
   - Yaw correction command
   - Forward speed command

---

## Example Scenario

Drone Position:

```cpp
{0.0, 0.0}
```

Target Position:

```cpp
{40.0, 20.0}
```

Current Heading:

```cpp
90.0
```

Example Output:

```text
Turn Command (deg): -20
Forward Speed (m/s): 3
```

---

## Applications

This project can serve as a foundation for:

- UAV navigation systems
- Autonomous drone guidance
- Robotics path-following systems
- Flight control software experiments
- Navigation and control simulations

---

## Future Improvements

- PID-based heading controller
- Waypoint navigation
- Multiple target support
- GPS coordinate integration
- 3D navigation
- Wind and disturbance simulation
- Real-time visualization
- ROS2 integration

---

## Technologies Used

- C++
- Object-Oriented Programming (OOP)
- Navigation Algorithms
- Mathematical Guidance Logic

---

## Author

**Meryem Yılmaz**

Software Engineering Student

Interested in Autonomous Systems, UAV Technologies, Computer Vision, and Defense Industry Software Development.**
