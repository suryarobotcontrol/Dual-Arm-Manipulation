# Dual-Arm Robotic Manipulation with Improved Artificial Potential Field (iAPF)

This repository contains the implementation of a dual-arm robotic manipulation system using an improved Artificial Potential Field (iAPF) approach for collision avoidance and coordinated manipulation.

## Overview

The system provides a comprehensive framework for dual-arm asymmetric manipulation with inter-arm collision avoidance for handling industrial components. The approach uses computationally efficient collision detection by representing manipulator links as line segments in 3D space, enabling real-time distance monitoring through analytical solutions for parallel, intersecting, and skew configurations.

## Key Features

- **Efficient Collision Detection**: Line segment-based geometry for fast inter-arm collision detection
- **Three-way Force Equilibrium**: Novel potential field method combining exponential attractive, inverse-square repulsive, and exponential home-seeking forces
- **State-Based Task Allocation**: Priority-based state machine for deadlock-free simultaneous manipulation
- **Real-time Vision Integration**: Object detection and pose estimation for industrial components
- **Hybrid Linear-Angular Control**: Combined APF-derived linear velocity with PD-controlled angular velocity

## Repository Structure

```
.
├── apf.py                     # Artificial Potential Field implementation
├── arm.py                     # Robotic arm control classes and functions
├── fwd_kin_gen3lite.py        # Forward kinematics for Kinova Gen3 Lite robot
├── line_collision.py          # Collision detection algorithms
├── nutbolt.py                 # Object class definitions
├── sorting.py                 # Sorting algorithm implementation
├── utils.py                   # Utility functions and constants
├── main.py                    # Main application entry point
├── __init__.py                # Package initialization
└── README.md                  # This file
```

## Installation Requirements

```bash
# Clone the repository
git clone https://github.com/yourusername/Dual-Arm-Manipulation.git
cd dual-arm-manipulation

# Install dependencies
pip install numpy scipy matplotlib
```

Additional requirements:
- ROS Noetic
- Kinova Gen3 Lite robot arms with Kortex API
- Intel RealSense D415 camera
- Python 3.8+

## Usage

### Basic Execution

```bash
python main.py
```

This will initialize the dual-arm system and start the control loop. The system will:
1. Connect to the robot arms
2. Initialize the vision system
3. Detect industrial components (nuts and bolts)
4. Plan and execute collision-free manipulation

### Core Components

#### APF (Artificial Potential Field)

The `apf.py` module implements the improved Artificial Potential Field method with three force components:

```python
# Example usage of APF functions
from apf import force_att, force_rep, force_home, apf

# Calculate attractive force
f_goal, dist_goal = force_att(agent_p, goal_p, charge_goal)

# Calculate repulsive force
f_ob, dist_obs = force_rep(agent_p, obs_p, dist, -charge_ob)

# Calculate home-seeking force
f_home, dist_home = force_home(agent_p, home_p, dist_h, charge_ob)

# Calculate final force with all components
f_final, dist_goal, forces = apf(agent_p, goal_p, obs_p, home_p, dist, velocity)
```

#### Robotic Arm Control

The `arm.py` module provides classes for controlling both arms:

```python
# Initialize the dual-arm system
from arm import DualArms

arms = DualArms()

# Set targets for each arm
arms.nut_target([x, y, z, roll, pitch, yaw])  # For left arm
arms.bolt_target([x, y, z, roll, pitch, yaw])  # For right arm

# Update targets and calculate forces
arms.update_targets()
arms.check_collisions()
arms.calculate_apf_forces()
```

#### Collision Detection

The `line_collision.py` module provides analytical methods for detecting and measuring distance between arm links:

```python
from line_collision import detect_coll

# Calculate minimum distance between two line segments
dist = detect_coll(p1, p2, p3, p4)
```

## System Architecture

The system follows a modular architecture with the following components:

1. **Perception**: Vision-based detection and pose estimation
2. **Planning**: Improved APF-based motion planning
3. **Control**: Hybrid velocity control (linear and angular)
4. **State Management**: Priority-based state machine
5. **Forward Kinematics**: Real-time position and orientation calculation

## State Machine

The system uses a state machine to manage the manipulation process:

- `IDLE`: Initial state, waiting for targets
- `PICK`: Moving towards target object
- `PICK_LOCK`: Exclusive access to approach target
- `GRIP`: Gripping target object
- `PLACE`: Moving towards placement location
- `PLACE_LOCK`: Exclusive access to approach placement
- `UNGRIP`: Releasing object
- `DONE`: Task completed

## Examples

### Setting Up Dual-Arm Manipulation

```python
from apf_sorting import DualArms, Algorithm

# Initialize dual-arm system
arms = DualArms()

# Create algorithm instance
algorithm = Algorithm(arms)

# Set targets
pose_pick = [0.218, -0.15, 0.2, 0, -180, 90]
pose_drop = [0.218, 0.15, 0.2, 0, -180, 270]

arms.nut_target(pose_pick, pose_drop)

# Main control loop
while True:
    algorithm.set_targets()
    arms.update_targets()
    arms.check_collisions()
    arms.calculate_apf_forces()
```



## License

[MIT License](LICENSE)

## Contributors

- Surya Prakash S.K
- Darshan Kumar Prajapati
- Bhuvan Narula
- Amit Shukla

## Acknowledgments

This work was supported by the Centre for Artificial Intelligence and Robotics, IIT Mandi, and the School of Mechanical and Materials Engineering, IIT Mandi.
