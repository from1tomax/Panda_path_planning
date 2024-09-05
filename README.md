# Team_H

1. install educational version of CoppeliaSim_Edu_V4_1_0_Ubuntu20_04
2. install pyrep from https://github.com/stepjam/PyRep and follow installation quidelines

# Presentation

https://docs.google.com/presentation/d/1AJWDzzrQ4JDr2ALND8X1Dvz7XjJeEq1TzlvenHl9Lks/edit?usp=sharing

# Task Distribution

- Theresa Gräbner: Task 1 + Task 3 → Branch Theresa
- Peishi Liu: Task 2

# References

- ARCL_23_24: team b
- https://github.com/stepjam/PyRep
- https://moveit.picknik.ai/humble/doc/tutorials/pick_and_place_with_moveit_task_constructor/pick_and_place_with_moveit_task_constructor.html


# Tasks

- each Task has its own folder
- in /src you can find the executables
	- execute with "python Task[number].py"
- in files there are the results documented

# Task 1

Uses CoppeliaSim_Edu_V4_1_0_Ubuntu20_04 and PyRep https://github.com/stepjam/PyRep

## Task 1.1

- Goal: Test joint configurations for their validity
- Model: Given Franka Panda robot by chair

### Implementation:

- Self-Collision check: check each link against all others except neighbouring links
- Write result in json format in “task1_1_self_collision.data”

## Task 1.2

- Goal: test joint configurations and cartesian pose for validity + plan normalized/linear path if possible, otherwise plan an alternative path
- Model: Pyrep’s Panda Robot

### Implementation:
- Scene without ﬂoor → not interested in collision with ﬂoor
- For every cartesian pose: Plan cartesian trajectory
- For every joint configuration: Plan joint trajectory

#### Cartesian Trajectroy Implementation:

- Load Cartesian Pose
- Extract quaternions and position coordinates via Pinocchio
- Test Cartesian Pose
	- Reachability: Sample valid joint configuration
	- Linear Trajectory: Pyrep’s get_linear_path
	- Nonlinear Trajectory: Pyrep’s get_nonlinear_path 
- Write result in json format in “task1_2_cartesian_trajectory.data”


#### Joint Trajectory Implementation:

- Load joint configuration
- Reachability: Test if joint configurations are within the limits defined by Franka’s official website
- Validity: Configuration does not cause self-collision
- Normalized Trajectory: 
	- Calculate trajectory
	- Validate each path point
- Alternative Path: Modified PyRep’s get_path 
- Write result in json format in “task1_2_joint_trajectory.data”


# Task 2: System Requirements and Setup Instructions

## System Requirements

To complete this task, ensure your system meets the following requirements:

- **Operating System**: Ubuntu 22.04
- **ROS 2 Distribution**: Humble
- **MoveIt 2 Version**: Humble

## Installation Instructions

Given that the required packages are quite numerous, it is recommended to follow the official MoveIt 2 installation guide for ROS 2 Humble. This guide will help you set up the necessary environment on your system.

### Step-by-Step Guide

1. **Follow the MoveIt 2 Humble Startup Instructions**:
   - Visit the official MoveIt 2 Humble getting started guide for detailed instructions on installing and configuring MoveIt 2 on your system.

   - You can find the guide [here](https://moveit.picknik.ai/humble/doc/tutorials/getting_started/getting_started.html).

   This guide covers everything from installing ROS 2 Humble to setting up MoveIt 2 and running the first demo.
2. **build your workspace**
	- `cd ~/ws_moveit2/src` and clone the Task2 pkg under branch Peishi
	- use this command to install all the dependencies`rosdep install --from-paths src --ignore-src -r -y`
	- `cd ..` and build all the pkgs with `colcon build`
	-  then source your workspace with `source ~/ws_moveit2/install/setup.bash`
	-  start the simulation and visualize it with rviz, `ros2 launch task2 Task2.launch.py`

## Task 2.1
-  `ros2 launch reach_and_grasp.launch.py`

## Task 2.2
-  `ros2 launch reach_and_grasp_cluttered.launch.py`
- 	add the **motion planning** plugin into rviz and drag the button in the left bottom to see the Nullspace 

## Task 2.3
- not done

# Task 3

Uses CoppeliaSim_Edu_V4_1_0_Ubuntu20_04 and PyRep https://github.com/stepjam/PyRep

## Task 3.1

- Goal: Pick up cuboid and hold it with both robots
- Model: Pyrep’s Panda Robot
- Libraries: PyRep

### Implementation:

- Move robots into starting position
- Right Robot → 0.55 |   0  | 0.01
- Left Robot  → 0.7  | -0.1 | 0.3
- Right Robot grasps cuboid + moves to [0.7, -0.01, 0.3]
- Left Robot moves to [0.7, 0.01, 0.3] + grasps cuboid
- Save times and paths

## Task 3.2

- Goal: Pick up Cuboid and move it in dual-mode along a circular path
- Model: Pyrep’s Panda Robot
- Libraries: PyRep

### Execution:
- execute with "python Task3_2.py"
- programm asks if it should use old solution
	- write "y" for recreating the last solution
	- write "n" for recalculating solution
	
### Why change configuration at every waypoint?
- corrects pose of robot
- adds stability
- moves robot further away from singular configurations, which makes it easier to calculate a functioning, stable subtrajectory
- if robot comes to configurations which are close to its limits, the cartesian trajectory can most likely not be continued that way but the robot needs to change its configuration in place to continue
- used ARCL_23_24: team b as inspirement

### Initial Configuration
- saved in Initialisation_Conf.data

### Implementation: circular path in dual mode
- Move robots to starting position
- Calculate waypoints every 30 degree
- For every waypoint:
	- Move robots to initialization configuration of waypoint
	- Calculated 4 intermediate waypoints	
	- Pyrep’s get_path_from_cartesian_path

### Future Improvements:
- implement special control algorithm, such that robots at 180 degrees don't have to interrupt dual mode. 
- Robots should be able to move to new configuration without moving the end effector

