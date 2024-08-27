# Differential Wheeled Robot Navigation System

## Overview

This repository contains the code for a navigation system designed for a differential wheeled robot. The system enables the robot to follow a predefined path composed of multiple waypoints, with functionality for both linear and rotational movements. The navigation system uses state machines and PID controllers to manage and execute navigation tasks.

This project is the result of 10 months of continuous design, improvement, optimization, and real-world testing. The culmination of this effort is a navigation algorithm and system capable of moving the robot from any point A to point B with less than 1 millimeter of error on both the X and Y axes.

## Directory Structure

The repository is organized into directories containing the source and header files of the navigation algorithm, as well as an STM32 example that demonstrates the full implementation:

- `src/`: Contains the source files for the navigation algorithms.
  - `NavStateMachines.c`: Contains the implementation of state machines that control the robot's navigation, including path execution, waypoint navigation, and state transitions.
  - `Navigation.c`: Manages path calculation and trapezoidal profile generation.
  - `NavRoutineFunctions.c`: Interfaces with sensors to gather data for navigation tasks, including distance and angle measurements, and controls the actuators.
- `inc/`: Contains header files for the navigation algorithms and state machines.
- `STM32_example/`: Provides an example implementation for the STM32 platform, demonstrating how to integrate and use the navigation system with STM32 hardware.

## Key Components

### 1. Navigation State Machines

The core of the navigation system is based on state machines that manage different navigation tasks. The state machines are implemented as follows:

- **Path Execution State Machine (`Nav_vExecutePath`)**: Handles the overall execution of a path composed of multiple waypoints.
- **GoToXY State Machine (`Nav_vGoToXYStateMachine_BOB`)**: Manages movement to a specific coordinate, including rotation and linear motion.
- **Independent Trajectory State Machine (`Nav_vIndependantTrajectoryStateMachine`)**: Controls the execution of individual trajectories, including linear and angular movements.

### 2. Navigation Algorithms

The system uses the following algorithms and techniques to navigate:

- **PID Controllers**: Used for controlling distance and angle errors. The controllers adjust motor commands to achieve desired velocities and angles.
- **Trajectory Control**: Manages the robot's acceleration and trapezoidal velocity profiles to ensure smooth and accurate movements.

### 3. Sensor Integration

Sensors provide real-time feedback on the robot's position and orientation. Key sensor-related variables and functions include:

- **Distance Measurements**: Gathered from encoders and used to calculate the robot's position.
- **Angle Measurements**: Obtained from gyros or other sensors to manage the robot's orientation.

## Configuration

### External Variables

The system uses several external variables to store configuration and state information. These include:

- **Target Coordinates**: `Nav_XYTargets_ad` stores the target coordinates and angles.
- **Motor Commands**: `Nav_RightMotorCommand_i32` and `Nav_LeftMotorCommand_i32` control the robot's movement.
- **PID Parameters**: Includes `Nav_KpDistance_d`, `Nav_KiDistance_d`, `Nav_KdDistance_d` for distance control, and similar parameters for angle control.

### State Definitions

State definitions for the various state machines are provided in the header files and used throughout the implementation to manage different stages of navigation.

## Usage

1. **Initialize the System**: Set up the navigation system by configuring the state machines and sensors.
2. **Define Waypoints**: Use the `Nav_XYTargets_ad` array to specify the coordinates and angles for the robot to follow.
3. **Execute Path**: Call the `Nav_vExecutePath()` function to start navigating along the defined waypoints.
4. **Monitor and Adjust**: Use real-time data from sensors to adjust the navigation parameters and ensure accurate path execution.

## Notes

- Ensure that the sensors and encoders are properly calibrated before starting the navigation.
- Adjust PID parameters and trajectory settings based on the specific requirements of your robot and environment.

---

This README provides a high-level overview of the navigation system, including its history, configuration, and usage. Adjustments can be made based on the specific details of your implementation and any additional requirements you may have.