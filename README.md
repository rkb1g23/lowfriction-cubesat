# lowfriction-cubesat
Low-Friction Environment CubeSat Software

This repository contains the software for a Low-Friction Environment (LFE) CubeSat prototype, designed to integrate light tracking, mode switching, and attitude control using a combination of sensors, fans, and a reaction wheel.

Code Overview

The main Arduino sketch implements the following functionality:

Mode Management

Reads a PWM signal from the mode switch pin to determine the CubeSat’s operational mode:

MODE_LIGHT_TRACKING – orients the satellite toward a light source

MODE_STANDBY – idle state

MODE_YAW_CONTROL – manual yaw control via PWM input

Updates the current mode continuously in the loop() function.

Light Tracking

Uses two VEML7700 lux sensors and one TCS34725 RGB sensor to detect light direction and intensity.

Computes a PID control signal to drive the reaction wheel and adjust orientation.

Continuously compares current light readings to an ambient light baseline, only activating tracking if the light exceeds a threshold.

Reaction Wheel Control

Implements a motor driver interface with PWM and direction pins to control a single-axis reaction wheel.

PID controller adjusts wheel speed based on light-tracking error or yaw input, enabling precise orientation.

Includes safety limits and dead zones to prevent excessive or unstable rotation.

Propulsion Fans

The code supports four translation fans and one hover fan, allowing small translational movements and stabilization within the low-friction environment.

Fan outputs are coordinated with sensor readings and control logic to maintain stability and respond to light or yaw commands.

Initialization

Initializes IMU, lux sensors, and RGB sensor at startup.

Sets a baseline ambient light reading from the RGB sensor to calibrate light tracking.

Ensures all actuators are in a safe, stopped state at startup.

Summary

The code integrates sensing, mode management, and actuation to demonstrate autonomous light tracking and orientation control in a low-friction CubeSat testbed. It provides a framework for further experimentation with attitude control and low-drag propulsion systems.
This project highlights the challenges and solutions involved in small satellite control, stabilization, and light-based navigation, providing a reference for anyone developing CubeSats or similar low-friction robotic systems.
