# lowfriction-cubesat
Software for a Low-Friction Environment CubeSat prototype, including code for light tracking, mode switching, and reaction wheel attitude control, using 2 lux sensors, 1 RGB sensor, a reaction wheel, flight computer, 4 translation fans, and 1 hover fan.

Low-Friction Environment CubeSat

This repository contains the software developed for a Low-Friction Environment (LFE) CubeSat prototype, designed to validate core subsystems and demonstrate functionality as outlined in the project design brief. The code integrates sensing, control, and actuation to achieve light tracking, attitude control, and precise motion within a low-friction test environment.

Features

Light Tracking: Uses 2 lux sensors and 1 RGB sensor to detect and orient the CubeSat toward a light source. The system continuously adjusts orientation for optimal light alignment.

Mode Switching: Manages transitions between operational states, including idle, tracking, and attitude control, ensuring the CubeSat responds appropriately to mission conditions.

Attitude Control: Implements closed-loop control using a reaction wheel to stabilize and adjust satellite orientation.

Propulsion Control: Coordinates 4 translation fans and 1 hover fan for motion and stabilization within the low-friction environment.

Hardware Overview

2 × Lux sensors

1 × RGB sensor

1 × Reaction wheel

1 × Flight computer

4 × Translation fans

1 × Hover fan

Purpose

The repository demonstrates practical integration of sensing, control, and actuation for a CubeSat in a simulated low-friction environment. It serves as both a proof of concept and a foundation for future CubeSat research and educational projects. The code is organized to allow easy access to individual subsystems, enabling testing, adaptation, or expansion for related applications.

This project highlights the challenges and solutions involved in small satellite control, stabilization, and light-based navigation, providing a reference for anyone developing CubeSats or similar low-friction robotic systems.
