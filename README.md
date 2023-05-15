# Inertia Wheel Pendulum

This repository contains a complete project for an inertia wheel pendulum, including a 3D mechanical design, embedded C code for STM32 microcontroller, visualization in LabVIEW, and LQR (Linear-Quadratic Regulator) parameter generation using MATLAB.

## Table of Contents
- [Description](#description)
- [Mechanical Design (SolidWorks)](#mechanical-design)
- [LQR Parameter Generation](#lqr-parameter-generation)
- [Code (STM Cube IDE)](#embedded-c-code)
- [LabVIEW Visualization](#labview-visualization)
- [Installation](#installation)
- [Usage](#usage)

## Description
The inertia wheel pendulum is a physical system used in control systems education and research. It consists of a pendulum attached to a rotating wheel, allowing for interesting dynamic behavior and control challenges. This repository provides a comprehensive project that covers the mechanical design, embedded software, visualization, and control parameter generation.

## Mechanical Design
The `mechanical_design` directory contains all the necessary files for the 3D mechanical design of the inertia wheel pendulum. It includes CAD files, assembly instructions, and any additional documentation related to the physical construction of the system on SolidWorks.

## LQR Parameter Generation
The `lqr_parameter_generation` directory contains MATLAB scripts and functions for generating the LQR control parameters. LQR is a popular control technique used to stabilize and control linear systems. The provided MATLAB code helps calculate the optimal control gains for the inertia wheel pendulum.

## Embedded C Code
The `embedded_c_code` directory contains the source code written in C language for programming the STM32 microcontroller that controls the inertia wheel pendulum. The code implements various control algorithms and interfaces with the sensors and actuators of the system.

## LabVIEW Visualization
The `labview_visualization` directory contains a LabVIEW project that provides a graphical visualization of the inertia wheel pendulum. It allows users to monitor the real-time behavior of the system and analyze its dynamics. The LabVIEW code interfaces with the embedded system through a communication protocol.

## Installation
To use this project, follow these steps:
1. Clone the repository: `git clone https://github.com/your-username/inertia-wheel-pendulum.git`
2. Install any necessary software dependencies for the mechanical design, embedded C code, LabVIEW visualization, and MATLAB scripts.
3. Refer to the individual directories for specific installation instructions and requirements.

## Usage
Once the installation is complete, you can utilize the project in the following ways:
- Print and assemble the mechanical design using the provided documentation and proper encoder and actuator
- Generate LQR control parameters using MATLAB scripts and tune the control algorithm for the pendulum.
- Flash the embedded C project onto the STM32 microcontroller to control the inertia wheel pendulum. (change according to hardware used)
- Run the LabVIEW visualization project to observe and analyze the system's behavior.
