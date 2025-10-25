# Quadrotor with Gimbal Tracking a Fixed-Wing Aircraft Simulation

This repository contains a set of MATLAB scripts that simulate a quadrotor with a gimbal tracking a fixed-wing aircraft. The simulation uses a Dynamic Feedback Linearization for the quadrotor and a 6-degrees-of-freedom (6DOF) model for the fixed-wing aircraft.

## Project Structure

The project is organized into the following directories:

-   **`DFL_controller/`**: Contains the scripts related to the Dynamic Feedback Linearization Controller.
-   **`models/`**: Contains the dynamics models for the quadrotor and the fixed-wing aircraft.
-   **`utilities/`**: Contains helper functions used by the other scripts.

## Files and Descriptions

### Configuration

-   **`config.m`**: This file contains all the parameters for the simulation, including the quadrotor and fixed-wing aircraft parameters, as well as the simulation time and time step.

### Main Simulation Script

-   **`test_fw_realtime_tracking.m`**: This is the main script to run the simulation. It first runs `config.m` to load all the necessary parameters, then initializes the initial conditions for both the quadrotor and the fixed-wing aircraft, and then uses the `ode45` solver to simulate their flight over time. After the simulation, it generates several plots to visualize the results, including the 3D trajectories, position tracking error, and gimbal angle performance.

### DFL Controller

-   **`DFL_controller/controller_generation.m`**: This is a symbolic math script that uses MATLAB's Symbolic Math Toolbox to derive the feedback linearization controller for the quadrotor-gimbal system. It defines the system's dynamics symbolically, specifies the desired outputs (position, yaw, and gimbal angles), and then automatically computes the relative degree of each output. The script then calculates the decoupling matrix and drift terms, which are used to derive the control law. Finally, it generates two MATLAB functions, `alpha_gimbal_func.m` and `beta_gimbal_func.m`, which implement the derived control law.

-   **`DFL_controller/alpha_gimbal_func.m`**: This is an auto-generated function that calculates the `alpha` component of the feedback linearization control law. This component is a function of the system's state and parameters, and it represents the part of the control input that cancels out the nonlinearities in the system's dynamics.

-   **`DFL_controller/beta_gimbal_func.m`**: This is an auto-generated function that calculates the `beta` component of the feedback linearization control law, which is the decoupling matrix. This matrix multiplies the virtual control input `v` to produce the actual control input `u`.

### Dynamics Models

-   **`models/unified_dynamics.m`**: This function acts as the bridge between the fixed-wing aircraft and the quadrotor. At each time step, it first calls `fw_6dof_quat.m` to calculate the state of the fixed-wing based on predefined control inputs (thrust and elevator angle). The resulting fixed-wing state (position, velocity, acceleration, etc.) is then used as the reference trajectory for the quadrotor. Finally, it calls `quadrotor_dynamics_realtime.m` to compute the quadrotor's state, which attempts to follow this reference trajectory.

-   **`models/fw_6dof_quat.m`**: This function implements a standard 6-degrees-of-freedom (6DOF) model for the fixed-wing aircraft using quaternions to represent its attitude. It calculates the aerodynamic forces and moments based on the aircraft's state and control inputs (thrust, elevator, aileron, and rudder). These forces and moments are then used to determine the aircraft's translational and rotational acceleration, which are integrated over time to simulate its motion. The function also outputs the acceleration, jerk, and snap of the fixed-wing in the NED frame, which are used as reference inputs for the quadrotor's controller.

-   **`models/quadrotor_dynamics_realtime.m`**: This function implements the quadrotor's dynamics and a Differential Flatness Controller (DFL). This controller calculates the necessary motor thrusts and moments to make the quadrotor follow the reference trajectory provided by the fixed-wing aircraft. The script also includes a first-order model for the gimbal, which is controlled to point in the direction of the fixed-wing's velocity vector. The controller uses feedback linearization to simplify the control design, and the `alpha_gimbal_func.m` and `beta_gimbal_func.m` functions are the result of a symbolic derivation of the feedback linearization law.

### Utilities

-   **`utilities/Lie_derivative.m`**: This is a helper function that contains a recursive function to compute the Lie derivative of a scalar function `h` with respect to a vector field `f`. This is a fundamental operation in nonlinear control theory and is used extensively in the `controller_generation.m` script to derive the feedback linearization control law.

-   **`utilities/stlread.m`**: A function to read STL files.

## How to Run the Simulation

1.  Open MATLAB.
2.  Navigate to the directory containing these files.
3.  Run the `test_fw_realtime_tracking.m` script.

The script will run the simulation and generate several plots showing the results.

## Simulation Workflow

1.  **`test_fw_realtime_tracking.m`** runs **`config.m`** to load all parameters.
2.  **`test_fw_realtime_tracking.m`** sets up the initial conditions.
3.  It calls `ode45` with the **`models/unified_dynamics.m`** function.
4.  **`models/unified_dynamics.m`** calls **`models/fw_6dof_quat.m`** to get the fixed-wing's state.
5.  The fixed-wing's state is used as a reference for the quadrotor.
6.  **`models/unified_dynamics.m`** then calls **`models/quadrotor_dynamics_realtime.m`** to calculate the quadrotor's state.
7.  **`models/quadrotor_dynamics_realtime.m`** uses the DFL controller, which in turn calls **`DFL_controller/alpha_gimbal_func.m`** and **`DFL_controller/beta_gimbal_func.m`** to compute the control inputs.
8.  The state derivatives are returned to `ode45`, which integrates them over time.
9.  After the simulation is complete, **`test_fw_realtime_tracking.m`** plots the results.
