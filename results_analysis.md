# Analysis of Simulation Results

This document presents an analysis of the simulation results for the quadrotor tracking a fixed-wing aircraft. The analysis is divided into two main scenarios, based on the trajectory of the fixed-wing aircraft: a circular loop and a straight level flight.

## Loop Trajectory

The loop trajectory is a challenging maneuver that tests the agility and tracking performance of the controller in a dynamic, changing environment.

### 3D Trajectory Tracking

The 3D trajectory plot (`3d_trajectory.pdf`) visualizes the path of the quadrotor and the fixed-wing aircraft in the NED (North-East-Down) frame. In a successful simulation, the quadrotor's path should closely follow the circular path of the fixed-wing aircraft, demonstrating the controller's ability to maintain a small tracking error even during sustained turns. The plot also shows the orientation of both vehicles at different points in time, providing a qualitative assessment of the tracking performance.

### Position and Orientation Tracking

The time-series plots for position and orientation (`position_and_orientation.pdf`) provide a more detailed, quantitative view of the tracking performance.

-   **Position Tracking**: The plots for X, Y, and Z positions show the quadrotor's coordinates over time compared to the fixed-wing's coordinates. For the loop trajectory, the X and Y positions will be sinusoidal, while the Z position (altitude) should remain relatively constant. The controller's effectiveness is demonstrated by the small and bounded error between the quadrotor and the fixed-wing's position plots.
-   **Orientation Tracking**: The global roll, pitch, and yaw plots show how the gimbal's orientation tracks the fixed-wing's orientation. During the loop, the roll angle of the fixed-wing will be significant and relatively constant, while the yaw angle will change continuously. The gimbal's orientation should closely match the fixed-wing's orientation, indicating successful pointing and tracking.

### Control Inputs

The control input plots (`control_inputs.pdf`) show the commands sent to the quadrotor's actuators. For the loop trajectory, we expect to see continuous and smooth control inputs. The total thrust should remain relatively constant to maintain altitude, while the roll, pitch, and yaw moments will be actively changing to follow the circular path. The gimbal control inputs will also be active to keep the gimbal pointing at the fixed-wing. Smooth control inputs without saturation are indicative of a well-behaved and stable controller.

## Straight Level Flight Trajectory

The straight level flight trajectory is a less dynamic scenario, which is useful for evaluating the controller's steady-state performance and its ability to reject disturbances.

### 3D Trajectory Tracking

In the 3D trajectory plot for this scenario, both the fixed-wing and the quadrotor should follow a straight line. The key indicator of performance is the proximity of the two paths. A successful test will show the quadrotor's path quickly converging to and then maintaining the straight path of the fixed-wing aircraft.

### Position and Orientation Tracking

-   **Position Tracking**: The position plots will show a linear change in one or two of the position coordinates (e.g., X), while the other coordinates remain constant. The tracking error should converge to a small value, demonstrating good steady-state tracking performance.
-   **Orientation Tracking**: For a straight flight, the fixed-wing's orientation will be mostly constant. The gimbal's orientation should also remain constant and aligned with the fixed-wing's orientation. This demonstrates the controller's ability to maintain a stable lock on the target.

### Control Inputs

During the straight level flight, after an initial transient phase, the control inputs are expected to settle to relatively constant values. The thrust will balance the weight of the quadrotor, and the moments will be small, primarily acting to counteract any disturbances. The gimbal control inputs should also be minimal, as the target is not maneuvering. This indicates that the controller can efficiently maintain the desired state without excessive control effort.

## Debugging and Verification Plots

The `debug_roll_comparison.pdf` and `debug_pitch_comparison.pdf` plots are used for verification and debugging. They compare the orientation angles calculated through different methods (post-processing, simulation, drone body, and fixed-wing). In a successful simulation, these different calculations should be consistent, which validates the accuracy of the simulation and the post-processing steps. Any significant discrepancies in these plots would point to potential issues in the simulation or the mathematical models used.
