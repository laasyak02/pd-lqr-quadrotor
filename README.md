# PD & LQR Control for Quadrotor Trajectory Tracking

A MATLAB simulation that designs, implements and compares two controllers for autonomous quadrotor flight: a **Proportional-Derivative (PD)** controller and a **Linear Quadratic Regulator (LQR)**. The controllers are used to fly a **Crazyflie** quadrotor to track two 3D trajectories - a **circle** and a **diamond** trajectory and compare them on tracking accuracy and computational cost.

This project was done as part of the final project for the RBE 502: Robot Control course at WPI. Built on the course's Crazyflie quadrotor simulator.

<table>
  <tr>
    <th></th>
    <th>Circle (top view)</th>
    <th>Diamond (side view)</th>
  </tr>
  <tr>
    <td><b>PD</b></td>
    <td><img width="400" alt="PD controller on the circle trajectory" src="media/pd_circle_top.png" /></td>
    <td><img width="400" alt="PD controller on the diamond trajectory" src="media/pd_diamond_side.png" /></td>
  </tr>
  <tr>
    <td><b>LQR</b></td>
    <td><img width="400" alt="LQR controller on the circle trajectory" src="media/lqr_circle_top.png" /></td>
    <td><img width="400" alt="LQR controller on the diamond trajectory" src="media/lqr_diamond_side.png" /></td>
  </tr>
</table>
<p align="center"><em>Blue: desired path. Red: path actually flown.</em></p>

---

## Overview

A quadrotor is controlled through four inputs: the collective thrust `u1` and the three body moments `u2 = [Mx, My, Mz]`. Both controllers compute these from the current state and the desired trajectory.

Both are designed under a **near-hover assumption**: the roll and pitch angles stay small, and the thrust stays close to the value needed to hover. Under this assumption, the trigonometric terms in the dynamics can be approximated (for example, `sin φ ≈ φ` and `cos φ ≈ 1`), which makes the model much simpler to design around. The cost is that the quadrotor is limited to smooth, non-aggressive maneuvers.

The desired position, velocity, acceleration and yaw at each moment come from smooth polynomial trajectories in `traj/`:

- **Circle:** a 1 m radius loop in the horizontal plane over 8 s, climbing to 0.5 m.
- **Diamond:** a diamond in the vertical (y–z) plane, flown as four 2 s segments with sharp corners.

---

## PD controller

`control/pid_controller.m` is a **cascaded** controller. A quadrotor can only accelerate sideways by tilting, so an outer position loop decides the acceleration needed and an inner attitude loop tilts the quadrotor to produce it.

- **Position loop:** computes the desired acceleration from the position and velocity errors, `r̈_des = r̈_T + Kd·(ṙ_T − ṙ) + Kp·(r_T − r)`. Its vertical part sets the thrust `u1`, and its horizontal part gives the desired roll and pitch.
- **Attitude loop:** computes the moments `u2` that drive the roll, pitch and yaw to their targets, using PD feedback on the angle and angular-rate errors.
- **Gains (hand-tuned):** position `Kp = [11, 11, 12]`, `Kd = [10.4, 10.4, 10]`; attitude `Kp = [74, 74, 20]`, `Kd = [58, 58, 15]`.

---

## LQR controller

`control/lqr_controller.m` controls the quadrotor as a single **12-state system** (position, Euler angles, linear velocity and body rates) and computes all four control inputs together.

- **Linearization:** LQR needs a linear model, so at every time step the near-hover dynamics are linearized around the desired state from the trajectory and the hover input, giving matrices `A` and `B`.
- **Optimal gain:** MATLAB's `lqr(A, B, Q, R)` finds the gain `K` that best balances tracking error against control effort. This is then used to calculate the control as `u = K·(x_des − x) + u_des`.
- **Weights:** `Q = diag([10,10,10, 1,1,10, 10,10,10, 0.1,0.1,0.1])` weights position, velocity and yaw most for tight tracking. `R = diag([0.1, 1, 1, 1])` penalizes thrust less than the moments.

---

## Results

Both controllers complete both trajectories, but LQR tracks them more closely. In the flight paths at the top, LQR stays close to the diamond's edges, while the PD path overshoots the corners and drifts off some of the edges.

The difference is most clearly seen in **velocity tracking on the diamond trajectory**, where the sharp corners require rapid changes of direction:

<table>
  <tr>
    <th>PD</th>
    <th>LQR</th>
  </tr>
  <tr>
    <td><img width="400" alt="PD velocity tracking on the diamond" src="media/pd_diamond_vel.png" /></td>
    <td><img width="400" alt="LQR velocity tracking on the diamond" src="media/lqr_diamond_vel.png" /></td>
  </tr>
</table>
<p align="center"><em>Velocity on the diamond trajectory. Blue: desired. Red: actual.</em></p>

- **Tracking accuracy:** PD overshoots at each corner resulting in oscillation. LQR tracks the desired velocity closely, with lower position and velocity errors overall.
- **Computational cost:** LQR re-linearizes the model and solves the Riccati equation at every step, which makes it significantly slower to run than PD.
- **Trade-off:** PD is simpler to implement and tune, while LQR gives noticeably better tracking on dynamic trajectories.

---

## Repository structure

| Path | Description |
|------|-------------|
| `control/pid_controller.m` | PD position and attitude controller. |
| `control/lqr_controller.m` | LQR controller: per-step linearization and `lqr()`. |
| `traj/circle.m`, `traj/diamond.m` | Circle and diamond trajectory generators. |
| `traj/jump.m` | Step trajectory used for the hover test. |
| `test_trajectory.m` | Simulation loop: selects the controller and trajectory, runs the simulation and plots the results. |
| `runsim.m` | Entry point: sets the start and goal and calls `test_trajectory`. |
| `dynamics/` | Quadrotor equations of motion and Crazyflie parameters. |
| `utils/` | Rotation and quaternion helpers, plotting, map loading and collision checking (simulator framework). |
| `maps/` | Example environment maps. |

---

## Stack

`MATLAB` · `PD control` · `LQR` · `Linearization` · `Trajectory generation` · `Quadrotor dynamics`

---

## Running it

Requires MATLAB with the **Control System Toolbox** (for `lqr`).

1. Choose the trajectory and controller near the top of `test_trajectory.m`:
   ```matlab
   trajhandle    = @circle;        % or @diamond
   controlhandle = @pid_controller; % PD
   % controlhandle = @(qd, t, qn, params)lqr_controller(qd, t, qn, params, trajhandle);  % LQR
   ```
2. From the repo root, run:
   ```matlab
   runsim
   ```

The simulator animates the flight in 3D, then plots the desired (blue) and actual (red) position and velocity.

---

## Acknowledgements

Built on the RBE 502 (WPI) Crazyflie quadrotor simulator framework. The design and tuning of the PD and LQR controllers, and the trajectory-tracking analysis, are my own work.
