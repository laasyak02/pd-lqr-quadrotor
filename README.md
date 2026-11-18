# PD & LQR Control for Quadrotor Trajectory Tracking

A MATLAB simulation that compares two ways of making a **Crazyflie quadrotor** follow a 3D trajectory: a classic cascaded **PD controller** and an **LQR controller** that re-linearizes the dynamics at every time step. Both controllers fly the same **circle** and **diamond** trajectories, and are compared on tracking accuracy and computational cost.

Final project for **RBE 502: Robot Control, WPI**. Built on the course's Crazyflie quadrotor simulator.

<table>
  <tr>
    <th></th>
    <th>Circle (top view)</th>
    <th>Diamond (side view)</th>
  </tr>
  <tr>
    <td><b>PD</b></td>
    <td><img width="300" alt="PD controller on the circle trajectory" src="media/pd_circle_top.png" /></td>
    <td><img width="300" alt="PD controller on the diamond trajectory" src="media/pd_diamond_side.png" /></td>
  </tr>
  <tr>
    <td><b>LQR</b></td>
    <td><img width="300" alt="LQR controller on the circle trajectory" src="media/lqr_circle_top.png" /></td>
    <td><img width="300" alt="LQR controller on the diamond trajectory" src="media/lqr_diamond_side.png" /></td>
  </tr>
</table>
<p align="center"><em>Blue: desired path. Red: path actually flown.</em></p>

---

## Overview

Each controller computes the four control inputs, collective thrust `u1` and body moments `u2 = [Mx, My, Mz]`, from the current state and the desired trajectory. Both are designed under a **near-hover (small-angle) assumption**, which keeps the dynamics tractable and limits the quadrotor to non-aggressive maneuvers.

The desired position, velocity, acceleration and yaw come from smooth polynomial trajectories in `traj/`:

- **Circle:** a 1 m radius loop in the horizontal plane over 8 s, climbing to 0.5 m (7th-order polynomials).
- **Diamond:** a diamond in the y–z plane, flown as four 2 s segments with sharp corners (quintic polynomials).

---

## PD controller

`control/pid_controller.m` is a **cascaded** controller, since a quadrotor produces lateral motion only by tilting its thrust vector.

- **Position loop:** computes a commanded acceleration from the feedforward trajectory acceleration plus PD feedback on the position and velocity errors, `r̈_des = r̈_T + Kd·(ṙ_T − ṙ) + Kp·(r_T − r)`. The z-component sets the thrust `u1 = m·g + m·r̈_des,z`. The x and y components are inverted through the near-hover dynamics to give the desired roll and pitch.
- **Attitude loop:** tracks those angles and the desired yaw with PD feedback on the attitude and body-rate errors, `u2 = I·(Kp,att·e_angle + Kd,att·e_rate)`.
- **Gains (hand-tuned):** position `Kp = [11, 11, 12]`, `Kd = [10.4, 10.4, 10]`; attitude `Kp = [74, 74, 20]`, `Kd = [58, 58, 15]` (roll, pitch, yaw).

---

## LQR controller

`control/lqr_controller.m` controls the full **12-state model** (position, Euler angles, linear velocity, body rates) with all four inputs together, rather than as separate loops.

- **Linearization along the trajectory:** at every time step, the near-hover dynamics `ẋ = f(x, u)` are linearized around the desired state and the hover input `u_des = [m·g; 0; 0; 0]`. The Jacobians `A = ∂f/∂x` (12×12) and `B = ∂f/∂u` (12×4) were derived analytically, so the model stays accurate as the reference moves.
- **Optimal gain:** `lqr(A, B, Q, R)` solves the Riccati equation for the gain `K` that minimizes a quadratic cost on tracking error and control effort. The control law is `u = K·(x_des − x) + u_des`.
- **Weights:** `Q = diag([10,10,10, 1,1,10, 10,10,10, 0.1,0.1,0.1])` prioritizes position, velocity and yaw for tight tracking and less drift while turning. `R = diag([0.1, 1, 1, 1])` penalizes thrust less than the moments.

---

## Results

Both controllers complete both trajectories, but LQR tracks them more closely. In the flight paths at the top, LQR stays close to the diamond's edges, while the PD path overshoots the corners and drifts off some of the edges.

The difference is clearest in **velocity tracking on the diamond**, where the sharp corners require rapid changes of direction:

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

- **Tracking accuracy:** PD overshoots at each corner and keeps oscillating after the trajectory ends. LQR tracks the desired velocity closely, with lower position and velocity errors overall.
- **Computational cost:** LQR re-linearizes the model and solves the Riccati equation at every step, which makes it significantly slower to run than PD.
- **Trade-off:** PD is simpler to implement and tune, while LQR gives noticeably better tracking on dynamic trajectories.

---

## Repository structure

| Path | Description |
|------|-------------|
| `control/pid_controller.m` | PD position and attitude controller. |
| `control/lqr_controller.m` | LQR controller: per-step linearization and `lqr()`. |
| `traj/circle.m`, `traj/diamond.m` | Circle and diamond trajectory generators. |
| `traj/jump.m` | Step trajectory used for the hover test in the first phase. |
| `test_trajectory.m` | Simulation loop: selects the controller and trajectory, runs the simulation and plots the results. |
| `runsim.m` | Entry point: sets the start and goal and calls `test_trajectory`. |
| `dynamics/` | Quadrotor equations of motion and Crazyflie parameters. |
| `utils/` | Rotation and quaternion helpers, plotting, map loading and collision checking (simulator framework). |
| `maps/` | Example environment maps. |
| `media/` | Figures used in this README. |

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

Built on the RBE 502 (WPI) Crazyflie quadrotor simulator framework. The design and tuning of the PD and LQR controllers, and the trajectory-tracking comparison, are my own work.

**Laasya Kodandaramu**
