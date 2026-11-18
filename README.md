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

The quadrotor is controlled through four inputs: the total thrust `u1` and the three body moments `u2 = [Mx, My, Mz]`. Both controllers assume the quadrotor stays close to hover, with small roll and pitch angles. This keeps the dynamics simple enough to design around, at the cost of ruling out aggressive maneuvers.

The trajectories are generated as smooth polynomials in `traj/`, which give the desired position, velocity, acceleration and yaw at every moment:

- **Circle:** one loop of radius 1 m over 8 s, climbing gently to 0.5 m.
- **Diamond:** a diamond in the vertical (y–z) plane, flown as four 2 s legs with sharp corners.

---

## PD controller

The PD controller (`control/pid_controller.m`) works in two stages, because a quadrotor can only move sideways by tilting:

1. **Position loop.** It compares the current position and velocity with the desired ones and computes the acceleration needed to correct the error, `r̈_des = r̈_T + Kd·(ṙ_T − ṙ) + Kp·(r_T − r)`. The vertical part sets the thrust `u1`. The horizontal part is converted into the roll and pitch angles that would produce it.
2. **Attitude loop.** It then computes the moments `u2` needed to reach those angles and the desired yaw, using PD feedback on the angle and angular-rate errors.

The gains were tuned by hand: position `Kp = [11, 11, 12]` and `Kd = [10.4, 10.4, 10]`, and attitude `Kp = [74, 74, 20]` and `Kd = [58, 58, 15]` for roll, pitch and yaw.

---

## LQR controller

The LQR controller (`control/lqr_controller.m`) treats the quadrotor as a single system with 12 states (position, Euler angles, linear velocity and body rates) and computes all four inputs at once.

Since LQR needs a linear model, the near-hover dynamics are linearized at every time step around the point where the quadrotor *should* be: the desired state from the trajectory, with hover thrust as the nominal input. This gives matrices `A` and `B` that describe how small errors evolve. MATLAB's `lqr(A, B, Q, R)` then finds the gain `K` that corrects those errors while balancing accuracy against control effort, and the command is `u = K·(x_des − x) + u_des`.

The weights set what the controller cares about most. `Q` puts the highest weight on position, velocity and yaw, so the quadrotor stays on the path and doesn't drift while turning. `R` penalizes thrust less than the moments.

---

## Results

Both controllers complete both trajectories, but LQR follows them more closely. In the paths at the top of this page, LQR stays close to the diamond's edges, while the PD path overshoots the corners and drifts off some of the edges.

The difference is clearest in velocity on the diamond, where the sharp corners demand quick changes of direction:

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

PD overshoots at each corner and keeps oscillating after the trajectory ends, while LQR tracks the desired velocity smoothly. The trade-off is computation: re-linearizing and solving for a new gain at every step makes LQR much slower to run than PD. PD remains simpler to implement and tune, but is less accurate on dynamic trajectories like these.

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
