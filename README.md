# PD & LQR Control for Quadrotor Trajectory Tracking

A MATLAB simulation that designs, implements, and **compares two controllers — Proportional-Derivative (PD) and Linear Quadratic Regulator (LQR)** — for autonomous quadrotor flight: hovering at a target, and tracking full 3D trajectories. Both controllers are run on a **circle** and a **diamond** trajectory and compared on tracking accuracy and computational cost.

This project was done as part of the final project for the RBE 502: Robot Control course at WPI. Built on the RBE 502 Crazyflie quadrotor simulator.

---

## Overview

The quadrotor is controlled near a hovering equilibrium (small roll/pitch/yaw), and the goal is to compute the four control inputs — total thrust `u1` and body moments `u2 = [Mx, My, Mz]` — that drive it along a desired trajectory. Two controllers are implemented and compared:

- **PD controller** — a cascaded position + attitude controller. A position loop turns position/velocity error into a desired acceleration (and thrust `u1`); the near-hover dynamics then give the desired roll/pitch, and an attitude loop produces the moments `u2`.
- **LQR controller** — the nonlinear dynamics are linearized **about the desired state at each timestep** into `A`, `B`, and MATLAB's `lqr()` solves for the optimal gain `K`; the control is `u = K·(x_des − x) + u_des`, with `u_des` the nominal hover input.

## Approach

**PD controller** (`control/pid_controller.m`)
- **Position loop:** `r̈_des = r̈_T + Kd·(ṙ_T − ṙ) + Kp·(r_T − r)`, giving total thrust `u1 = m·g + m·r̈_des,z`.
- **Attitude loop:** desired roll/pitch are back-computed from the commanded horizontal acceleration and desired yaw; the moments are `u2 = I·(Kp,att·(e_euler) + Kd,att·(e_omega))`.
- **Gains used:** position `Kp = [11, 11, 12]`, `Kd = [10.4, 10.4, 10]`; attitude `Kp = [74, 74, 20]`, `Kd = [58, 58, 15]` (roll, pitch, yaw).

**LQR controller** (`control/lqr_controller.m`)
- Full 12-state model `x = [position; Euler angles; linear velocity; angular velocity]`.
- At each step the dynamics `ẋ = f(x,u)` are linearized around the desired state/input to build `A` (12×12) and `B` (12×4), then `K = lqr(A, B, Q, R)`.
- **Weights:** `Q = diag([10,10,10, 1,1,10, 10,10,10, 0.1,0.1,0.1])` (higher weight on position, velocity, and yaw for tighter tracking), `R = diag([0.1, 1, 1, 1])`.
- Control law: `u = K·(x_des − x) + u_des`, `u_des = [m·g; 0; 0; 0]`.

Desired states (position, velocity, acceleration, yaw) come from the trajectory generators in `traj/` — `circle.m` and `diamond.m`.

## Results

Both controllers successfully track the circle and diamond trajectories. Comparing them:

- **LQR tracks more accurately** — position and especially **velocity error are noticeably lower** than PD. The quadrotor follows the desired path more closely under LQR.
- **PD is simpler** and easy to implement, but shows higher velocity-tracking error on these dynamic trajectories.
- **Trade-off:** LQR's accuracy comes at a **computational cost** — re-linearizing and solving the Riccati equation every timestep makes it significantly slower to run than PD.

## Repository structure

| Path | Description |
|------|-------------|
| `control/pid_controller.m` | **PD** position + attitude controller. |
| `control/lqr_controller.m` | **LQR** controller (per-step linearization + `lqr()`). |
| `traj/circle.m`, `traj/diamond.m`, `traj/jump.m` | Trajectory generators (desired pos/vel/acc/yaw). |
| `dynamics/` | Quadrotor equations of motion and Crazyflie parameters. |
| `runsim.m` | Main simulation entry point. |
| `test_trajectory.m` | Trajectory-tracking test harness. |
| `utils/` | Rotation/quaternion helpers, plotting, map loading, collision checks. |
| `maps/` | Example environment maps. |

## Running it

Requires MATLAB (R2020a or later).

```matlab
% from the repo root in MATLAB:
runsim
```

Select the controller by pointing the simulator at `pid_controller.m` or `lqr_controller.m`, and choose the trajectory (`circle` / `diamond`) in the run/trajectory setup. The simulator animates the flight and plots position and velocity tracking (desired vs. actual).

## Acknowledgements

Built on the RBE 502 (WPI) Crazyflie quadrotor simulator framework. Controller design and tuning (PD and LQR) and the trajectory-tracking comparison are my own work.

**Laasya Kodandaramu**
