# Pose-Estimation Human Following DDMR Mobile Robot
## using Model Predictive Control (MPC)

> Simulation of a Differential Drive Mobile Robot (DDMR) that follows a human using Model Predictive Control (MPC) and Pose-Estimation — built with PyBullet and Python.

---

## 📋 Table of Contents
- [Overview](#overview)
- [System Architecture](#system-architecture)
- [Mathematical Equations](#mathematical-equations)
- [Installation](#installation)
- [How to Run](#how-to-run)
- [GUI Controls](#gui-controls)
- [Project Structure](#project-structure)

---

## Overview

This project simulates a **human-following mobile robot** using:

- **PyBullet** — 3D physics simulation
- **TurtleBot (DDMR)** — Differential Drive Mobile Robot
- **Pose-Estimation** — measures relative position of human to robot (simulated Kinect)
- **MPC (CasADi + IPOPT)** — optimal controller that computes velocity commands

The simulated human walks randomly in the arena. The robot continuously estimates the human's position and uses MPC to follow at a safe distance.

---

## System Architecture

```
┌─────────────┐     position      ┌──────────────────┐
│    Kinect   │ ───────────────►  │  Pose-Estimation │
│  (Simulated)│                   │  d, bearing, ref  │
└─────────────┘                   └────────┬─────────┘
                                           │ ref_state
                                           ▼
┌─────────────┐   v, ω optimal   ┌──────────────────┐
│    DDMR     │ ◄──────────────  │  MPC Controller  │
│  (TurtleBot)│                   │  CasADi + IPOPT  │
└─────────────┘                   └──────────────────┘
```

---

## Mathematical Equations

### 1. DDMR Kinematic Model

The robot motion is described by the discrete-time kinematic equations:

$$x_{k+1} = x_k + v_k \cos\theta_k \cdot \Delta t$$

$$y_{k+1} = y_k + v_k \sin\theta_k \cdot \Delta t$$

$$\theta_{k+1} = \theta_k + \omega_k \cdot \Delta t$$

Where:
- $x_k, y_k$ — robot position at time step $k$
- $\theta_k$ — robot orientation (yaw) at time step $k$
- $v_k$ — linear velocity (m/s)
- $\omega_k$ — angular velocity (rad/s)
- $\Delta t = 0.05$ s — sampling time

---

### 2. Pose-Estimation

**Euclidean Distance:**

$$d = \sqrt{(x_{human} - x_{robot})^2 + (y_{human} - y_{robot})^2}$$

**Angle to Human:**

$$\alpha = \text{atan2}(y_{human} - y_{robot},\ x_{human} - x_{robot})$$

**Bearing Error:**

$$e_{bearing} = \text{atan2}\big(\sin(\alpha - \theta),\ \cos(\alpha - \theta)\big)$$

**Target Position (safe following distance):**

$$x_{target} = x_{human} - d_{safe} \cdot \cos\alpha$$

$$y_{target} = y_{human} - d_{safe} \cdot \sin\alpha$$

Where $d_{safe} = 0.8$ m is the safe following distance.

---

### 3. MPC Cost Function

The MPC minimizes the following cost function over prediction horizon $N$:

$$J = \sum_{i=1}^{N} \Big[ Q\big(e_{x,i}^2 + e_{y,i}^2 + 0.3\,e_{\theta,i}^2\big) + R\big(\Delta v_i^2 + \Delta\omega_i^2\big) \Big] + Q\big(e_{x,N}^2 + e_{y,N}^2\big)$$

Where:

| Symbol | Description | Default |
|--------|-------------|---------|
| $e_{x,i} = x_{target} - x_i$ | Position error in X | — |
| $e_{y,i} = y_{target} - y_i$ | Position error in Y | — |
| $e_{\theta,i} = \theta_{target} - \theta_i$ | Orientation error | weighted 0.3 |
| $\Delta v_i = v_i - v_{i-1}$ | Change in linear velocity | — |
| $\Delta\omega_i = \omega_i - \omega_{i-1}$ | Change in angular velocity | — |
| $Q$ | Position error weight | 15.0 |
| $R$ | Control effort weight | 0.5 |
| $N$ | Prediction horizon | 10 steps |

---

### 4. MPC Optimization Problem

At each time step $k$, solve:

$$\min_{v_0, \omega_0, \dots, v_{N-1}, \omega_{N-1}} J$$

Subject to:

$$X_{k+1} = f(X_k, U_k) \quad \text{(kinematic model constraint)}$$

$$X_0 = x_{now} \quad \text{(initial state constraint)}$$

$$0 \leq v_k \leq v_{max}$$

$$-\omega_{max} \leq \omega_k \leq \omega_{max}$$

Only the **first control input** $[v_0^*, \omega_0^*]$ is applied (Receding Horizon Principle), then the optimization is repeated at the next time step.

---

### 5. Wheel Velocity Conversion

The MPC outputs $(v, \omega)$ which are converted to individual wheel velocities:

$$v_{left} = \frac{v - \omega \cdot L}{R}$$

$$v_{right} = \frac{v + \omega \cdot L}{R}$$

Where:
- $L = 0.16$ m — half wheelbase (TurtleBot3)
- $R = 0.033$ m — wheel radius (TurtleBot3)

---


## GUI Controls

| Control | Description |
|---------|-------------|
| `PAUSE / RESUME` | Pause or resume simulation |
| `RESET` | Reset robot and human positions |
| `Sim Speed` | Adjust simulation speed (0.1× – 4×) |
| `Safe Distance` | Target following distance (m) |
| `v_max` | Max linear velocity (m/s) |
| `w_max` | Max angular velocity (rad/s) |
| `Q weight` | Position error weight in cost function |
| `R weight` | Control effort weight in cost function |
| `Human Speed` | Walking speed of simulated human |

---

## References

- Differential Drive Robot Kinematics
- Model Predictive Control — Rawlings & Mayne
- CasADi: A software framework for nonlinear optimization
- PyBullet Documentation
