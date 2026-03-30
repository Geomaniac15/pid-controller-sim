# 3D PID Controller Simulation

A Python-based simulation of a 3D control system using PID control, feedforward dynamics, and stochastic optimisation.
The system tracks a moving target under realistic conditions including noise, drag, and wind disturbances.

---

## Overview

This project simulates a point-mass system attempting to follow a dynamic 3D trajectory using:
- PID control (Proportional, Integral, Derivative)
- Feedforward control (target velocity and acceleration)
- Measurement noise + exponential filtering
- Environmental disturbances (wind, drag, varying mass)
- Parameter optimisation via stochastic search

The goal is to produce a controller that is:
- Stable
- Responsive
- Robust to uncertainty

---

## Demo

![Simulation Demo](demo.gif)

---

## Installation

```bash
git clone https://github.com/Geomaniac15/pid-controller-sim.git
cd pid-controller-sim
pip install matplotlib
```

---

## Features

### Control System
- Full 3D PID controller
- Integral windup limiting
- Velocity-based derivative term
- Feedforward acceleration and velocity

### Physics Simulation
- Mass-dependent dynamics
- Wind disturbances (time-varying)
- Linear drag forces
- Speed limiting

### Noise & Filtering
- Uniform noise applied to measurements
- Exponential smoothing filter (alpha)

### Optimisation
- Randomised parameter search over:
    - Kp, Kd, Ki
    - alpha (filter strength)
- Evaluates performance across multiple randomised environments
- Uses a cost function balancing:
    - tracking accuracy
    - smoothness
    - control effort
    - success conditions

### Visualisation
- Real-time 3D animation using Matplotlib
- Displays:
  - Actual trajectory
  - Measured (noisy) trajectory
  - Moving target

---

## Usage

### Run simulation with current parameters

```python
MODE = 'run'
```

### Optimise controller parameters

```python
MODE = 'optimise'
```

This will:
1. Search for optimal parameters
2. Print the best result
3. Automatically run the simulation using those parameters

---

## Parameters

| Parameter | Description |
|----------|------------|
| Kp | Proportional gain (responsiveness) |
| Kd | Derivative gain (damping) |
| Ki | Integral gain (removes steady-state error) |
| alpha | Filter strength (noise smoothing) |
| dt | Simulation timestep |
| limit | Integral windup cap |

---

## Cost Function

The optimiser minimises:
- Distance to target (weighted)
- Velocity magnitude (smoothness)
- Control effort (acceleration penalties)
- Failure penalty (if target not reached)

---

## Success Condition

The system is considered successful if:
- It remains within a small radius of the target
- For a sustained period of time

---

## Example Output

```text
Kp: 4.62
Kd: 2.86
Ki: 0.06
alpha: 0.09

Best cost: 1515.15
Success: True
Found at timestep 6.95
```

---

## Future Improvements
- Smarter optimisation (Bayesian / CMA-ES)
- Better visualisation of forces (wind, drag, control)
- Vectorised simulation (NumPy)
- Multi-agent or obstacle avoidance
- Reinforcement learning controller comparison

---

## Concepts Demonstrated
- Classical control theory (PID)
- Feedforward control
- Noise filtering
- Simulation of dynamical systems
- Stochastic optimisation
- Robust control under uncertainty

---

## Notes
- Matplotlib 3D rendering is not optimised for performance
- Results vary due to randomised environments
- This is a simulation, not a physically accurate flight model

---

## Author

George :)