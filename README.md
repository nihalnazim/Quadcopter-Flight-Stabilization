# Quadcopter Flight Stabilization Simulation

This project is a Python-based numerical simulation of a simplified quadcopter stabilization system. The simulation focuses on single-axis roll stabilization using a proportional-derivative (PD) controller, discrete-time numerical integration, and an IMU-style sensor model.

The goal of the project is to model how a quadcopter can correct its roll angle after an initial tilt or disturbance and return to a stable equilibrium position.

---

## Project Description

The simulation models the roll angle of a quadcopter using two main state variables:

- `theta`: roll angle in radians  
- `omega`: angular velocity in radians per second  

The system uses a PD controller to compute a corrective control input. The controller attempts to drive the roll angle back to zero while damping angular velocity to avoid unstable oscillations.

A disturbance is applied at `t = 1.0 s` to test whether the system can reject a sudden change in orientation and return to equilibrium.

---

## Numerical Methods Used

The simulation uses discrete-time numerical integration with:

```python
dt = 1 / 250
```

The rotational dynamics are modeled as:

```text
theta_dot = omega
omega_dot = (torque - damping * omega) / inertia
```

The state is updated using a **semi-implicit Euler method**, where angular velocity is updated before the angle. This improves stability compared to a basic Euler update.

---

## Control Method

The controller uses a PD control law:

```text
u = kp * error - kd * angular_rate
```

Where:
- `kp` controls responsiveness to angle error  
- `kd` adds damping using angular velocity  
- `u` is the control input  

The derivative term uses measured angular rate instead of numerically differentiating the error, reducing noise amplification.

---

## IMU Simulation

The project includes an IMU simulator that models accelerometer and gyroscope readings.

- The **accelerometer** estimates orientation using the gravity vector  
- The **gyroscope** measures angular velocity  
- Gaussian noise is added for realism  

A **complementary filter** combines both to estimate roll angle.

### Modes

```python
use_imu = False
```
Uses true roll angle (ideal case)

```python
use_imu = True
```
Uses simulated IMU + filter (realistic case)

---

## Repository Organization

```text
main.py
src/
    attitude.py
    control.py
    imu.py
    motors.py
    safety.py
    calibration.py
    battery.py
required-libraries.txt
```

---

## Important Files

- `main.py`  
  Runs the simulation and generates plots  

- `src/control.py`  
  PD controller implementation  

- `src/motors.py`  
  Rotational dynamics and integration  

- `src/attitude.py`  
  State estimation + complementary filter  

- `src/imu.py`  
  IMU simulation  

- `src/safety.py`  
  Safety constraints  

---

## How to Run

Clone the repository:

```bash
git clone https://github.com/nihalnazim/Quadcopter-Flight-Stabilization.git
cd Quadcopter-Flight-Stabilization
```

Install dependencies:

```bash
pip install -r required-libraries.txt
```

Run the simulation:

```bash
python main.py
```

---

## Required Libraries

```
numpy
matplotlib
```

---

## Output

The simulation produces:

1. Roll angle vs time  
2. Angular velocity vs time  
3. Control input vs time  

When IMU mode is enabled, the first plot compares true vs measured roll angle.

---

## Results

The system successfully stabilizes after an initial roll offset and a disturbance at `t = 1.0 s`.

- The response is **slightly underdamped**, showing small overshoot  
- The system remains stable and converges to zero  
- The IMU-based version shows realistic noise but still stabilizes correctly  

---

## Summary

This project demonstrates:
- Numerical integration of rotational dynamics  
- PD control for stabilization  
- Sensor modeling and noise  
- Complementary filtering for state estimation  