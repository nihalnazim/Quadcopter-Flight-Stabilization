# Quadcopter Flight Stablization

## Project Description


The purpose of this project is to further improve upon the preexisting program that was created with the intent of actively stabilizing a 3D printed quadcopter U.A.V. To achieve this the program would need to analyze the data collected by an I.M.U., from that data it would need to separate sensor noise, caused by vibrations and consumer grade components. From that filtered data it would need to utilize it to calculate the positional vectors and then respond accordingly to stabilize the U.A.V.


The previous iteration of the program was able to simultaneously activate the four brushless motors, and their designated E.S.Cs. However despite several attempts no data was recorded. The E.S.Cs and the motors would both be armed and then activated fully, yet the program would show no sign of recording data. And to further complicate the situation the Raspberry Pi Pico was experiencing port communication issues. To improve upon this the Raspberry Pi Pico needs to be replaced, and data recording issue would need to be addressed, before creating the P.I.D controller to analyze the data set and finally input create the output for the motors.

---

## Objectives

- Replace the drones faulty hardware
- Debug the Data recording aspect of the preexisting program
- Improve the drone’s stabilization control logic
- Create a reliable and consistent processing loop for the I.M.U.
- Reduce oscillations and unstable motor responses cause my sensor noise

---


## Approach

- Utilize the preexisting quadcopter control program
- Alter it to fix the data recording issue
- Replace the current Raspberry Microcontroller to allow for reliable testing
- Identify weaknesses in the current control loop and sensor integration
- Create the filter to sort the I.M.U. data
- Restructure code into clearer modules (sensor input, control logic, motor output)
- Refine control logic to produce smoother and more stable responses
- Alter the program to allow for parallel processing 

---

## Expected Challenges

- Hardware communication issues will arise affecting sensor reliability and testing capabilities
- Difficulties in resolving the data recording of the I.M.U data
- Noisy and inconsistent I.M.U data
- Maintaining stable behavior (not over or under correcting) within discrete timestep updates
- Integrating improved logic with existing code structure

---

## Timeline

Week 1: Review existing code and identify weaknesses
Week 2: Restructure control loop and improve sensor handling
Week 3: Implement control improvements and test basic behavior
Week 4: Tune parameters and analyze stability and response
Week 5: Finalize results, comparisons, and documentation

---

## Planned Repository Structure

```
project/
├── README.md
├── requirements.txt
├── main.py
│
├── src/
│   ├── attitude.py
│   ├── battery.py
│   ├── calibration.py
│   ├── control.py
│   ├── imu.py
│   ├── motors.py
│   └── safety.py
│
├── docs/
│   ├── proposal_notes.md
│   └── design_notes.md
│
├── results/
│   ├── figures/
│   └── logs/
│
├── tests/
│
└── legacy/
    └── previous_c_version/
```

---

## Tools and Libraries

- Python
- NumPy
- Matplotlib
- Microcontroller development tools (for deployment and testing)

---

## Stretch Goals (Optional)

- Visualize (graph) stabilization performance over time
- Compare multiple control strategies (e.g., proportional vs PID)
- Simulate control behavior before hardware testing

## Previous Grant Proposal 

[View Full Proposal](Drone_Stability_Grant_Proposal_Final-2.pdf)
---
