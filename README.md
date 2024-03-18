# Intelligent and Adaptive Control Systems, ECE AUTh, 2023-24

For the given system
$$M \ddot{q} + G \sin(q) + C \dot{q} = u$$
design and simulate 2 controllers. For each scenario, test the system's robustness under external disturbances.

## Controller 1

After linearizing the system, design and simulate a direct model reference adaptive control (D-MRAC) scheme using output feedback.

## Controller 2

Without linearizing the system, design and simulate a direct model reference adaptive control (D-MRAC) scheme using state feedback. The reference model
$$\dot{x}_r = A_r x_r + B_r  r(t)$$ 
should have a damping ratio equal to 0.7 and a natural frequency equal to 1 $rad/s$ .
