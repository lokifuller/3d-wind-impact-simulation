# Cavity Flow 2D Computational Fluid Dynamics (CFD) using C++ and OpenGL

---

🔗 **[Various Cavity Flow Experiment Demos as well as Explanation for Methods and Code](https://lokifuller.netlify.app/lid_driven_cavity_demonstration)**  
*This README is a simplified project overview.*

---

## 🚀 Overview

This project simulates the **2D Lid-Driven Cavity problem** using **C++** for the numerical solver and **OpenGL** for real-time visualization. It is a classic problem in **Computational Fluid Dynamics (CFD)** which calculates how fluid moves in a closed box with a moving lid.

This project is extremely modular with easy implementation of different flows and obstacles as shown in my [cavity flow experiments](https://lokifuller.netlify.app/lid_driven_cavity_demonstration).

## 🧠 Concept

- A box filled with fluid has a **horizontally moving top lid**, which causes circulation inside the box.
- The simulation solves the **Navier-Stokes equations** for incompressible flow.
- The **SIMPLE algorithm** is used to iteratively solve velocity and pressure fields.
- A **staggered grid** is implemented to store pressure and velocity.
- **OpenGL** visualizes the fluid flow and velocity with live velocity vectors and color mapping.

## 🛠️ General Steps

1. **Define grid and physical parameters**  
   Set up the number of cells, time step, Reynolds number, etc.

2. **Initialize velocity and pressure fields**  
   Start with velocities and pressures equal to zero and assign lid velocity on the top wall.

3. **Apply boundary conditions**  
   Moving lid at the top, no-slip on all other walls, meaning velocity is 0 when hitting a wall.

4. **Predict velocities**  
   Use momentum equations to compute provisional velocities.

5. **Correct pressure**  
   Solve the pressure correction equation to ensure mass conservation (incompressibility).

6. **Update velocities and pressure**  
   Apply corrections to get correct velocity fields.

7. **Check for convergence**  
   If the solution has not converged, go to step 4.

8. **Visualize with OpenGL**  
   Render domain edges and velocity vectors in real time.

---

📌 Again, for experiments and full implementation explanation and code go here:  
👉 **[https://lokifuller.netlify.app/lid_driven_cavity_explanation](https://lokifuller.netlify.app/lid_driven_cavity_demonstration)**

---
