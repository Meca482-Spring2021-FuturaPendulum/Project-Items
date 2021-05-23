# Furuta Pendulum
Rotary inverted pendulum framework created for MECA 482, Control System Design, at CSU Chico

# **Introduction**

The Furuta Pendulum was invented in 1992 at the Tokyo Institute of Technology by Katsuhisa Furuta. It is composed of an arm which can rotate around a vertical axis to support an 
inverted pendulum that can rotate freely about a horizontal axis. The pendulum arm is attached to an encoder that reads the angle of the pendulum and creates a response that 
drives a motor to rotate an arm positioned 90° relative to the pendulum to keep it balanced upright. The project's goal is to create a control system algorithm in Matlab that can 
model this both linearly and nonlinearly in CoppeliaSim. An example of a Furuta/rotary pendulum manufactured by Quanser is shown below in Figure 1.

![fig 1](https://user-images.githubusercontent.com/84546893/119276584-c7d23a80-bbcf-11eb-8dbb-872fc528ce85.png)

**Figure 1.** Example of Furuta Pendulum (QUANSER QUBE)

# **Modeling**

The system is underactuated since it employs only control input and consists of two degrees of freedom. Only one degree of freedom, the angular position of the rotary arm, 𝜭, is actively controlled. The angular position of the pendulum, 𝜶, is dependent upon the position of the rotary arm and the position of its own center of mass relative to the rotary arm. Table 1 below lists all parameters used in the calculations.

![table 1](https://user-images.githubusercontent.com/84546893/119276655-20093c80-bbd0-11eb-97a4-9fb43ccccc0d.png)

**Table 1.** Furuta Pendulum Parameters

The non-linear equations to represent the motion of the pendulum, Eq.(1) and Eq.(2), were derived from the representation that defines the pendulum’s motion shown in Figure 2.

![fig 2](https://user-images.githubusercontent.com/84546893/119276700-66f73200-bbd0-11eb-9581-d03918bab300.png)

**Figure. 2** Pendulum Direction Conventions

![eq 1 eq 2](https://user-images.githubusercontent.com/84546893/119276696-61015100-bbd0-11eb-9c57-1db25746f288.png)

Eq.(1) and Eq.(2) were linearized to develop a state-space model for the pendulum. The state-space equations are defined in Eq.(3) and Eq.(4).
![eq 3 eq 4](https://user-images.githubusercontent.com/84546893/119276749-afaeeb00-bbd0-11eb-8b05-4f82e3e8e89a.png)

In these equations, x represents the state, y represents the output, and u represents the control input. The state and output for the Furuta pendulum are defined in Eq.(5) and Eq.(6).
![eq 5 eq 6](https://user-images.githubusercontent.com/84546893/119276790-e422a700-bbd0-11eb-8e15-1392640b17b1.png)


