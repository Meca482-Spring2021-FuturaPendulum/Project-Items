# Furuta Pendulum
**Group 8**

<center> Andres Sanchez

Cody Breuninger
<center>Jimmie Whitton

Joe Overcashier
<center>Max Rohde

Mitchell Cabral

# Table of Contents
**1. Introduction**

**2. Modeling**

**3. Controller Design and Simulation**

**4. Appendix A: MATLab Code**

**5. References**

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

**Figure 2.** Pendulum Direction Conventions

![eq 1 eq 2](https://user-images.githubusercontent.com/84546893/119276696-61015100-bbd0-11eb-9c57-1db25746f288.png)

Eq.(1) and Eq.(2) were linearized to develop a state-space model for the pendulum. The state-space equations are defined in Eq.(3) and Eq.(4).

![eq 3 eq 4](https://user-images.githubusercontent.com/84546893/119276749-afaeeb00-bbd0-11eb-8b05-4f82e3e8e89a.png)

In these equations, x represents the state, y represents the output, and u represents the control input. The state and output for the Furuta pendulum are defined in Eq.(5) and Eq.(6).

![eq 5 eq 6](https://user-images.githubusercontent.com/84546893/119276790-e422a700-bbd0-11eb-8e15-1392640b17b1.png)

To linearize Eq.(1) and Eq.(2), it is assumed that all initial conditions for all the variables, 0, 0, 0,and 0, are zero. With these conditions, Eq.(7) and Eq.(8) are derived.

![eq 7 eq 8](https://user-images.githubusercontent.com/84546893/119276816-0b797400-bbd1-11eb-998f-890dc13b3e73.png)

Using Eq.(7) and Eq.(8), the following matrix,Eq.(9) is created where the determinant of the 2x2 matrix is shown in Eq.(10.1) with Eq.(10.2) being its simplification. 

![eq 9 eq 10](https://user-images.githubusercontent.com/84546893/119276837-28ae4280-bbd1-11eb-9f8b-cc0399337ea7.png)

Next, the angular acceleration of the pendulum and rotary arm are solved for in Eq.(11).

![eq 11](https://user-images.githubusercontent.com/84546893/119276849-349a0480-bbd1-11eb-8bf3-24dbe5fdc262.png)

From Eq.(11), the first row of the matrix equates to Eq.(12.1), simplified in Eq.(12.2), while the second is described in Eq.(13.1) and simplified in Eq.(13.2).

![eq 12 eq 13](https://user-images.githubusercontent.com/84546893/119276877-5a270e00-bbd1-11eb-83c3-04c572ab4f45.png)

From Eq.(5),  x1= x3,  x2=x4where x can be substituted for each variable: , ,  ,and  . From these the matrices A and B can be found. In Eq.(14) and Eq.(15)  x3and  x4are described.

![eq 14 eq 15](https://user-images.githubusercontent.com/84546893/119276911-86428f00-bbd1-11eb-9710-dbecc135b3e6.png)

From these equations A and B can be described from Eq.(3) below where Eq.(16) represents the A matrix and Eq.(17) represents the B matrix.

![eq 16 eq 17](https://user-images.githubusercontent.com/84546893/119276918-8e9aca00-bbd1-11eb-839b-49d308a076de.png)

# Controller Design and Simulation

# Appendix A: Simulation Code

# References

J. Á. Acosta, "Furuta's Pendulum: A Conservative Nonlinear Model for Theory and Practise", Mathematical Problems in Engineering, vol. 2010, Article ID 742894, 29 pages, 2010. https://doi.org/10.1155/2010/742894



