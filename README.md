# Furuta Pendulum
**Group 8**

Andres Sanchez

Cody Breuninger

Jimmie Whitton

Joe Overcashier
Max Rohde

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

From Eq.(5),  x1'= x3,  x2'=x4 where x can be substituted for each variable: 𝜭, 𝜶, 𝜭', and 𝜶'. From these the matrices A and B can be found. In Eq.(14) and Eq.(15) x3' and  x4' are described.

![eq 14 eq 15](https://user-images.githubusercontent.com/84546893/119276911-86428f00-bbd1-11eb-9710-dbecc135b3e6.png)

From these equations A and B can be described from Eq.(3) below where Eq.(16) represents the A matrix and Eq.(17) represents the B matrix.

![eq 16 eq 17](https://user-images.githubusercontent.com/84546893/119276918-8e9aca00-bbd1-11eb-839b-49d308a076de.png)

# Controller Design and Simulation

# Appendix A: Simulation Code
g = 9.81;

%Pendulum Output parameters ( PEND_TYPE, 'MEDIUM_12IN') 
Mp = 0.127;
Lp = 0.33655;     
lp = Lp/2; 
Jp = Mp*Lp^2/12; 
Bp = 0.0024;  

%Arm Output parameters 
Mr = 0.257;            
Lr = 0.2159;          
lr = (2+7/16)*0.0254;        
Jr = Mr*Lr^2/12;     
Br = 113.5e-3;   

Kgi = 14;       
Kge = 5;       
Kg = Kgi * Kge; 


% Motor Inertia 
Jm_rotor = 3.9e-7;      
Jtach = 0;
Jm = Jm_rotor + Jtach;
ng = 0.9;                                  
nm =0.69;                                 
kt = 1.088 * 0.2780139 * 0.0254; %=.00767           
Rm = 2.6;                                  
km = 0.804 / 1000 * (60 / ( 2 * pi )); %=.00767      

%Amplifier Max Output Voltage (V) and Output Current (A)
%VoltPAQ
VMAX_AMP = 24;
IMAX_AMP = 4;
K_AMP = 1;

% Potentiometer Sensitivity (rd/V)
K_POT = -(352 * pi / 180 / 10);

% Encoder Resolution, for a quadrature encoder, (rd/count)    
K_ENC = 2 * pi / ( 4 * 1024 );

Jt = (Jp.*Mp.*(Lr.^2)) + (Jr.*Jp) + ((1./4).*Jr.*Mp.*(Lp.^2)); 

Jt = Jr*Jp+Mp*(Lp/2)^2*Jr+Jp*Mp*Lr^2;
A = [ 0 0 1 0; 0 0 0 1; 0 Mp^2*(Lp/2)^2*Lr*g/Jt -Dr*(Jp+Mp*(Lp/2)^2)/Jt -Mp*(Lp/2)*Lr*Dp/Jt; 0 Mp*g*(Lp/2)*(Jr+Mp*Lr^2)/Jt -Mp*(Lp/2)*Lr*Dr/Jt -Dp*(Jr+Mp*Lr^2)/Jt];
B = [ 0; 0; (Jp+Mp*(Lp/2)^2)/Jt; Mp*(Lp/2)*Lr/Jt];
C = eye(2,4);
D = zeros(2,1);

% Add actuator dynamics

A(3,3) = A(3,3) - Kg^2*kt*km/Rm*B(3);
A(4,3) = A(4,3) - Kg^2*kt*km/Rm*B(4);
B = Kg * kt * B / Rm;
states = {'theta' 'theta_dot' 'alpha' 'alpha_dot'};
inputs = {'u'};
outputs = {'theta' ; 'alpha'};

system_FurutaPendulum = ss(A,B,C,D, 'statename',states, 'inputname', inputs, 'outputname',outputs) %open loop system
system_TransferF = tf(system_FurutaPendulum) %Transfer Function

%System Requirements 
zeta = .07;
wn = 4;
d_p3 = -30; %Pole Location
d_p4 = -40; %Pole Location
%Feedback Gain

% Intermediate calculations for finding desired pole placement
sigma = zeta .* wn;
wd = wn .* (sqrt(1 - (zeta.^2)));

% Calculated Complex conjugate dominant poles
p1 = complex(-sigma,wd);    %p1 = -sigma + wd j;
p2 = complex(-sigma,-wd);   %p2 = -sigma - wd j;
p3 = -30
p4 = -40
desired_poles = [p1 p2 p3 p4];      % Assembles desired pole placement

[K, prec, message] = place(A,B,desired_poles);       % Calculates Controller Gains

K = Control_FurutaPendulum(A,B,wn,zeta,d_p3,d_p4);
        
%% Plot Closed Loop Result - Control Design via Transformation
% Similar to Ex. 12.4 out of the textbook
% Form the compensated State space matrices using the controller gain
A2 = A - B*K     


Tss = ss(A2, B, C, D);   % Forms a LTI (Linear Time Invariant) State-Space Object
T = tf(Tss);                        % Create T(s) - Transformed Model;
T = minreal(T);                     % Cancel common terms and display T(s)

new_poles = pole(T);     % Displays the poles of T

% Plot the compensated step response of the closed-loop transfer function
figure(10);
step(Tss)
title('Compensated Step Response');

% Calculated parameters 
[K2, prec_new, message_new] = place(A2,B,desired_poles);

% Initialize API
sim=remApi('remoteApi');

% Using the prototype file (remoteApiProto.m)
sim.simxFinish(-1);

% Note: Just in case, close all opened connections
clientID=sim.simxStart('192.168.1.151',19999,true,true,5000,5);

if (clientID>-1)
    
    disp('Connected to remote API server');                     
    set_param('Furuta_PendulumTEAM', 'SimulationCommand', 'start')               

    %sim.simxSetJointTargetVelocity(clientID,j1,pos_val,sim.simx_opmode_streaming);
    
    while(1)  % In this while loop, we will have the communication
        
        % Step 1: Initialize Joint and Link Handles where you defined your joints and links under the set coppelia names
        [err_code_1_object, Jr] = sim.simxGetObjectHandle(clientID,'Jr',sim.simx_opmode_blocking);
        [err_code_2_object, Jp] = sim.simxGetObjectHandle(clientID,'Jp',sim.simx_opmode_blocking);
        [err_code_3_object, Lr] = sim.simxGetObjectHandle(clientID,'Lr',sim.simx_opmode_blocking);
        [err_code_4_object, Lp] = sim.simxGetObjectHandle(clientID,'Lp',sim.simx_opmode_blocking);
                %%if errorCode is not vrep.simx_return_ok, this does not mean there is an error:            
                %%it could be that the first streamed values have not yet arrived, or that the signal            
                %%is empty/non-existent  
                
% Sensor Data from Coppelia
        [err_code_1_position, theta_Jr] = sim.simxGetJointPosition(clientID, Jr , sim.simx_opmode_streaming);
        [err_code_2_position, alpha_Jp] = sim.simxGetJointPosition(clientID, Jp , sim.simx_opmode_streaming);
        [err_code_1_velocity, linear_velo_theta, theta_dot_Jr] = sim.simxGetObjectVelocity(clientID, Lr ,sim.simx_opmode_streaming);
        [err_code_2_velocity, linear_velo_alpha, alpha_dot_Jp] = sim.simxGetObjectVelocity(clientID, Lp ,sim.simx_opmode_streaming);
        
        theta_dot_about_z = theta_dot_Jr(3);
        alpha_dot_about_x = alpha_dot_Jp(1);
        alpha_dot_about_y = alpha_dot_Jp(2);
        
                 pause(.01);
                
% Actuator Data from Simulink
            % We receive the sensor data from Simulink model 'Furuta_Pendulum' and 'To Workspace theta' block via RuntimeObject
            theta_s = get_param('Furuta_PendulumTEAM/To Workspace theta','RuntimeObject');
            theta_s.InputPort(1).Data;    % Receive the data
            %simout_t = sim('Furuta_Pendulum/To Workspace theta','SimulationMode','normal', 'SaveState','on','StateSaveName','xoutNew','SaveOutput','on','OutputSaveName','youtNew');
            %theta_s = simout_t.get('youtNew');
            %assignin('base','theta_s',theta_s);

            
            % We receive the sensor data from Simulink model 'Furuta_Pendulum' and 'To Workspace theta' block via RuntimeObject
            alpha_s = get_param('Furuta_PendulumTEAM/To Workspace alpha','RuntimeObject');
            alpha_s.InputPort(2).Data;    % Receive the data
            %simout_a = sim('Furuta_Pendulum/To Workspace alpha','SimulationMode','normal');
            %alpha_s = simout_a.get('alpha_s');
            %assignin('base','alpha_s',alpha_s);
            
% Coppelia motion dictated by Simulink model - Will uncomment when I manage to get simulink to write to the work space
        % Note: Unknown if this code is correct couldn't get past simulink 'to workspace' block error (dot indexing error) so couldn't get to debug coppellia sim motion
            [err_code_1_set_target_position] = sim.simxSetJointTargetPosition(clientID, Jr, theta_s, sim.simx_opmode_streaming);
            [err_code_2_set_target_velocity] = sim.simxSetJointTargetVelocity(clientID, Jr, theta_dot_s, sim.simx_opmode_streaming);
            
   end
end
# References

J. Á. Acosta, "Furuta's Pendulum: A Conservative Nonlinear Model for Theory and Practise", Mathematical Problems in Engineering, vol. 2010, Article ID 742894, 29 pages, 2010. https://doi.org/10.1155/2010/742894



