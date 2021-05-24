% MECA 482 TEAM 8 FURUTA PENDUUM

%Pendulum parameters
g = 9.81;
Mp = 0.127;
Lp = 0.33655;
lp = Lp/2;
Jp = Mp*Lp^2/12;
Bp = 0.0024;
Dr = .015;
Dp = .015;

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

VMAX_AMP = 24;
IMAX_AMP = 4;
K_AMP = 1;

% Potentiometer Sensitivity (rad/V)
K_POT = -(352 * pi / 180 / 10); 

% Encoder Resolution (rad/count)    
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

% Intermediate calculations for finding desired pole placement
sigma = zeta .* wn;
wd = wn .* (sqrt(1 - (zeta.^2)));

% Pole Placement
p1 = complex(-sigma,wd);        %p1 = -sigma + wd j;
p2 = complex(-sigma,-wd);       %p2 = -sigma - wd j;
p3 = -30;
p4 = -40;
new_poles = [p1 p2 p3 p4];      % Assembles desired pole placement
K = Control_FurutaPendulum(A,B,zeta,wn,p3,p4);

% Calculate the new A matrix
A2 = (A - B*K);     

% New State Space Representation
new_sys = ss(A2, B, C, D);      % New State Space System 
Trans = tf(new_sys);
figure(1);
h = pzplot(new_sys);            % the x indicates a pole and o indicates a zero
grid on;
figure(2);
h2 = iopzplot(new_sys);

% Plot the compensated step response of the closed-loop transfer function
figure(5);
step(new_sys)

% Quanser Hardware Values
K_ENC = 2 * pi / ( 4 * 1024 );  % Encoder Sensitvity
K_POT = -(352 * pi / 180 / 10); % Potentiometer Sensitivity
wcf_1 = 2 * pi * 10.0;          % Cut-Off Freq
wcf_2 = 2 * pi * 10.0;          % Cut-Off Freq
epsilon = 12.0 * pi / 180;
eta_g = 0.90;
eta_m = 0.69;
Er = Mp*g*Lp;

% Initialize API
sim=remApi('remoteApi');
sim.simxFinish(-1);
clientID=sim.simxStart('192.168.1.151',19999,true,true,5000,5);
if (clientID>-1)
    disp('Connected to remote API server');                     
    set_param('Linearization', 'SimulationCommand', 'start')               
end