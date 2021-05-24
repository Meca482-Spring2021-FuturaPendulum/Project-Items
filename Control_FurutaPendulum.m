function K = Control_FurutaPendulum(A,B,zeta,wn,p3,p4)

%Dominant Poles along Real axis
sigma = zeta*wn;
%Dominant Poles along imaginary axis
wd=wn*(1-zeta)^0.5
%Desired Poles( -30 and -40 )
poles = [-sigma+j*wd, -sigma-j*wd, p3, p4];
%Control gain with Pole Placement command
K = acker(A, B, poles)

end