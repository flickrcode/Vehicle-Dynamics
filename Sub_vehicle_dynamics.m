function  [vDot,omegaDotf,omegaDotr,Fzf,Fzr]=...
    Sub_vehicle_dynamics(v,Tdrivf,Tdrivr,slipf,slipr,slope,road_cond,CONST)

% Read parameters
c_d=CONST.c_d;
A_f=CONST.A_f;
air=CONST.air;

M=CONST.M;
Ir=CONST.Ir; %[kg*m^2] Moment of inertia for two wheels and axle 
g=CONST.g;

h=CONST.h;
L=CONST.L;
L_f=CONST.Lf;
L_r=CONST.Lr;

R=CONST.R;
f_r=CONST.f_r; % rolling resistance coefficient

% Calculate air resistance 
Fair=0.5*c_d*A_f*air*v^2;

% Magic Tire Formula
muf=Sub_magic_tireformula(slipf,road_cond);
mur=Sub_magic_tireformula(slipr,road_cond);

% Write the explicit solutions here!

Fzf = -(L_f*M*g*h*mur*cos(slope) - L*L_r*M*g*cos(slope) + L_r*M*g*h*mur*cos(slope))/(L*(L + h*muf - h*mur));    %[N] front normal force
Fzr = (L*L_f*M*g*cos(slope) + L_f*M*g*h*muf*cos(slope) + L_r*M*g*h*muf*cos(slope))/(L*(L + h*muf - h*mur));    %[N] rear normal force
omegaDotf = (L^2*Tdrivf + L*Tdrivf*h*muf - L*Tdrivf*h*mur - L*L_r*M*R*f_r*g*cos(slope) - L*L_r*M*R*g*muf*cos(slope) + L_f*M*R*f_r*g*h*mur*cos(slope) + L_r*M*R*f_r*g*h*mur*cos(slope) + L_f*M*R*g*h*muf*mur*cos(slope) + L_r*M*R*g*h*muf*mur*cos(slope))/(Ir*L*(L + h*muf - h*mur)); %[rad/s/s] rotational acceleration, front
omegaDotr = -(L*Tdrivr*h*mur - L*Tdrivr*h*muf - L^2*Tdrivr + L*L_f*M*R*f_r*g*cos(slope) + L*L_f*M*R*g*mur*cos(slope) + L_f*M*R*f_r*g*h*muf*cos(slope) + L_r*M*R*f_r*g*h*muf*cos(slope) + L_f*M*R*g*h*muf*mur*cos(slope) + L_r*M*R*g*h*muf*mur*cos(slope))/(Ir*L*(L + h*muf - h*mur)); %[rad/s/s] rotational acceleration, rear
vDot = -(Fair*L + Fair*h*muf - Fair*h*mur + L*M*g*sin(slope) - L_f*M*g*mur*cos(slope) - L_r*M*g*muf*cos(slope) + M*g*h*muf*sin(slope) - M*g*h*mur*sin(slope))/(M*(L + h*muf - h*mur));     %[m/s/s] acceleration

end