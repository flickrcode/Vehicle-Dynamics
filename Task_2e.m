clc
clear all


R = 0.316;
Ir = 2;
f_r = 0.0164;
M = 1675;
g = 9.81;
L = 2.675;
L_f = 0.4*L; L_r = L-L_f;
h = 0.543;
c_d = 0.30; A_f = 2.17; air = 1.3;



road_condition = 1;     % Dry Asphalt
slope = 5*pi/180;


% Defining Intital Conditions for Time t=0
t0 = 0;
t_end = 20;
delta_t = 0.005;
t = t0:delta_t:t_end;

s = 0;
v = 0.1;
wf = (1.05*v)/R;
wr = v/R; 

s_vector = zeros(size(t));
v_vector = zeros(size(t));
v_dot_vector = zeros(size(t));
wf_vector = zeros(size(t));
wr_vector = zeros(size(t));
F_fz_vector = zeros(size(t));
F_rz_vector = zeros(size(t));
slip_f_vector = zeros(size(t));
slip_r_vector = zeros(size(t));
W_dot_f_vector = zeros(size(t));
W_dot_r_vector = zeros(size(t));
% Tdrivf_vector = zeros(size(t));
% Tdrivr_vector = zeros(size(t));

Tdrivf_max = 3;
Tdrivr_max = 4;


mue_step = 0.001;
[mu_vec,surface] = TyreSlipModel(0:mue_step:1,road_condition);
[mutilmax, idMuMax] = max(mu_vec);
slipopt = idMuMax*mue_step;

for i = 1:length(t)
    slipf = Slip(v,wf,R);
    slipr = Slip(v,wr,R);
    
    if(slipf > slipopt)
        slipf = slipopt;
    end
    
    if(slipr > slipopt)
        slipr = slipopt;
    end
    
%     if(Tdrivf > Tdrivf_max)
%         Tdrivf = Tdrivf_max;
%     end
%     
%     if(Tdrivr > Tdrivr_max)
%         Tdrivr = Tdrivr_max;
%     end
    
    Fair = 0.5*c_d*A_f*air*v^2;
    
    muf = TyreSlipModel(slipf,road_condition);
    mur = TyreSlipModel(slipr,road_condition);
    
    F_fz = -(L_f*M*g*h*mur*cos(slope) - L*L_r*M*g*cos(slope) + L_r*M*g*h*mur*cos(slope))/(L*(L + h*muf - h*mur));    %[N] front normal force
    F_rz = (L*L_f*M*g*cos(slope) + L_f*M*g*h*muf*cos(slope) + L_r*M*g*h*muf*cos(slope))/(L*(L + h*muf - h*mur));    %[N] rear normal force
    W_dot_f = (L^2*Tdrivf_max + L*Tdrivf_max*h*muf - L*Tdrivf_max*h*mur - L*L_r*M*R*f_r*g*cos(slope) - L*L_r*M*R*g*muf*cos(slope) + L_f*M*R*f_r*g*h*mur*cos(slope) + L_r*M*R*f_r*g*h*mur*cos(slope) + L_f*M*R*g*h*muf*mur*cos(slope) + L_r*M*R*g*h*muf*mur*cos(slope))/(Ir*L*(L + h*muf - h*mur)); 
    W_dot_r = -(L*Tdrivr_max*h*mur - L*Tdrivr_max*h*muf - L^2*Tdrivr_max + L*L_f*M*R*f_r*g*cos(slope) + L*L_f*M*R*g*mur*cos(slope) + L_f*M*R*f_r*g*h*muf*cos(slope) + L_r*M*R*f_r*g*h*muf*cos(slope) + L_f*M*R*g*h*muf*mur*cos(slope) + L_r*M*R*g*h*muf*mur*cos(slope))/(Ir*L*(L + h*muf - h*mur));
    v_dot = -(Fair*L + Fair*h*muf - Fair*h*mur + L*M*g*sin(slope) - L_f*M*g*mur*cos(slope) - L_r*M*g*muf*cos(slope) + M*g*h*muf*sin(slope) - M*g*h*mur*sin(slope))/(M*(L + h*muf - h*mur));
    
    s = s + v*delta_t;
    v = v + v_dot*delta_t;
    wf = wf + W_dot_f*delta_t;
    wr = wr + W_dot_r*delta_t;
    
    s_vector(i) = s;
    v_vector(i) = v;
    v_dot_vector(i) = v_dot;
    wf_vector(i) = wf;
    wr_vector(i) = wr;
    F_fz_vector(i) = F_fz;
    F_rz_vector(i) = F_rz;
    slip_f_vector(i) = slipf;
    slip_r_vector(i) = slipr;
    W_dot_f_vector(i) = W_dot_f;
    W_dot_r_vector(i) = W_dot_r;
%     Tdrivf_vector(i) = Tdrivf;
%     Tdrivr_vector(i) = Tdrivr;
    
end

f1 = figure;
plot(t,s_vector*10)
set(gca,'fontweight','bold');
xlabel('Time [sec]','fontsize',12);
ylabel('Distance Travelled [m]','fontsize',12);
title('Distance Travelled vs Time','fontsize',14);
legend('Dry Asphalt')

f2 = figure;
plot(t,F_fz_vector/1000,t,F_rz_vector/1000)
set(gca,'fontweight','bold');
xlabel('Time [sec]','fontsize',12);
ylabel('F_fz,F_rz [kN]','fontsize',12);
title('Vertical Forces vs Time','fontsize',14);
legend('Front','Rear')

f3 = figure;
plot(t,slip_f_vector,t,slip_r_vector)
set(gca,'fontweight','bold');
xlabel('Time [sec]','fontsize',12);
ylabel('Slip_f,Slip_r','fontsize',12);
title('Slip (Front,Rear) vs Time','fontsize',14);
legend('Slip Front','Slip Rear')
