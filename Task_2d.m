% Task 2d
clc
clear all

syms F_fx F_rx F_fz F_rz W_dot_f W_dot_r v_dot ...
     M g slope L_f L_r L h R f_r Fair Tdrivf Tdrivr Ir muf mur;

e = f_r * R;

% E0 = 0 == F_rz + F_fz - (m*g*cos(theta));                                                       % Sum of Vertial Forces
E1 = 0 == F_rx + F_fx - Fair - (M*v_dot) - (M*g*sin(slope));                                   % Sum of Longitudinal Forces
E2 = 0 == (F_rz*L) - (M*g*cos(slope)*L_f) - (M*v_dot*h) - (Fair*h) - (M*g*sin(slope)*h);      % Moments about the Front Axle
E3 = 0 == -(F_fz*L) + (M*g*cos(slope)*L_r) - (M*v_dot*h) - (Fair*h) - (M*g*sin(slope)*h);      % Moments about the Rear Axle
E4 = 0 == -Tdrivf + (F_fz*e) + (F_fx*R) + (Ir*W_dot_f);                                             % Moment about the centre of the Front Wheel
E5 = 0 == -Tdrivr + (F_rz*e) + (F_rx*R) + (Ir*W_dot_r);                                             % Moment about the centre of the Rear Wheel
E6 = 0 == F_fx - (muf*F_fz);                                                                   % Normalized Longitudinal Tyre Force about the Front Wheel
E7 = 0 == F_rx - (mur*F_rz);                                                                   % Normalized Longitudinal Tyre Force about the Front Wheel

sol = solve([E1;E2;E3;E4;E5;E6;E7],[F_fx;F_rx;F_fz;F_rz;W_dot_f;W_dot_r;v_dot]);