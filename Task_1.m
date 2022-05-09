% Task 1a

clc
clear all

Sx = 0:0.05:1;
K = 3*pi/180;
magic_tire = @(C,D,E) (D*sin(C*atan((atan(K)/(C*D))*Sx*100-E*((atan(K)/(C*D))*Sx*100-atan((atan(K)/(C*D))*Sx*100)))));


M1 = magic_tire(1.45,1,-4);                 % Dry Asphalt
M2 = magic_tire(1.35,0.6,-0.20);            % Wet Asphalt
M3 = magic_tire(1.5,0.1,0.8);               % Ice

f1 = figure;
plot(Sx,M1)
hold on;
plot(Sx,M2)
hold on;
plot(Sx,M3)

set(gca,'fontweight','bold');
xlabel('Longitudinal Slip','fontsize',12);
ylabel('Tractive Force','fontsize',12);
title('Tractive Force vs Longitudinal Tyre Slip','fontsize',14);
legend('Dry Asphalt','Wet Asphalt','Ice')

%% Longitudinal Tyre Slip refers to the relative measure of the speed of the 
%% wheel when it is rotating after it comes in contact witht the road surface
%% and the theroretical speed of the vehicle i.e. when it is not not loaded 
%% or in contact witht the road surface. Basically, it is a measure of the energy loss.


% Task 1b

K = 3*pi/180;
Sx_exp = 0:0.05:0.95;
Mu_exp = [0.0,0.25,0.53,0.77,0.89,0.95,0.94,0.92,0.90,0.86,0.85,0.83,0.81,0.80,0.79,0.78,0.77,0.76,0.75,0.74];

f2 = figure;
plot(Sx_exp,Mu_exp,'r-','LineWidth',1.75)           % For the given experimental set of data
hold on;


Sx_n = 0:0.05:0.95;
E = -0.2:-0.2:-4;
D = 0.05:0.05:1;                                                         
C = 0.55:0.05:1.5;

for i = 1:length(E)
    for j = 1:length(D)
        for l = 1:length(C)
            mu_n(:,j,i) = D(j)*sin(C(l)*atan((atan(K)/(C(l)*D(j)))*Sx_n*100-E(i)*((atan(K)/(C(l)*D(j)))*Sx_n*100-atan((atan(K)/(C(l)*D(j)))*Sx_n*100))));
        end
    end
end

M4 = magic_tire(1.5,0.95,0);                        % At E=0, keeping the values of C&D constant for the best fit 
plot(Sx,M4,'LineWidth',1.25)
hold on;

plot(Sx_n,mu_n(:,end-1,end),'*-','LineWidth',1.5,'MarkerSize',7,'Color',[0 0 0.7])      % At E=-4, keeping the values of C&D constant for the best fit
hold on;
plot(Sx_n,mu_n(:,end-1,15))                         % At E=-3, keeping the values of C&D constant for the best fit
plot(Sx_n,mu_n(:,end-1,10))                         % At E=-2, keeping the values of C&D constant for the best fit
plot(Sx_n,mu_n(:,end-1,5))                          % At E=-1, keeping the values of C&D constant for the best fit

set(gca,'fontweight','bold');
xlabel('Longitudinal Slip','fontsize',12);
ylabel('Tractive Force','fontsize',12);
title('Tractive Force vs Longitudinal Tyre Slip','fontsize',14);
legend('Experimental Data','E = 0','Best Fit (at E = -4)','E = -3','E = -2','E = -1');

