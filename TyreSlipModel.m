% Task 2e
function [fx_fz,surface_p] = TyreSlipModel(Sx,road_cond)
conditions = {'Dry Asphalt','Wet Asphalt','Ice'};
C = {1.45,1.35,1.5};
D = {1.00,0.60,0.10};
E = {-4,-0.20,0.80};

surface_p = struct('Conditions',conditions,'C',C,'D',D,'E',E);

if(road_cond == 1)
    disp(surface_p(1));
    
elseif(road_cond == 2)
    disp(surface_p(2));
    
elseif(road_cond == 3)
    disp(surface_p(3));
    
end

K = 3*pi/180;
for i = 1:length(surface_p)
    surface_p(i).B = (atan(K)/(surface_p(i).C*surface_p(i).D));
    fx_fz = (surface_p(i).D*sin(surface_p(i).C*atan(surface_p(i).B*Sx*100-surface_p(i).E*(surface_p(i).B*Sx*100-atan(surface_p(i).B*Sx*100)))));
end

    
    
% if(road_cond == 1)
%     surface_p = 'Dry Asphalt';
%     C = 1.45;
%     D = 1.00;
%     E = -4.00;
%     
% elseif(road_cond == 2)
%     surface_p = 'Wet Asphalt';
%     C = 1.35;
%     D = 0.60;
%     E = -0.20;
%     
% elseif(road_cond == 3)
%     surface_p = 'Ice';
%     C = 1.5;
%     D = 0.10;
%     E = 0.80;
% end
% 
% K = 3*pi/180;
% 
% fx_fz = (D*sin(C*atan((atan(K)/(C*D))*Sx*100-E*((atan(K)/(C*D))*Sx*100-atan((atan(K)/(C*D))*Sx*100)))));

end

