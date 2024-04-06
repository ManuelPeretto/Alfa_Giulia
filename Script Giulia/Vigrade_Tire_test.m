clear
close all
clc

data=readtable("C:\Users\manue\Desktop\Tire_test1.csv");

currentFile = mfilename('fullpath');
[pathstr, ~, ~] = fileparts(currentFile);
addpath(genpath(pathstr));


Tyre.Params_f = mfeval.readTIR(strcat(pathstr,"\file tir\FRONT_V1Pirelli_Cinturato_AR_Giulia_2.2_JTD_150_AT8_DC_LMUX_OK_LMUY_V1.tir"));
Tyre.Params_r = mfeval.readTIR(strcat(pathstr,"\file tir\REAR_V1Pirelli_Cinturato_AR_Giulia_2.2_JTD_150_AT8_DC_LMUX_OK_LMUY_V1.tir"));

time = data.time_TIME(1:670);
n = numel(time);

alfa = data.wheel_angles_toe(1:670);
%alfa = data.til_wheel_tire_kinematics_lateral_slip;

Fz = data.til_wheel_tire_forces_normal(1:670);
%Fz = zeros(n,1)+45000;

Fy_data = data.til_wheel_tire_forces_lateral(1:670);

% scatter(time,Fz,'filled');
% plot(time,Fz);



kappa = zeros(n,1);
gamma = zeros(n,1);
phit = zeros (n,1);
Vx = ones (n,1)*90/3.6;

outMF = mfeval(Tyre.Params_f , [Fz kappa alfa gamma phit Vx] , 111);

Fy_mfeval = -outMF(:,2);


figure
hold on
set(gca,'TickLabelInterpreter','latex');
subtitle('Front left',Interpreter='latex',fontsize=14);
scatter3(-alfa,Fz,-Fy_mfeval,10,'filled');
scatter3(-alfa,Fz,Fy_data);
xlabel('$\alpha [rad]$',Interpreter='latex',fontsize=16);
ylabel('$F_z [N]$',Interpreter='latex',fontsize=14);
zlabel('$F_y [N]$',Interpreter='latex',fontsize=14);
legend('$F_y (MFeval)$','$F_y (Data)$',Interpreter='latex',fontsize=16);




