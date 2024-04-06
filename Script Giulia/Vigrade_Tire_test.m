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

%outMF = mfeval(Tyre.Params_f , [Fz kappa alfa gamma phit Vx] , 111);
%
%Fy_mfeval = -outMF(:,2);

x0=[1,1,1,1,1];
lb=[0.8,0.5,-5,0.7,0.7];
ub=[1.2,1.3,+5,1.3,1.3];

%%
input_struct = struct('Fz',Fz,'Kappa',kappa,'alfa',alfa,'camber',gamma,'phit',phit,'Vx',Vx);

%%
Tyre_params=Tyre.Params_f;

%%
outMF= mfeval(Tyre_params, [input_struct.Fz input_struct.Kappa input_struct.alfa input_struct.camber input_struct.phit input_struct.Vx] , 111);
figure
hold on
set(gca,'TickLabelInterpreter','latex');
scatter3(input_struct.alfa,input_struct.Fz,Fy_data,30,'filled');
scatter3(input_struct.alfa,input_struct.Fz,outMF(:,2),30,'filled');

param=fmincon( @(parsToOpti) errorCalc(Tyre_params,input_struct,parsToOpti,Fy_data) ,x0,[],[],[],[],lb,ub);

Tyre_params.LMUY=Tyre_params.LMUY*param(1);
Tyre_params.LKY=Tyre_params.LKY*param(2);
Tyre_params.LEY=Tyre_params.LEY*param(3);
Tyre_params.LCY=Tyre_params.LCY*param(4);
Tyre_params.LKYC=Tyre_params.LKYC*param(5);
outMF= mfeval(Tyre_params, [input_struct.Fz input_struct.Kappa input_struct.alfa input_struct.camber input_struct.phit input_struct.Vx] , 111);

scatter3(input_struct.alfa,input_struct.Fz,outMF(:,2),30,'filled');
xlabel('$\alpha [rad]$',Interpreter='latex',fontsize=16);
ylabel('$F_z [N]$',Interpreter='latex',fontsize=14);
zlabel('$F_y [N]$',Interpreter='latex',fontsize=14);
legend('$F_y (Data)$','$F_y (MFeval)$','$F_y (Fitted)$',Interpreter='latex',fontsize=16);

function yError = errorCalc(Tyre_params,input_struct,parsToOpti,Fy_data)
     Tyre_params.LMUY=Tyre_params.LMUY*parsToOpti(1);
     Tyre_params.LKY=Tyre_params.LKY*parsToOpti(2);
     Tyre_params.LEY=Tyre_params.LEY*parsToOpti(3);
     Tyre_params.LCY=Tyre_params.LCY*parsToOpti(4);
     Tyre_params.LKYC=Tyre_params.LKYC*parsToOpti(5);
     outMF= mfeval(Tyre_params, [input_struct.Fz input_struct.Kappa input_struct.alfa input_struct.camber input_struct.phit input_struct.Vx] , 111);
     yError = sum((abs(outMF(:,2))-(Fy_data)).^2);
end



% figure
% hold on
% set(gca,'TickLabelInterpreter','latex');
% subtitle('Front left',Interpreter='latex',fontsize=14);
% scatter3(-alfa,Fz,-Fy_mfeval,10,'filled');
% scatter3(-alfa,Fz,Fy_data);
% xlabel('$\alpha [rad]$',Interpreter='latex',fontsize=16);
% ylabel('$F_z [N]$',Interpreter='latex',fontsize=14);
% zlabel('$F_y [N]$',Interpreter='latex',fontsize=14);
% legend('$F_y (MFeval)$','$F_y (Data)$',Interpreter='latex',fontsize=16);




