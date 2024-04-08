clear
close all
clc

data=readtable("C:\Users\manue\Desktop\Simulazioni Vi-grade rampsteer\data_Manuel_long_13_03_2024.csv");

currentFile = mfilename('fullpath');
[path_giulia, ~, ~] = fileparts(currentFile);
addpath(genpath(path_giulia));


Tyre.Params_r = mfeval.readTIR(strcat(path_giulia,"\file tir\REAR_V1Pirelli_Cinturato_AR_Giulia_2.2_JTD_150_AT8_DC_LMUX_OK_LMUY_V1.tir"));

filter = 50;
time = data.time_TIME(filter:end);

n=numel(time);
speed= 60;


alfa_rl = data.Tire_Lateral_Slip_Without_Lag_L2(filter:end);
alfa_rr = data.Tire_Lateral_Slip_Without_Lag_R2(filter:end);

Kappa_rl = data.Tire_Longitudinal_Slip_Without_Lag_L2(filter:end);
Kappa_rr = data.Tire_Longitudinal_Slip_Without_Lag_R2(filter:end);

camber_rl = data.wheel_angles_camber_L2(filter:end);
camber_rr = data.wheel_angles_camber_R2(filter:end);

Fz_rl = data.Tire_Ground_Surface_Force_Z_L2(filter:end);
Fz_rr = data.Tire_Ground_Surface_Force_Z_R2(filter:end);

Fy_rl_data = data.Tire_Ground_Surface_Force_Y_L2(filter:end);
Fy_rr_data = data.Tire_Ground_Surface_Force_Y_R2(filter:end);

%%
Vx = ones (n,1)*(speed./3.6);

x0=[1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1];
lb=[0.8,0.5,-5,0.7,0.7,0.7,0.7,0.7,0.7,0.7,0.7,0.7,0.7,0.7,0.7,0.7,0.7,0.7,0.7];
ub=[1.2,1.3,+5,1.3,1.3,1.3,1.3,1.3,1.3,1.3,1.3,1.3,1.3,1.3,1.3,1.3,1.3,1.3,1.3];

%%
input_struct = struct('Fz',Fz_rl,'Kappa',Kappa_rl,'alfa',alfa_rl,'camber',-camber_rl,'phit',0*alfa_rl,'Vx',Vx);

%%
Tyre_params=Tyre.Params_r;
Fy_data=Fy_rl_data;

%%
outMF= mfeval(Tyre_params, [input_struct.Fz input_struct.Kappa input_struct.alfa input_struct.camber input_struct.phit input_struct.Vx] , 111);
figure
hold on
set(gca,'TickLabelInterpreter','latex');
scatter3(input_struct.alfa,input_struct.Fz,Fy_data,30,'filled');
scatter3(input_struct.alfa,input_struct.Fz,outMF(:,2),30,'filled');

param=fmincon( @(parsToOpti) errorCalc(Tyre_params,input_struct,parsToOpti,Fy_data) ,x0,[],[],[],[],lb,ub);

Tyre_params.LFZO = Tyre_params.LFZO * param(1);
Tyre_params.LCX  = Tyre_params.LCX  * param(2);
Tyre_params.LMUX = Tyre_params.LMUX * param(3);
Tyre_params.LEX  = Tyre_params.LEX  * param(4);
Tyre_params.LKX  = Tyre_params.LKX  * param(5);
Tyre_params.LHX  = Tyre_params.LHX  * param(6);
Tyre_params.LVX  = Tyre_params.LVX  * param(7);
Tyre_params.LCY  = Tyre_params.LCY  * param(8);
Tyre_params.LMUY = Tyre_params.LMUY * param(9);
Tyre_params.LEY  = Tyre_params.LEY  * param(10);
Tyre_params.LKY  = Tyre_params.LKY  * param(11);
Tyre_params.LKYC = Tyre_params.LKYC * param(12);
Tyre_params.LKZC = Tyre_params.LKZC * param(13);
Tyre_params.LHY  = Tyre_params.LHY  * param(14);
Tyre_params.LVY  = Tyre_params.LVY  * param(15);
Tyre_params.LTR  = Tyre_params.LTR  * param(16);
Tyre_params.LRES = Tyre_params.LRES * param(17);
Tyre_params.LXAL = Tyre_params.LXAL * param(18);
Tyre_params.LYKA = Tyre_params.LYKA * param(19);

outMF= mfeval(Tyre_params, [input_struct.Fz input_struct.Kappa input_struct.alfa input_struct.camber input_struct.phit input_struct.Vx] , 111);

scatter3(input_struct.alfa,input_struct.Fz,outMF(:,2),30,'filled');
xlabel('$\alpha [rad]$',Interpreter='latex',fontsize=16);
ylabel('$F_z [N]$',Interpreter='latex',fontsize=14);
zlabel('$F_y [N]$',Interpreter='latex',fontsize=14);
legend('$F_y (Data)$','$F_y (MFeval)$','$F_y (Fitted)$',Interpreter='latex',fontsize=16);

function yError = errorCalc(Tyre_params,input_struct,parsToOpti,Fy_data)
Tyre_params.LFZO = Tyre_params.LFZO * parsToOpti(1);
Tyre_params.LCX  = Tyre_params.LCX  * parsToOpti(2);
Tyre_params.LMUX = Tyre_params.LMUX * parsToOpti(3);
Tyre_params.LEX  = Tyre_params.LEX  * parsToOpti(4);
Tyre_params.LKX  = Tyre_params.LKX  * parsToOpti(5);
Tyre_params.LHX  = Tyre_params.LHX  * parsToOpti(6);
Tyre_params.LVX  = Tyre_params.LVX  * parsToOpti(7);
Tyre_params.LCY  = Tyre_params.LCY  * parsToOpti(8);
Tyre_params.LMUY = Tyre_params.LMUY * parsToOpti(9);
Tyre_params.LEY  = Tyre_params.LEY  * parsToOpti(10);
Tyre_params.LKY  = Tyre_params.LKY  * parsToOpti(11);
Tyre_params.LKYC = Tyre_params.LKYC * parsToOpti(12);
Tyre_params.LKZC = Tyre_params.LKZC * parsToOpti(13);
Tyre_params.LHY  = Tyre_params.LHY  * parsToOpti(14);
Tyre_params.LVY  = Tyre_params.LVY  * parsToOpti(15);
Tyre_params.LTR  = Tyre_params.LTR  * parsToOpti(16);
Tyre_params.LRES = Tyre_params.LRES * parsToOpti(17);
Tyre_params.LXAL = Tyre_params.LXAL * parsToOpti(18);
Tyre_params.LYKA = Tyre_params.LYKA * parsToOpti(19);

     outMF= mfeval(Tyre_params, [input_struct.Fz input_struct.Kappa input_struct.alfa input_struct.camber input_struct.phit input_struct.Vx] , 111);
     yError = sum((abs(outMF(:,2))-(Fy_data)).^2);
end

% LFZO  
% LCX   
% LMUX  
% LEX   
% LKX   
% LHX   
% LVX   
% LCY   
% LMUY  
% LEY   
% LKY   
% LKYC  
% LKZC  
% LHY   
% LVY   
% LTR   
% LRES  
% LXAL  
% LYKA  
% LVYKA 







