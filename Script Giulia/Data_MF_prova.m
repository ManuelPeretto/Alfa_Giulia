clear
close all
clc
%data=readtable("C:\Users\manue\Desktop\data_ramp_steer.csv");
data=readtable("data_manuel_11_03_24.csv");
currentFile = mfilename('fullpath');
[pathstr, ~, ~] = fileparts(currentFile);
addpath(genpath(pathstr));


Vehicle.tau = 11.8;
Vehicle.L = 2.82;

Tyre.Params_f = mfeval.readTIR("C:\Users\manue\Desktop\Script Giulia\file tir\FRONT_V1Pirelli_Cinturato_AR_Giulia_2.2_JTD_150_AT8_DC_LMUX_OK_LMUY_V1.tir");
Tyre.Params_r = mfeval.readTIR("C:\Users\manue\Desktop\Script Giulia\file tir\REAR_V1Pirelli_Cinturato_AR_Giulia_2.2_JTD_150_AT8_DC_LMUX_OK_LMUY_V1.tir");

time = data.time_TIME;

deltaf_vec = data.driver_demands_driver_steering_angle ./ Vehicle.tau; % [rad]
u = data.chassis_velocities_longitudinal;
ay = -data.chassis_accelerations_lateral;

sterzo_din = (deltaf_vec) - ((Vehicle.L.*ay)./(u.^2));
mu = max(ay);

alfa_fl = data.Tire_Lateral_Slip_Without_Lag_L1;
alfa_fr = data.Tire_Lateral_Slip_Without_Lag_R1;
alfa_rl = data.Tire_Lateral_Slip_Without_Lag_L2;
alfa_rr = data.Tire_Lateral_Slip_Without_Lag_R2;

% alfa_fl = data.Tire_Lateral_Slip_With_Lag_L1;
% alfa_fr = data.Tire_Lateral_Slip_With_Lag_R1;
% alfa_rl = data.Tire_Lateral_Slip_With_Lag_L2;
% alfa_rr = data.Tire_Lateral_Slip_With_Lag_R2;

Kappa_fl = data.Tire_Longitudinal_Slip_Without_Lag_L1;
Kappa_fr = data.Tire_Longitudinal_Slip_Without_Lag_R1;
Kappa_rl = data.Tire_Longitudinal_Slip_Without_Lag_L2;
Kappa_rr = data.Tire_Longitudinal_Slip_Without_Lag_R2;


camber_fl = data.wheel_angles_camber_L1;
camber_fr = data.wheel_angles_camber_R1;
camber_rl = data.wheel_angles_camber_L2;
camber_rr = data.wheel_angles_camber_R2;

Fz_fl = data.Tire_Ground_Surface_Force_Z_L1;
Fz_fr = data.Tire_Ground_Surface_Force_Z_R1;
Fz_rl = data.Tire_Ground_Surface_Force_Z_L2;
Fz_rr = data.Tire_Ground_Surface_Force_Z_R2;

Fx_fl = data.Tire_Ground_Surface_Force_X_L1;

n = numel(time);
kappa = zeros(n,1);
gamma = zeros(n,1);
phit = zeros (n,1);
Vx = ones (n,1)*60/3.6;
pressure = zeros (n,1) + 220000;

outMF_fl = mfeval(Tyre.Params_f , [Fz_fl Kappa_fl alfa_fl -camber_fl phit Vx pressure] , 111);
outMF_fr = mfeval(Tyre.Params_f , [Fz_fr Kappa_fr alfa_fr +camber_fr phit Vx pressure] , 111);
outMF_rl = mfeval(Tyre.Params_r , [Fz_rl Kappa_rl alfa_rl -camber_rl phit Vx pressure] , 111);
outMF_rr = mfeval(Tyre.Params_r , [Fz_rr Kappa_rr alfa_rr +camber_rr phit Vx pressure] , 111);

Fy_fl = -outMF_fl(:,2);
Fy_fr = -outMF_fr(:,2);
Fy_rl = -outMF_rl(:,2);
Fy_rr = -outMF_rr(:,2);

Fy_fl_data = data.Tire_Ground_Surface_Force_Y_L1;
Fy_fr_data = data.Tire_Ground_Surface_Force_Y_R1;
Fy_rl_data = data.Tire_Ground_Surface_Force_Y_L2;
Fy_rr_data = data.Tire_Ground_Surface_Force_Y_R2;

figure
hold on
set(gca,'TickLabelInterpreter','latex');
subtitle('Front left',Interpreter='latex',fontsize=14);
scatter3(-alfa_fl,Fz_fl,-Fy_fl,10,'filled');
scatter3(-alfa_fl,Fz_fl,Fy_fl_data,10,'filled');
xlabel('$\alpha [rad]$',Interpreter='latex',fontsize=16);
ylabel('$F_z [N]$',Interpreter='latex',fontsize=14);
zlabel('$F_y [N]$',Interpreter='latex',fontsize=14);
legend('$F_y (MFeval)$','$F_y (Data)$',Interpreter='latex',fontsize=16);

figure
hold on
set(gca,'TickLabelInterpreter','latex');
subtitle('Front right',Interpreter='latex',fontsize=14);
scatter3(-alfa_fr,Fz_fr,-Fy_fr,10,'filled');
scatter3(-alfa_fr,Fz_fr,Fy_fr_data,10,'filled');
xlabel('$\alpha [rad]$',Interpreter='latex',fontsize=16);
ylabel('$F_z [N]$',Interpreter='latex',fontsize=14);
zlabel('$F_y [N]$',Interpreter='latex',fontsize=14);
legend('$F_y (MFeval)$','$F_y (Data)$',Interpreter='latex',fontsize=16);

figure
hold on
set(gca,'TickLabelInterpreter','latex');
subtitle('Rear left',Interpreter='latex',fontsize=14);
scatter3(-alfa_rl,Fz_rl,-Fy_rl,10,'filled');
scatter3(-alfa_rl,Fz_rl,Fy_rl_data,10,'filled');
xlabel('$\alpha [rad]$',Interpreter='latex',fontsize=16);
ylabel('$F_z [N]$',Interpreter='latex',fontsize=14);
zlabel('$F_y [N]$',Interpreter='latex',fontsize=14);
legend('$F_y (MFeval)$','$F_y (Data)$',Interpreter='latex',fontsize=16);

figure
hold on
set(gca,'TickLabelInterpreter','latex');
subtitle('Rear right',Interpreter='latex',fontsize=14);
scatter3(-alfa_rr,Fz_rr,-Fy_rr,10,'filled');
scatter3(-alfa_rr,Fz_rr,Fy_rr_data,10,'filled');
xlabel('$\alpha [rad]$',Interpreter='latex',fontsize=16);
ylabel('$F_z [N]$',Interpreter='latex',fontsize=14);
zlabel('$F_y [N]$',Interpreter='latex',fontsize=14);
legend('$F_y (MFeval)$','$F_y (Data)$',Interpreter='latex',fontsize=16);




