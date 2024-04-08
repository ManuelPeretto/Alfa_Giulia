clear
close all
clc
%data=readtable("data_Manuel_19_02_2024.csv");
%data=readtable("data_Manuel_11_03_24.csv");
data=readtable("data_Manuel_long_13_03_2024.csv");

currentFile = mfilename('fullpath');
[pathstr, ~, ~] = fileparts(currentFile);
addpath(genpath(pathstr));


Vehicle.tau = 12.6;
Vehicle.L = 2.82;

Tyre.Params_f = mfeval.readTIR("C:\Users\manue\Desktop\Script Giulia\FRONT_V2Pirelli_Cinturato_AR_Giulia_2.2_JTD_150_AT8_DC_LMUX_OK_LMUY_V2.tir");
Tyre.Params_r = mfeval.readTIR("C:\Users\manue\Desktop\Script Giulia\REAR_V2Pirelli_Cinturato_AR_Giulia_2.2_JTD_150_AT8_DC_LMUX_OK_LMUY_V2.tir");

time = data.time_TIME;

n=numel(time);
speed= 60;

deltaf_vec = data.driver_demands_driver_steering_angle ./ Vehicle.tau; % [rad]
delta_fl = -data.wheel_angles_toe_L1;
delta_fr = data.wheel_angles_toe_R1;
delta_f = (delta_fl + delta_fr) ./2;

u = data.chassis_velocities_longitudinal./3.6;
ay = -data.chassis_accelerations_lateral.*9.81;

%sterzo_din = (deltaf_vec) - ((Vehicle.L.*ay)./(u.^2));
sterzo_din = (delta_f) - ((Vehicle.L.*ay)./(u.^2));
mu = max(ay./9.81);

alfa_fl = data.Tire_Lateral_Slip_Without_Lag_L1;
alfa_fr = data.Tire_Lateral_Slip_Without_Lag_R1;
alfa_rl = data.Tire_Lateral_Slip_Without_Lag_L2;
alfa_rr = data.Tire_Lateral_Slip_Without_Lag_R2;

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


Fy_fl_data = data.Tire_Ground_Surface_Force_Y_L1;
Fy_fr_data = data.Tire_Ground_Surface_Force_Y_R1;
Fy_rl_data = data.Tire_Ground_Surface_Force_Y_L2;
Fy_rr_data = data.Tire_Ground_Surface_Force_Y_R2;

Vx = ones (n,1)*(speed./3.6);

x0=[1,1,1];
lb=[0.8,0.7,-5];
ub=[1.2,1.3,+5];

%%
input_struct = struct('Fz',Fz_fl,'Kappa',Kappa_fl,'alfa',alfa_fl,'camber',-camber_fl,'phit',0*alfa_fl,'Vx',Vx);

%%
Tyre_params=Tyre.Params_f;
Fy_data=Fy_fl_data;
param=fmincon( @(parsToOpti) errorCalc(Tyre_params,input_struct,parsToOpti,Fy_data) ,x0,[],[],[],[],lb,ub);

Tyre_params.LMUY=Tyre_params.LMUY*param(1);
Tyre_params.LKY=Tyre_params.LKY*param(2);
Tyre_params.LEY=Tyre_params.LEY*param(3);
outMF= mfeval(Tyre_params, [input_struct.Fz input_struct.Kappa input_struct.alfa input_struct.camber input_struct.phit input_struct.Vx] , 111);

figure
hold on
scatter3(input_struct.alfa,input_struct.Fz,Fy_data,30,'filled');
scatter3(input_struct.alfa,input_struct.Fz,-outMF(:,2),30,'filled');

function yError = errorCalc(Tyre_params,input_struct,parsToOpti,Fy_data)
     Tyre_params.LMUY=Tyre_params.LMUY*parsToOpti(1);
     Tyre_params.LKY=Tyre_params.LKY*parsToOpti(2);
     Tyre_params.LEY=Tyre_params.LEY*parsToOpti(3);
     outMF= mfeval(Tyre_params, [input_struct.Fz input_struct.Kappa input_struct.alfa input_struct.camber input_struct.phit input_struct.Vx] , 111);
     yError = sum((abs(-outMF(:,2))-(Fy_data)).^2);
end


