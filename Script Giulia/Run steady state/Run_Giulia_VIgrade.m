clear
close all
clc

currentFile = mfilename('fullpath');
[pathstr, ~, ~] = fileparts(currentFile);
path_giulia = fileparts(pathstr);
addpath(genpath(path_giulia));

data=readtable("C:\Users\manue\Desktop\data_ramp_steer.csv");

%% Parameters Input
Vehicle.m = 1449;                                   % total mass in Kg
Vehicle.J = 2129;                                   % rotational inertia of yaw motion
Vehicle.L = 2.82;                                   % Wheelbase
Vehicle.wd = 0.533687943;                           % Weight distribution
Vehicle.b = Vehicle.wd*Vehicle.L;                   % distance from gravity center to rear axle
Vehicle.a = Vehicle.L-Vehicle.b;                    % distance from gravity center to front axle
Vehicle.tau = 11.8;

Vehicle.Wf = 1.557;             % Wheel track front 
Vehicle.Wr = 1.625;             % Wheel track rear
Vehicle.h = 0.592;              % Gravity center height
Vehicle.dr = 0.086;             % height of rear roll center
Vehicle.df = 0.041;             % height of front roll center
Vehicle.dd = Vehicle.df+(Vehicle.dr-Vehicle.df)*Vehicle.a/Vehicle.L;

Vehicle.ks_f = 20.8e3;                             % N/m % front suspension stiffness
Vehicle.k_antiroll_f = 4.168e4;                   % front rollbar stiffness
Vehicle.k_roll_f = Vehicle.Wf^2 / 2 * (Vehicle.ks_f + 2*Vehicle.k_antiroll_f); % front roll stiffness

Vehicle.ks_r = 28.8e3;                             % N/m % rear suspension stiffness
Vehicle.k_antiroll_r = 1.1709e+04;                % rear rollbar stiffness
Vehicle.k_roll_r = Vehicle.Wr^2 / 2 * (Vehicle.ks_r + 2 * Vehicle.k_antiroll_r); % rear roll stiffness

Vehicle.Kf_K = Vehicle.k_roll_f / (Vehicle.k_roll_f + Vehicle.k_roll_r);   % (Kf / Ktotale) Roll stiffness distribution to front axle

Vehicle.toe = 0;               % [deg]
Vehicle.percentuale_Ack = 0;

%%
Vehicle.Fzf = Vehicle.m * 9.81 * Vehicle.b / Vehicle.L;       % Radial Force on front axle
Vehicle.Fzr = Vehicle.m * 9.81 * Vehicle.a / Vehicle.L;       % Radial Force on rear axle

% load transfer distribution
Vehicle.d = (Vehicle.Kf_K * ( Vehicle.h - Vehicle.dd ) / Vehicle.h) + (Vehicle.b / Vehicle.h * Vehicle.df / Vehicle.L);

%% select file tir

Tyre.Params_f = mfeval.readTIR("C:\Users\manue\Desktop\Script Giulia\file tir\FRONT_V1Pirelli_Cinturato_AR_Giulia_2.2_JTD_150_AT8_DC_LMUX_OK_LMUY_V1.tir");
Tyre.Params_r = mfeval.readTIR("C:\Users\manue\Desktop\Script Giulia\file tir\REAR_V1Pirelli_Cinturato_AR_Giulia_2.2_JTD_150_AT8_DC_LMUX_OK_LMUY_V1.tir");

choice_model = 2;
%% Simulation Time


tvec = data.time_TIME;
N=length(tvec);

%% Rampsteer simulation
     
deltaf_vec = data.driver_demands_driver_steering_angle ./ Vehicle.tau; % [rad]
deltaf_vec = -rad2deg(deltaf_vec); %[deg]

V_vec=[60];
V = zeros(1,N) + V_vec(1);

%% Calc undeersteer gradient

[Tyre] = F_Calcola_CS(V_vec(1),Vehicle,Tyre,choice_model);

Tyre.CSnormalizzata_front = Tyre.CSf / Vehicle.Fzf;
Tyre.CSnormalizzata_rear = Tyre.CSr / Vehicle.Fzr;


Vehicle.Gradiente = ( Vehicle.b * Tyre.CSr - Vehicle.a * Tyre.CSf ) / ( Tyre.CSr * Tyre.CSf * Vehicle.L ) * Vehicle.m * 9.81;
Vehicle.Grad_sterzo = 1/Tyre.CSnormalizzata_front - 1/Tyre.CSnormalizzata_rear;


%% Vehicle

[Alfa,Force,Solution,delta] = F_Doubletrack_VIgrade(V,deltaf_vec,Vehicle,Tyre,N,data);

%%
% Plot understeer gradient
ay = Solution.r .* Solution.u;


% Formula (2) Canton pag.5.
sterzo_din = deg2rad(deltaf_vec) - ((Vehicle.L.*ay.')./(Solution.u.'.^2));

Grad = (diff(rad2deg(sterzo_din))./diff(ay.'./9.81));

%% Graph
figure(1)
hold on
plot(ay/9.81,rad2deg(sterzo_din));
set(gca,'TickLabelInterpreter','latex');
ylabel('$\delta_d$ [deg]',Interpreter='latex',fontsize=14);
xlabel('$\frac{a_y}{g}$',Interpreter='latex',fontsize=16);
%xlim([0 Tyre.mu+0.1]);
ylim([0 2]);
yline(0,'--');

title('Dinamic steering angle',Interpreter='latex',fontsize=16,LineWidth=5);
xline(Tyre.mu,'--',Color='red');
mu_txt = [' $\mu$ = ' , num2str(Tyre.mu)];

scatter(ay(10)./9.81,mean(Grad(10:100)),'MarkerEdgeColor','blue','MarkerFaceColor','blue');

Grad_txt = ['$\frac{d(\delta_D)}{d(ay/g)}$ = ' , num2str(mean(Grad(10:100))),' [deg/g]'];
legend('$\delta_D=\delta - \frac{L}{V^2}*a_y$','',Grad_txt,Interpreter='latex',fontsize=14);

% 
% figure
% hold on
% scatter(Alfa.alfafl,Force.Fyfl,'filled');
% scatter(Alfa.alfafr,Force.Fyfr,'filled');
% scatter(Alfa.alfarl,Force.Fyrl,'filled');
% scatter(Alfa.alfarr,Force.Fyrr,'filled');
% legend('Fyfl','Fyfr','Fyrl','Fyrr');
