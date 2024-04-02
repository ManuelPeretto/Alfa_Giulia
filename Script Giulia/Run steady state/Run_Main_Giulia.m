clear
close all
clc

currentFile = mfilename('fullpath');
[pathstr, ~, ~] = fileparts(currentFile);
path_giulia = fileparts(pathstr);
addpath(genpath(path_giulia));

%% Parameters Input
Vehicle.m = 1750;                  % total mass in Kg
Vehicle.J = 2129;                  % rotational inertia of yaw motion
Vehicle.L = 2.82;                  % Wheelbase
Vehicle.wd = 0.533;                 % Weight distribution
Vehicle.b = Vehicle.wd*Vehicle.L;  % distance from gravity center to rear axle
Vehicle.a = Vehicle.L-Vehicle.b;   % distance from gravity center to front axle
Vehicle.tau = 11.8;

Vehicle.Wf = 1.557;                % Wheel track front 
Vehicle.Wr = 1.625;                % Wheel track rear
Vehicle.h = 0.592;                 % Gravity center height
Vehicle.dr = 0.086;                % height of rear roll center
Vehicle.df = 0.041;                % height of front roll center
Vehicle.dd = Vehicle.df+(Vehicle.dr-Vehicle.df)*Vehicle.a/Vehicle.L;

Vehicle.ks_f = 20.8e3;                             % N/m % front suspension stiffness
Vehicle.k_antiroll_f = 4.168e4;                   % front rollbar stiffness
Vehicle.k_roll_f = Vehicle.Wf^2 / 2 * (Vehicle.ks_f + 2*Vehicle.k_antiroll_f); % front roll stiffness

Vehicle.ks_r = 28.8e3;                             % N/m % rear suspension stiffness
Vehicle.k_antiroll_r = 1.1709e+04;                % rear rollbar stiffness
Vehicle.k_roll_r = Vehicle.Wr^2 / 2 * (Vehicle.ks_r + 2 * Vehicle.k_antiroll_r); % rear roll stiffness

Vehicle.Kf_K = Vehicle.k_roll_f / (Vehicle.k_roll_f + Vehicle.k_roll_r);   % (Kf / Ktotale) Roll stiffness distribution to front axle

Vehicle.toe_f = -0.157;               % [deg] Negativo = toe-out
Vehicle.toe_r = 0.296;                % [deg] Positivo = toe-in
%Vehicle.toe_f = 0;                   % [deg] Negativo = toe-out
%Vehicle.toe_r = 0;                   % [deg] Positivo = toe-in 
Vehicle.percentuale_Ack = 0;          % Ackermann 0 e 1


%%
Vehicle.Fzf = Vehicle.m * 9.81 * Vehicle.b / Vehicle.L;       % Radial Force on front axle
Vehicle.Fzr = Vehicle.m * 9.81 * Vehicle.a / Vehicle.L;       % Radial Force on rear axle

% load transfer distribution
Vehicle.d = (Vehicle.Kf_K * ( Vehicle.h - Vehicle.dd ) / Vehicle.h) + (Vehicle.b / Vehicle.h * Vehicle.df / Vehicle.L);

%% select file tir

Tyre.Params_f = mfeval.readTIR("C:\Users\manue\Documents\GitHub\Alfa_Giulia\Script Giulia\file tir\FRONT_V2Pirelli_Cinturato_AR_Giulia_2.2_JTD_150_AT8_DC_LMUX_OK_LMUY_V2.tir");
Tyre.Params_r = mfeval.readTIR("C:\Users\manue\Documents\GitHub\Alfa_Giulia\Script Giulia\file tir\REAR_V2Pirelli_Cinturato_AR_Giulia_2.2_JTD_150_AT8_DC_LMUX_OK_LMUY_V2.tir");


%% Set simulation Time
dt=0.01;    % time step [s]
tMax=60;   % final time [s]

tvec=[0:dt:tMax]; % generate time vector
N=length(tvec);

%%
data=readtable("data_Manuel_long_13_03_2024.csv");
time = data.time_TIME;

%Vehicle.camber_fl = interp1(time,data.wheel_angles_camber_R1,tvec');
%Vehicle.camber_fr = interp1(time,data.wheel_angles_camber_L1,tvec');
%Vehicle.camber_rl = interp1(time,data.wheel_angles_camber_R2,tvec');
%Vehicle.camber_rr = interp1(time,data.wheel_angles_camber_L2,tvec');

Vehicle.camber_fl = zeros(N,1);
Vehicle.camber_fr = zeros(N,1);
Vehicle.camber_rl = zeros(N,1);
Vehicle.camber_rr = zeros(N,1);

% p_fl = polyfit(data.Tire_Ground_Surface_Force_Z_L1 , data.wheel_angles_camber_L1 , 1);
% p_fr = polyfit(data.Tire_Ground_Surface_Force_Z_R1 , data.wheel_angles_camber_R1 , 1);
% p_rl = polyfit(data.Tire_Ground_Surface_Force_Z_L2 , data.wheel_angles_camber_L2 , 1);
% p_rr = polyfit(data.Tire_Ground_Surface_Force_Z_R2 , data.wheel_angles_camber_R2 , 1);
% 
% Vehicle.camber_fl_test = polyval(p_fl , data.Tire_Ground_Surface_Force_Z_L1);
% Vehicle.camber_fr_test = polyval(p_fr , data.Tire_Ground_Surface_Force_Z_R1);
% Vehicle.camber_rl_test = polyval(p_rl , data.Tire_Ground_Surface_Force_Z_L2);
% Vehicle.camber_rr_test = polyval(p_rr , data.Tire_Ground_Surface_Force_Z_R2);
% 
% color_list=colormap(lines(4));
% figure()
% hold on
% set(gca,'TickLabelInterpreter','latex');
% plot(time,Vehicle.camber_fl,linewidth=1.5,color=color_list(1,:));
% plot(time,Vehicle.camber_fl_test,'--',color=color_list(1,:));
% plot(time,Vehicle.camber_fr,linewidth=1.5,color=color_list(2,:));
% plot(time,Vehicle.camber_fr_test,'--',color=color_list(2,:));
% plot(time,Vehicle.camber_rl,linewidth=1.5,color=color_list(3,:));
% plot(time,Vehicle.camber_rl_test,'--',color=color_list(3,:));
% plot(time,Vehicle.camber_rr,linewidth=1.5,color=color_list(4,:));
% plot(time,Vehicle.camber_rr_test,'--',color=color_list(4,:));
% ylabel('camber angle [rad]',Interpreter='latex',fontsize=14);
% xlabel('time [sec]',Interpreter='latex',fontsize=14);
% legend('front left','','front right','','rear left','','rear right','',Interpreter='latex',fontsize=14);
% 
% 
% figure()
% hold on
% set(gca,'TickLabelInterpreter','latex');
% plot(tvec,Vehicle.camber_fl,linewidth=1.5);
% plot(tvec,Vehicle.camber_fr,linewidth=1.5);
% plot(tvec,Vehicle.camber_rl,linewidth=1.5);
% plot(tvec,Vehicle.camber_rr,linewidth=1.5);
% ylabel('camber angle [rad]',Interpreter='latex',fontsize=14);
% xlabel('time [sec]',Interpreter='latex',fontsize=14);
% legend('front left','front right','rear left','rear right',Interpreter='latex',fontsize=14);
% 
% figure(10)
% hold on
% set(gca,'TickLabelInterpreter','latex');
% plot(tvec,Vehicle.Kappa_fl,linewidth=1.5);
% plot(tvec,Vehicle.Kappa_fr,linewidth=1.5);
% plot(tvec,Vehicle.Kappa_rl,linewidth=1.5);
% plot(tvec,Vehicle.Kappa_rr,linewidth=1.5);
% legend('front left','front right','rear left','rear right',Interpreter='latex',fontsize=14);

%% Rampsteer simulation
     
deltaf_vec=linspace(0,15,N); % vector steering angle fixed

%V_vec=[30:30:120];
V_vec=[60];

%%
choice_model = 4;
Vehicle.choice_approx = 1;

%% Calc undeersteer gradient

[Tyre] = F_Calcola_CS(V_vec(1),Vehicle,Tyre,choice_model);

Tyre.CSnormalizzata_front = Tyre.CSf / Vehicle.Fzf;
Tyre.CSnormalizzata_rear = Tyre.CSr / Vehicle.Fzr;


Vehicle.Gradiente = ( Vehicle.b * Tyre.CSr - Vehicle.a * Tyre.CSf ) / ( Tyre.CSr * Tyre.CSf * Vehicle.L ) * Vehicle.m * 9.81;
Vehicle.Grad_sterzo = 1/Tyre.CSnormalizzata_front - 1/Tyre.CSnormalizzata_rear;


%%   

leg_V=string(numel(V_vec)*2);
Grad = zeros(numel(V_vec),N-1);
colorlist = colormap(lines(numel(V_vec)));
jj=0;

for ii=1:numel(V_vec)

    V = zeros(1,N)+V_vec(ii)/3.6;

    %% Vehicle
 
    [Solution] = F_Doubletrack_ss(V,deltaf_vec,Vehicle,Tyre,N,choice_model);
          
    %%     
    % Plot understeer gradient
    ay = Solution.r .* Solution.u;

    % Formula (2) Canton pag.5.
    sterzo_din = deg2rad(deltaf_vec) - ((Vehicle.L.*ay)./(Solution.u.^2));
    
    Grad(ii,:) = (diff(rad2deg(sterzo_din))./diff(ay./9.81));
    
    figure(1)
    hold on
    plot(ay/9.81,rad2deg(sterzo_din),'color',colorlist(ii,:),LineWidth=1.5);
    
    %jj=jj+1;
    %leg_V(jj) = strcat('$\delta_D(u =',num2str(V_vec(ii)),' [km/h])$');

    %scatter(ay(1,2)./9.81,Grad(ii,1),'MarkerEdgeColor',colorlist(ii,:),'MarkerFaceColor',colorlist(ii,:),LineWidth=2);
    %jj=jj+1;
    %leg_V(jj) = strcat('$\frac{d(\delta_D)}{d(ay/g)} = ' , num2str(Grad(ii,1)),' [deg/g]$');
end    

%% Graph
figure(1)
hold on
grid on
set(gca,'TickLabelInterpreter','latex');
ylabel('$\delta_d$ [deg]',Interpreter='latex',fontsize=14);
xlabel('$\frac{a_y}{g}$',Interpreter='latex',fontsize=16);
%xlim([0 Tyre.mu+0.1]);
xlim([0 1.2]);
ylim([0 5]);
yline(0,'--');

title('Dinamic steering angle',Interpreter='latex',fontsize=16,LineWidth=5);
%txt = [' $\zeta = ',num2str(rad2deg(Vehicle.Gradiente)),' [deg/g]$'];
%subtitle(txt,Interpreter='latex',fontsize=12);
%xline(Tyre.mu,'--',Color='red');
%mu_txt = [' $\mu$ = ' , num2str(Tyre.mu)];

%leg_V(end+1) = '';
%leg_V(end+1) = mu_txt;
legend(leg_V,Interpreter='latex',fontsize=12);

% figure
% hold on
% set(gca,'TickLabelInterpreter','latex');
% scatter(tvec,Force.Fzfl,'filled');
% scatter(tvec,Force.Fzfr,'filled');
% scatter(tvec,Force.Fzrl,'filled');
% scatter(tvec,Force.Fzrr,'filled');
% xlabel('$Time [sec]$',Interpreter='latex',fontsize=16);
% ylabel('$F_Z [N]$',Interpreter='latex',fontsize=14);
% legend('$F_{Zfl}$','$F_{Zfr}$','$F_{Zrl}$','$F_{Zrr}$',Interpreter='latex',fontsize=16);


% figure
% hold on
% set(gca,'TickLabelInterpreter','latex');
% scatter(Alfa.alfafl,Force.Fyfl,'filled');
% scatter(Alfa.alfafr,Force.Fyfr,'filled');
% scatter(Alfa.alfarl,Force.Fyrl,'filled');
% scatter(Alfa.alfarr,Force.Fyrr,'filled');
% legend('$F_{Yfl}$','$F_{Yfr}$','$F_{Yrl}$','$F_{Yrr}$',Interpreter='latex',fontsize=16);

