clear
close all
clc

currentFile = mfilename('fullpath');
[pathstr, ~, ~] = fileparts(currentFile);
path_giulia = fileparts(pathstr);
addpath(genpath(path_giulia));

%% Parameters Input
Vehicle.m = 1750;                                   % total mass in Kg
Vehicle.J = 2129;                                   % rotational inertia of yaw motion
Vehicle.L = 2.82;                                   % Wheelbase
Vehicle.wd = 0.533;                                 % Weight distribution
Vehicle.b = Vehicle.wd*Vehicle.L;                   % distance from gravity center to rear axle
Vehicle.a = Vehicle.L-Vehicle.b;                    % distance from gravity center to front axle
Vehicle.tau = 11.8;

Vehicle.Wf = 1.557;                                 % Wheel track front 
Vehicle.Wr = 1.625;                                 % Wheel track rear
Vehicle.h = 0.592;                                  % Gravity center height
Vehicle.dr = 0.086;                                 % height of rear roll center
Vehicle.df = 0.041;                                 % height of front roll center
Vehicle.dd = Vehicle.df+(Vehicle.dr-Vehicle.df)*Vehicle.a/Vehicle.L;

Vehicle.ks_f = 20.8e3;                             % N/m % front suspension stiffness
Vehicle.k_antiroll_f = 4.168e4;                   % front rollbar stiffness
Vehicle.k_roll_f = Vehicle.Wf^2 / 2 * (Vehicle.ks_f + 2*Vehicle.k_antiroll_f); % front roll stiffness

Vehicle.ks_r = 28.8e3;                             % N/m % rear suspension stiffness
Vehicle.k_antiroll_r = 1.1709e+04;                % rear rollbar stiffness
Vehicle.k_roll_r = Vehicle.Wr^2 / 2 * (Vehicle.ks_r + 2 * Vehicle.k_antiroll_r); % rear roll stiffness

Vehicle.Kf_K = Vehicle.k_roll_f / (Vehicle.k_roll_f + Vehicle.k_roll_r);   % (Kf / Ktotale) Roll stiffness distribution to front axle

Vehicle.toe_f = -0.157;               % [deg]
Vehicle.toe_r = 0.296;
%Vehicle.toe_f = 0;               % [deg]
%Vehicle.toe_r = 0;
Vehicle.percentuale_Ack = 0;   % Ackermann 0 e 1


%%
Vehicle.Fzf = Vehicle.m * 9.81 * Vehicle.b / Vehicle.L;       % Radial Force on front axle
Vehicle.Fzr = Vehicle.m * 9.81 * Vehicle.a / Vehicle.L;       % Radial Force on rear axle

% load transfer distribution
Vehicle.d = (Vehicle.Kf_K * ( Vehicle.h - Vehicle.dd ) / Vehicle.h) + (Vehicle.b / Vehicle.h * Vehicle.df / Vehicle.L);

%% select file tir

Tyre.Params_f = mfeval.readTIR(strcat(path_giulia,"\file tir\FRONT_V2Pirelli_Cinturato_AR_Giulia_2.2_JTD_150_AT8_DC_LMUX_OK_LMUY_V2.tir"));
Tyre.Params_r = mfeval.readTIR(strcat(path_giulia,"\file tir\REAR_V2Pirelli_Cinturato_AR_Giulia_2.2_JTD_150_AT8_DC_LMUX_OK_LMUY_V2.tir"));

%% Rampsteer simulation

V_vec=[60];

%%
choice_model = menu("Choose a Vehicle model","Single track linear","Single track NON linear","Double track linear","Double track NON linear");

%% Calc undeersteer gradient

[Tyre] = F_Calcola_CS(V_vec(1),Vehicle,Tyre,choice_model);

Tyre.CSnormalizzata_front = Tyre.CSf / Vehicle.Fzf;
Tyre.CSnormalizzata_rear = Tyre.CSr / Vehicle.Fzr;


Vehicle.Gradiente = ( Vehicle.b * Tyre.CSr - Vehicle.a * Tyre.CSf ) / ( Tyre.CSr * Tyre.CSf * Vehicle.L ) * Vehicle.m * 9.81;
Vehicle.Grad_sterzo = 1/Tyre.CSnormalizzata_front - 1/Tyre.CSnormalizzata_rear;


%% 
sf=deg2rad(15);
tmax = 60;
omega_steer=sf/tmax;     % [rad/sec]
tspan = [0:omega_steer:tmax];
N=length(tspan);

%%
data=readtable("C:\Users\manue\Desktop\Simulazioni Vi-grade rampsteer\data_Manuel_long_13_03_2024.csv");
time = data.time_TIME;

%Vehicle.camber_fl = interp1(time,data.wheel_angles_camber_L1,tspan');
%Vehicle.camber_fr = interp1(time,data.wheel_angles_camber_R1,tspan');
%Vehicle.camber_rl = interp1(time,data.wheel_angles_camber_L2,tspan');
%Vehicle.camber_rr = interp1(time,data.wheel_angles_camber_R2,tspan');

Vehicle.p_fl = polyfit(data.Tire_Ground_Surface_Force_Z_L1 , data.wheel_angles_camber_L1 , 1);
Vehicle.p_fr = polyfit(data.Tire_Ground_Surface_Force_Z_R1 , data.wheel_angles_camber_R1 , 1);
Vehicle.p_rl = polyfit(data.Tire_Ground_Surface_Force_Z_L2 , data.wheel_angles_camber_L2 , 1);
Vehicle.p_rr = polyfit(data.Tire_Ground_Surface_Force_Z_R2 , data.wheel_angles_camber_R2 , 1);
%%

options= odeset('RelTol',1e-6,'AbsTol',1e-3,'InitialStep',1e-3,'Refine',2,'Stats','on');

leg_V=string(numel(V_vec)*2);
Grad = zeros(numel(V_vec),N-1);
colorlist = colormap(lines(numel(V_vec)));
jj=0;

for ii=1:numel(V_vec)
   
    V = V_vec(ii)/3.6;

    y0 = [0  0];
    
    switch choice_model
        case {1 2}
         [Solution] = ode_singletrack_rampsteer_giulia(V,omega_steer,Vehicle,Tyre,tspan, y0,options,choice_model);
        case {3 4}
         [Solution] = ode_doubletrack_rampsteer_giulia(V,Vehicle,Tyre,omega_steer,choice_model,tspan,y0,options);
    end     
  
    %%
    
    deltaf_vec = Solution.delta;

    % Plot understeer gradient
    ay = Solution.r .* Solution.u;

    % Formula (2) Canton pag.5.
    sterzo_din = deltaf_vec - ((Vehicle.L.*ay)./(Solution.u.^2));
    
    Grad(ii,:) = (diff(rad2deg(sterzo_din))./diff(ay./9.81));

    figure(1)
    hold on
    plot(ay/9.81,rad2deg(sterzo_din),'color',colorlist(ii,:),LineWidth=1.5);
    
    jj=jj+1;
    leg_V(jj) = strcat('$ODE (',num2str(tmax),' [s])$');
 end    

%% Graph
figure(1)
grid on
set(gca,'TickLabelInterpreter','latex');
ylabel('$\delta_d$ [deg]',Interpreter='latex',fontsize=14);
xlabel('$\frac{a_y}{g}$',Interpreter='latex',fontsize=16);
xlim([0 Tyre.mu+0.1]);
ylim([0 8]);
yline(0,'--');

title('Dinamic steering angle',Interpreter='latex',fontsize=16,LineWidth=5);
legend(leg_V,Interpreter='latex',fontsize=12);

