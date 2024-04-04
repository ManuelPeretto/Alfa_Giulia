clear
close all
clc

currentFile = mfilename('fullpath');
[pathstr, ~, ~] = fileparts(currentFile);
path_giulia = fileparts(pathstr);
addpath(genpath(path_giulia));


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


Vehicle.percentuale_Ack = 0;
Vehicle.toe_f = 0;     % [deg]
Vehicle.toe_r = 0;     % [deg]

%%
Vehicle.Fzf = Vehicle.m * 9.81 * Vehicle.b / Vehicle.L;       % Radial Force on front axle
Vehicle.Fzr = Vehicle.m * 9.81 * Vehicle.a / Vehicle.L;       % Radial Force on rear axle

% load transfer distribution
%Vehicle.d = (Vehicle.Kf_K * ( Vehicle.h - Vehicle.dd ) / Vehicle.h) + (Vehicle.b / Vehicle.h * Vehicle.df / Vehicle.L);

%% select file tir
%choice_tyres = menu("Choose tyres","Select","Toyo","Pirelli V1");
choice_tyres = 3;
switch choice_tyres
    case 1
        [Tyre.Params_f] = select_file_tir(path_giulia);
        [Tyre.Params_r] = select_file_tir(path_giulia);
    case 2
        Tyre.Params_f = mfeval.readTIR(strcat(path_giulia,"\file tir\Toyo_AlfaGiulia.tir"));
        Tyre.Params_r = mfeval.readTIR(strcat(path_giulia,"\file tir\Toyo_AlfaGiulia.tir"));
    case 3
        Tyre.Params_f = mfeval.readTIR(strcat(path_giulia,"\file tir\FRONT_V1Pirelli_Cinturato_AR_Giulia_2.2_JTD_150_AT8_DC_LMUX_OK_LMUY_V1.tir"));
        Tyre.Params_r = mfeval.readTIR(strcat(path_giulia,"\file tir\REAR_V1Pirelli_Cinturato_AR_Giulia_2.2_JTD_150_AT8_DC_LMUX_OK_LMUY_V1.tir"));
end

choice_model = 4;
Vehicle.choice_approx = 1;
%% Set simulation Time
dt=0.1;    % time step [s]
tMax=30;   % final time [s]

tvec=[0:dt:tMax]; % generate time vector
N=length(tvec);

%% Rampsteer simulation
     
deltaf_vec=linspace(0,15,N); % vector steering angle fixed

V_vec=[90];

%% Calc undeersteer gradient

[Tyre] = F_Calcola_CS(V_vec(1),Vehicle,Tyre,choice_model);

Tyre.CSnormalizzata_front = Tyre.CSf / Vehicle.Fzf;
Tyre.CSnormalizzata_rear = Tyre.CSr / Vehicle.Fzr;


Vehicle.Gradiente = ( Vehicle.b * Tyre.CSr - Vehicle.a * Tyre.CSf ) / ( Tyre.CSr * Tyre.CSf * Vehicle.L ) * Vehicle.m * 9.81;
Vehicle.Grad_sterzo = 1/Tyre.CSnormalizzata_front - 1/Tyre.CSnormalizzata_rear;

%%
Vehicle.camber_fl = zeros(N,1);
Vehicle.camber_fr = zeros(N,1);
Vehicle.camber_rl = zeros(N,1);
Vehicle.camber_rr = zeros(N,1);

%% 

%Kf_K_vec = [0.3:0.1:0.8];
Kf_K_vec = [0.45:0.05:0.75];
n = numel(Kf_K_vec);

leg_V=string(n*2);
Grad = zeros(n,N-1);
colorlist = colormap(lines(n));
jj=0;

V = zeros(1,N)+V_vec(1)/3.6;
Piccoay = zeros (n,1);
IndicePicco_ay = zeros (n,1);

txt = [' $Rampsteer','',num2str(V_vec(1)),' [km/h]$'];

for ii=1:n

     Vehicle.Kf_K = Kf_K_vec(ii);
     Vehicle.d = (Vehicle.Kf_K * ( Vehicle.h - Vehicle.dd ) / Vehicle.h) + (Vehicle.b / Vehicle.h * Vehicle.df / Vehicle.L);

    %% Vehicle

    [Solution] = F_Doubletrack_ss(V,deltaf_vec,Vehicle,Tyre,N,choice_model);

    %%     
    % Plot understeer gradient
    ay = Solution.r .* Solution.u;
    [Piccoay(ii),IndicePicco_ay(ii)] = max(ay);
    
    sterzo_din = deg2rad(deltaf_vec) - ((Vehicle.L.*ay)./(Solution.u.^2));
    delta_star=deltaf_vec(IndicePicco_ay(ii));

    [deltal_star,deltar_star]=Ackermann(deg2rad(delta_star),Vehicle);
    
    Grad(ii,:) = (diff(rad2deg(sterzo_din))./diff(ay./9.81));

    figure(1)
    hold on
    plot(ay/9.81,rad2deg(sterzo_din),'color',colorlist(ii,:),LineWidth=1.5);
    
    jj=jj+1;
    leg_V(jj) = strcat('$\delta_D (\frac{K_f}{K} =',num2str(Kf_K_vec(ii)),')$');

end    

%% Graph
figure(1)
grid on
set(gca,'TickLabelInterpreter','latex');
ylabel('$\delta_d$ [deg]',Interpreter='latex',fontsize=14);
xlabel('$\frac{a_y}{g}$',Interpreter='latex',fontsize=16);
xlim([0 Tyre.mu+0.1]);
switch choice_tyres
    case {1 2}
        ylim([0 2]);
    case 3
        ylim([-0.2 1]);
end
yline(0,'--');

%title('Dinamic steering angle',Interpreter='latex',fontsize=16,LineWidth=5);
subtitle(txt,Interpreter='latex',fontsize=12);
xline(Tyre.mu,'--',Color='red');
mu_txt = [' $\mu$ = ' , num2str(Tyre.mu)];

leg_V(end+1) = '';
leg_V(end+1) = mu_txt;
legend(leg_V,Interpreter='latex',fontsize=12);

