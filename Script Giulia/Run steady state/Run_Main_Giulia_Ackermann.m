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

Vehicle.Kf_K = Vehicle.k_roll_f / (Vehicle.k_roll_f + Vehicle.k_roll_r);   % (Kf / Ktotale) Roll stiffness distribution to front axle

Vehicle.toe_f = 0;     % [deg]
Vehicle.toe_r = 0;     % [deg]

%%
Vehicle.Fzf = Vehicle.m * 9.81 * Vehicle.b / Vehicle.L;       % Radial Force on front axle
Vehicle.Fzr = Vehicle.m * 9.81 * Vehicle.a / Vehicle.L;       % Radial Force on rear axle

% load transfer distribution
Vehicle.d = (Vehicle.Kf_K * ( Vehicle.h - Vehicle.dd ) / Vehicle.h) + (Vehicle.b / Vehicle.h * Vehicle.df / Vehicle.L);

%% select file tir

%[Tyre] = select_file_tir(path_giulia);

Tyre.Params_f = mfeval.readTIR("C:\Users\manue\Desktop\Script Giulia\file tir\Toyo_AlfaGiulia.tir");
Tyre.Params_r = mfeval.readTIR("C:\Users\manue\Desktop\Script Giulia\file tir\Toyo_AlfaGiulia.tir");

%Tyre.Params_f = mfeval.readTIR("C:\Users\manue\Desktop\Script Giulia\file tir\FRONT_V1Pirelli_Cinturato_AR_Giulia_2.2_JTD_150_AT8_DC_LMUX_OK_LMUY_V1.tir");
%Tyre.Params_r = mfeval.readTIR("C:\Users\manue\Desktop\Script Giulia\file tir\REAR_V1Pirelli_Cinturato_AR_Giulia_2.2_JTD_150_AT8_DC_LMUX_OK_LMUY_V1.tir");

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

% percentuale_Ack_vec = [0 : 0.5 : 1];
percentuale_Ack_vec = [-8.5,-1,0,1];
n = numel(percentuale_Ack_vec);

leg_V=string(n*2);
Grad = zeros(n,N-1);
colorlist = colormap(lines(n));
jj=0;

V = zeros(1,N)+V_vec(1)/3.6;
Piccoay = zeros (n,1);
IndicePicco_ay = zeros (n,1);

txt = [' $Rampsteer','',num2str(V_vec(1)),' [km/h]$'];

for ii=1:n

    Vehicle.percentuale_Ack = percentuale_Ack_vec(ii);

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
    plot(ay/9.81,rad2deg(sterzo_din),'color',colorlist(ii,:));
    
    jj=jj+1;
    leg_V(jj) = strcat('$\delta_D (Ack =',num2str(percentuale_Ack_vec(ii)),')$ , $\delta_{in} =',num2str(rad2deg(deltar_star)),'$ [deg] , $\delta_{out} =',num2str(rad2deg(deltal_star)),'$ [deg], $\delta_{input} =',num2str(delta_star),'$ [deg]');

    %scatter(ay(1,2)./9.81,Grad(ii,1),'MarkerEdgeColor',colorlist(ii,:),'MarkerFaceColor',colorlist(ii,:),LineWidth=2);
    %jj=jj+1;
    %leg_V(jj) = strcat('$\frac{d(\delta_D)}{d(ay/g)} = ' , num2str(Grad(ii,1)),' [deg/g]$');
    
    sub_txt = strcat(txt,' , ',' Ack =',num2str(percentuale_Ack_vec(ii)));

%     figure
%     hold on
%     set(gca,'TickLabelInterpreter','latex');
%     subtitle(sub_txt,Interpreter='latex',fontsize=12);
%     scatter(Solution.alfa_fl,Solution.Fy_fl,'filled');
%     scatter(Solution.alfa_fr,Solution.Fy_fr,'filled');
%     scatter(Solution.alfa_rl,Solution.Fy_rl,'filled');
%     scatter(Solution.alfa_rr,Solution.Fy_rr,'filled');
%     xlabel('$\alpha [rad]$',Interpreter='latex',fontsize=16);
%     ylabel('$F_y [N]$',Interpreter='latex',fontsize=14);
%     ylim([0 inf]);
%     legend('$F_{Yfl}$','$F_{Yfr}$','$F_{Yrl}$','$F_{Yrr}$',Interpreter='latex',fontsize=16);
end    

%% Graph
figure(1)
grid on
set(gca,'TickLabelInterpreter','latex');
ylabel('$\delta_d$ [deg]',Interpreter='latex',fontsize=14);
xlabel('$\frac{a_y}{g}$',Interpreter='latex',fontsize=16);
xlim([0 Tyre.mu+0.1]);
ylim([0 2]);
yline(0,'--');

title('Dinamic steering angle',Interpreter='latex',fontsize=16,LineWidth=5);
subtitle(txt,Interpreter='latex',fontsize=12);
xline(Tyre.mu,'--',Color='red');
mu_txt = [' $\mu$ = ' , num2str(Tyre.mu)];

leg_V(end+1) = '';
leg_V(end+1) = mu_txt;
legend(leg_V,Interpreter='latex',fontsize=12);


% 
% figure()
% hold on
% set(gca,'TickLabelInterpreter','latex');
% subtitle(sub_txt,Interpreter='latex',fontsize=12);
% plot(percentuale_Ack_vec,Piccoay/9.81,color='blue',LineWidth=1.5);
% xlabel('Ackermann',Interpreter='latex',fontsize=14);
% ylabel('$max \frac{a_y}{g}$',Interpreter='latex',fontsize=16);
% xline(0,'--');
% indice_zero = find(~percentuale_Ack_vec);
% Pendenza = diff(Piccoay(indice_zero-1:indice_zero)./9.81) ./ diff(percentuale_Ack_vec(indice_zero-1:indice_zero));
% plot(percentuale_Ack_vec,(Piccoay(indice_zero)./9.81)+percentuale_Ack_vec.*Pendenza,'--r');
% 
% [Piccoay_ack,IndicePiccoay] = max(Piccoay/9.81);
% xline(percentuale_Ack_vec(1,IndicePiccoay),'--g');
% plot(percentuale_Ack_vec(1,IndicePiccoay) , Piccoay_ack , 'xr','MarkerSize',20);
% 
% ackermann_ott = strcat('Ackermann ottimale =',num2str(percentuale_Ack_vec(1,IndicePiccoay)));
% ay_max = strcat('$Ay_{max}$ =',num2str(Piccoay_ack));
% legend('','','',ackermann_ott,ay_max,Interpreter='latex',fontsize=12);
%set(gca,'Xtick',-5:1:5,'XMinorTick','on',XTickLabelRotation = 0);
%set(gca,'Ytick',YTickLabelRotation = 0);

