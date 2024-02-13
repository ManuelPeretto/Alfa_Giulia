clear
close

currentFile = mfilename('fullpath');
[pathstr, ~, ~] = fileparts(currentFile);
addpath(genpath(pathstr));

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
Vehicle.percentuale_Ack = 0.5;   % Ackermann


%%
Vehicle.Fzf = Vehicle.m * 9.81 * Vehicle.b / Vehicle.L;       % Radial Force on front axle
Vehicle.Fzr = Vehicle.m * 9.81 * Vehicle.a / Vehicle.L;       % Radial Force on rear axle

% load transfer distribution
Vehicle.d = Vehicle.Kf_K ;%* ( Vehicle.h - Vehicle.b ) / Vehicle.h + Vehicle.b / Vehicle.h * Vehicle.b / Vehicle.L;

%% select file tir

[file,path] = uigetfile('*.tir');
if isequal(file,0)
   disp('User selected Cancel');
   return
else
   disp(['User selected ', fullfile(path,file)]);
end
Tyre.Params_f = mfeval.readTIR(fullfile(path,file));
Tyre.Params_r = mfeval.readTIR(fullfile(path,file));
%% Set simulation Time
dt=0.1;    % time step [s]
tMax=30;   % final time [s]

tvec=[0:dt:tMax]; % generate time vector
N=length(tvec);

%% Rampsteer simulation
     
deltaf_vec=linspace(0,15,N); % vector steering angle fixed

V_vec=[30:30:120];

%%
choice_model = menu("Choose a Vehicle model","Single track NON linear","Double track NON linear","Single track linear","Double track linear");

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
    switch choice_model
        case 1
            [Alfa,Force,Solution] = F_Singletrack_nonlinear_ss_Giulia(V,deltaf_vec,Vehicle,Tyre,N);
        case 2
            [Alfa,Force,Solution,delta] = F_Doubletrack_nonlinear_ss_Giulia(V,deltaf_vec,Vehicle,Tyre,N);
        case 3
            [Alfa,Force,Solution] = F_Singletrack_linear_ss(V,deltaf_vec,Vehicle,Tyre,N);
        case 4    
            [Alfa,Force,Solution,delta] = F_Doubletrack_linear_ss(V,deltaf_vec,Vehicle,Tyre,N);
    end      
    %%     
    % Plot understeer gradient
    ay = Solution.r .* Solution.u;

    % Formula (2) Canton pag.5.
    sterzo_din = deg2rad(deltaf_vec) - ((Vehicle.L.*ay)./(Solution.u.^2));
    
    Grad(ii,:) = (diff(rad2deg(sterzo_din))./diff(ay./9.81));

    figure(1)
    hold on
    plot(ay/9.81,rad2deg(sterzo_din),'color',colorlist(ii,:));
    
    jj=jj+1;
    leg_V(jj) = strcat('$\delta_D(u =',num2str(V_vec(ii)),' [km/h])$');

    scatter(ay(1,2)./9.81,Grad(ii,1),'MarkerEdgeColor',colorlist(ii,:),'MarkerFaceColor',colorlist(ii,:),LineWidth=2);
    jj=jj+1;
    leg_V(jj) = strcat('$\frac{d(\delta_D)}{d(ay/g)} = ' , num2str(Grad(ii,1)),' [deg/g]$');
end    

%% Graph
figure(1)
set(gca,'TickLabelInterpreter','latex');
ylabel('$\delta_d$ [deg]',Interpreter='latex',fontsize=14);
xlabel('$\frac{a_y}{g}$',Interpreter='latex',fontsize=16);
xlim([0 Tyre.mu+0.1]);
ylim([0 1]);
yline(0,'--');

title('Dinamic steering angle',Interpreter='latex',fontsize=16,LineWidth=5);
txt = [' $\zeta = ',num2str(rad2deg(Vehicle.Gradiente)),' [deg/g]$'];
subtitle(txt,Interpreter='latex',fontsize=12);
xline(Tyre.mu,'--',Color='red');
mu_txt = [' $\mu$ = ' , num2str(Tyre.mu)];

leg_V(end+1) = '';
leg_V(end+1) = mu_txt;
legend(leg_V,Interpreter='latex',fontsize=12);