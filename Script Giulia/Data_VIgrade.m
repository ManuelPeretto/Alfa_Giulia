clear
close all
clc
%data=readtable("data_ramp_steer.csv");
data=readtable("data_manuel_11_03_24.csv");

Vehicle.tau = 11.8;
Vehicle.L = 2.82;

time = data.time_TIME;

deltaf_vec = data.driver_demands_driver_steering_angle ./ Vehicle.tau; % [rad]
u = data.chassis_velocities_longitudinal;
ay = -data.chassis_accelerations_lateral;

sterzo_din = (deltaf_vec) - ((Vehicle.L.*ay)./(u.^2));
mu = max(ay);

alfa_fl = data.Tire_Lateral_Slip_Without_Lag_L1;
alfa_fr = data.Tire_Lateral_Slip_Without_Lag_R2;
alfa_rl = data.Tire_Lateral_Slip_Without_Lag_L1;
alfa_rr = data.Tire_Lateral_Slip_Without_Lag_R2;


figure()
hold on
plot(time,data.Tire_Ground_Surface_Force_Y_L1,linewidth=1.5);
plot(time,data.Tire_Ground_Surface_Force_Y_R1,linewidth=1.5);
plot(time,data.Tire_Ground_Surface_Force_Y_L2,linewidth=1.5);
plot(time,data.Tire_Ground_Surface_Force_Y_R2,linewidth=1.5);
legend('front left','front right','rear left','rear right');

figure()
hold on
set(gca,'TickLabelInterpreter','latex');
plot(-alfa_fl,data.Tire_Ground_Surface_Force_Y_L1,linewidth=1.5);
plot(-alfa_fr,data.Tire_Ground_Surface_Force_Y_R1,linewidth=1.5);
plot(-alfa_rl,data.Tire_Ground_Surface_Force_Y_L2,linewidth=1.5);
plot(-alfa_rr,data.Tire_Ground_Surface_Force_Y_R2,linewidth=1.5);
legend('front left','front right','rear left','rear right',Interpreter='latex',fontsize=14);
xlim([0 inf]);
ylabel('$F_y [N]$',Interpreter='latex',fontsize=14);
xlabel('$\alpha [rad]$',Interpreter='latex',fontsize=16);

figure()
hold on
plot(time,data.Tire_Ground_Surface_Force_Z_L1,linewidth=1.5);
plot(time,data.Tire_Ground_Surface_Force_Z_R1,linewidth=1.5);
plot(time,data.Tire_Ground_Surface_Force_Z_L2,linewidth=1.5);
plot(time,data.Tire_Ground_Surface_Force_Z_R2,linewidth=1.5);
legend('front left','front right','rear left','rear right',Interpreter='latex',fontsize=14);
ylabel('$F_z [N]$',Interpreter='latex',fontsize=14);
xlabel('time [sec]',Interpreter='latex',fontsize=14);

figure()
hold on
set(gca,'TickLabelInterpreter','latex');
plot(data.Tire_Ground_Surface_Force_Z_L1 , data.Tire_Ground_Surface_Force_Y_L1 , linewidth=1.5);
plot(data.Tire_Ground_Surface_Force_Z_R1 , data.Tire_Ground_Surface_Force_Y_R1 , linewidth=1.5);
plot(data.Tire_Ground_Surface_Force_Z_L2 , data.Tire_Ground_Surface_Force_Y_L2 , linewidth=1.5);
plot(data.Tire_Ground_Surface_Force_Z_R2 , data.Tire_Ground_Surface_Force_Y_R2 , linewidth=1.5);
legend('front left','front right','rear left','rear right',Interpreter='latex',fontsize=14);
xlim([0 inf]);
ylabel('$F_y [N]$',Interpreter='latex',fontsize=14);
xlabel('$F_z [N]$',Interpreter='latex',fontsize=14);



%Fy_fl = data.;
%Fy_fr = data.;
%Fy_rl = data.;
%Fy_rr = data.;

camber_fl = data.wheel_angles_camber_L1;
camber_fr = data.wheel_angles_camber_R1;
camber_rl = data.wheel_angles_camber_L2;
camber_rr = data.wheel_angles_camber_R2;


aero_balance = data.aero_forces_aero_balance;

downforce_front = data.aero_forces_front_downforce;
downforce_rear = data.aero_forces_rear_downforce;
dragforce = data.aero_forces_drag_force;


figure
hold on
plot(ay,rad2deg(sterzo_din));
set(gca,'TickLabelInterpreter','latex');
ylabel('$\delta_d$ [deg]',Interpreter='latex',fontsize=14);
xlabel('$\frac{a_y}{g}$',Interpreter='latex',fontsize=16);
xlim([0 mu+0.1]);
%ylim([0 10]);
yline(0,'--');

title('Dinamic steering angle',Interpreter='latex',fontsize=16,LineWidth=5);
txt = ['Rampsteer 60 [km/h]'];
subtitle(txt,Interpreter='latex',fontsize=14);
mu_txt = [' $\mu$ = ' , num2str(mu)];

Grad = (diff(rad2deg(sterzo_din))./diff(ay));
scatter(ay(10),mean(Grad(10:100)),'MarkerEdgeColor','blue','MarkerFaceColor','blue');

Grad_txt = ['$\frac{d(\delta_D)}{d(ay/g)}$ = ' , num2str(mean(Grad(10:100))),' [deg/g]'];
legend('$\delta_D=\delta - \frac{L}{V^2}*a_y$','',Grad_txt,Interpreter='latex',fontsize=14);



c_fl = rad2deg(camber_fl(1));
c_fr = rad2deg(camber_fr(1));
c_rl = rad2deg(camber_rl(1));
c_rr = rad2deg(camber_rr(1));


% canali che hanno in uscita l'angolo di sterzo complessivo di ogni singola
% ruota
toe_fl = data.wheel_angles_toe_L1;
toe_fr = data.wheel_angles_toe_R1;
toe_rl = data.wheel_angles_toe_L2;
toe_rr = data.wheel_angles_toe_R2;



% figure()
% hold on
% set(gca,'TickLabelInterpreter','latex');
% plot(time,camber_fl,linewidth=1.5);
% plot(time,camber_fr,linewidth=1.5);
% plot(time,camber_rl,linewidth=1.5);
% plot(time,camber_rr,linewidth=1.5);
% ylabel('camber angle [rad]',Interpreter='latex',fontsize=14);
% xlabel('time [sec]',Interpreter='latex',fontsize=14);
% legend('front left','front right','rear left','rear right');

% figure()
% hold on
% set(gca,'TickLabelInterpreter','latex');
% plot(rad2deg(deltaf_vec),rad2deg(-toe_fl), linewidth=1.5);
% plot(rad2deg(deltaf_vec),rad2deg(toe_fr) , linewidth=1.5);
% ylabel('steering at wheels [deg]',Interpreter='latex',fontsize=14);
% xlabel('steering input [deg]',Interpreter='latex',fontsize=14);
% yline(0,'--r');
% legend('Front left','Front right');
% toe_txt = [' toe = ' , num2str(rad2deg(-toe_fl(1))),' [deg]'];
% subtitle(toe_txt,Interpreter='latex',fontsize=14);
% 
% figure()
% hold on
% set(gca,'TickLabelInterpreter','latex');
% plot(time,rad2deg(-toe_fl), linewidth=1.5);
% plot(time,rad2deg(toe_fr) , linewidth=1.5);
% ylabel('steering at wheels [deg]',Interpreter='latex',fontsize=14);
% xlabel('time [sec]',Interpreter='latex',fontsize=14);
% yline(0,'--r');
% legend('Front left','Front right');
% subtitle(toe_txt,Interpreter='latex',fontsize=14);

% figure()
% hold on
% set(gca,'TickLabelInterpreter','latex');
% plot(time,downforce_front,linewidth=1.5);
% plot(time,downforce_rear,linewidth=1.5);
% plot(time,dragforce,linewidth=1.5);
% ylabel('Force [N]',Interpreter='latex',fontsize=14);
% xlabel('time [sec]',Interpreter='latex',fontsize=14);
% legend('front','rear','drag');
% aero_txt = [' aerobalance = ' , num2str(aero_balance(1))];
% subtitle(aero_txt,Interpreter='latex',fontsize=14);


figure()
hold on
set(gca,'TickLabelInterpreter','latex');
gradiente = data.Vehicle_Understeer_Gradient;
plot(time,rad2deg(gradiente),linewidth=1.5);
ylabel('Understeer gradient [deg/g]',Interpreter='latex',fontsize=14);
xlabel('time [sec]',Interpreter='latex',fontsize=14);