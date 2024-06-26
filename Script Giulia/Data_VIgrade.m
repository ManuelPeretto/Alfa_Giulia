% This code has been used to analize Vi-grade simulation data. 
clear
close all
clc
% you need to change the file directory
data=readtable("C:\Users\manue\Desktop\Simulazioni Vi-grade rampsteer\data_Manuel_long_13_03_2024.csv");

Vehicle.tau = 11.8;
Vehicle.L = 2.82;

time = data.time_TIME;

deltaf_vec = data.driver_demands_driver_steering_angle ./ Vehicle.tau; % [rad]
u = data.chassis_velocities_longitudinal./3.6;
ay = -data.chassis_accelerations_lateral.*9.81;

sterzo_din = (deltaf_vec) - ((Vehicle.L.*ay)./(u.^2));
mu = max(ay./9.81);

alfa_fl = data.Tire_Lateral_Slip_Without_Lag_L1;
alfa_fr = data.Tire_Lateral_Slip_Without_Lag_R1;
alfa_rl = data.Tire_Lateral_Slip_Without_Lag_L2;
alfa_rr = data.Tire_Lateral_Slip_Without_Lag_R2;

alfa_f = -(alfa_fl + alfa_fr) ./ 2;
alfa_r = -(alfa_rl + alfa_rr) ./ 2;


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Plot Alfa Vs Fy
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

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Plot Time Vs Fz
figure()
hold on
set(gca,'TickLabelInterpreter','latex');
plot(time,data.Tire_Ground_Surface_Force_Z_R1,linewidth=1.5);
plot(time,data.Tire_Ground_Surface_Force_Z_L1,linewidth=1.5);
plot(time,data.Tire_Ground_Surface_Force_Z_R2,linewidth=1.5);
plot(time,data.Tire_Ground_Surface_Force_Z_L2,linewidth=1.5);
legend('$F_{Zfl}$','$F_{Zfr}$','$F_{Zrl}$','$F_{Zrr}$',Interpreter='latex',fontsize=16);
ylabel('$F_z [N]$',Interpreter='latex',fontsize=14);
xlabel('Time [s]',Interpreter='latex',fontsize=14);

massa= 1/9.81 *(data.Tire_Ground_Surface_Force_Z_L1(100) + data.Tire_Ground_Surface_Force_Z_R1(100) + data.Tire_Ground_Surface_Force_Z_L2(100) + data.Tire_Ground_Surface_Force_Z_R2(100));

camber_fl = data.wheel_angles_camber_L1;
camber_fr = data.wheel_angles_camber_R1;
camber_rl = data.wheel_angles_camber_L2;
camber_rr = data.wheel_angles_camber_R2;

toe_fl = data.wheel_angles_toe_L1;
toe_fr = data.wheel_angles_toe_R1;
toe_rl = data.wheel_angles_toe_L2;
toe_rr = data.wheel_angles_toe_R2;


aero_balance = data.aero_forces_aero_balance;

downforce_front = data.aero_forces_front_downforce;
downforce_rear = data.aero_forces_rear_downforce;
dragforce = data.aero_forces_drag_force;


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Plot Dinamic steering Angle
figure
hold on
grid on
plot((ay./9.81),rad2deg(alfa_f-alfa_r),color='green',LineWidth=1.5);
set(gca,'TickLabelInterpreter','latex');
ylabel('$\delta_D$\,[deg]',Interpreter='latex',fontsize=14);
xlabel('$\frac{a_y}{g}$',Interpreter='latex',fontsize=16);
xlim([0 mu+0.1]);
ylim([0 8]);
yline(0,'--');

title('Dinamic steering angle',Interpreter='latex',fontsize=16,LineWidth=5);
txt = ['Rampsteer 60 [km/h]'];
subtitle(txt,Interpreter='latex',fontsize=14);
mu_txt = [' $\mu$ = ' , num2str(mu)];

Grad = (diff(rad2deg(sterzo_din))./diff(ay./9.81));
scatter(ay(10),mean(Grad(10:300)),'MarkerEdgeColor','blue','MarkerFaceColor','blue');

Grad_txt = ['$\frac{d(\delta_D)}{d(ay/g)}$ = ' , num2str(mean(Grad(10:300))),' [deg/g]'];
legend('$\delta_D=\delta - \frac{L}{V^2}*a_y$','',Grad_txt,Interpreter='latex',fontsize=14);


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Plot Camber
figure()
hold on
set(gca,'TickLabelInterpreter','latex');
plot(time,rad2deg(camber_fl),linewidth=1.5);
plot(time,rad2deg(camber_fr),linewidth=1.5);
plot(time,rad2deg(camber_rl),linewidth=1.5);
plot(time,rad2deg(camber_rr),linewidth=1.5);
ylabel('camber angle [deg]',Interpreter='latex',fontsize=14);
xlabel('Time\,[s]',Interpreter='latex',fontsize=14);
legend('front left','front right','rear left','rear right',Interpreter='latex',fontsize=14);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Plot Toe angles
toe_txt=string(4);
figure()
hold on
set(gca,'TickLabelInterpreter','latex');
plot(time,rad2deg(-toe_fl), linewidth=1.5);
plot(time,rad2deg(toe_fr) , linewidth=1.5);
plot(time,rad2deg(-toe_rl), linewidth=1.5);
plot(time,rad2deg(toe_rr) , linewidth=1.5);
ylabel('Steering at wheels\,[deg]',Interpreter='latex',fontsize=14);
xlabel('Time\,[s]',Interpreter='latex',fontsize=14);
yline(0,'--r');
toe_txt(1) = ['Front left , toe = ' , num2str(rad2deg(toe_fl(1))),' [deg]'];
toe_txt(2) = ['Front right , toe = ' , num2str(rad2deg(toe_fr(1))),' [deg]'];
toe_txt(3) = ['Rear left , toe = ' , num2str(rad2deg(toe_rl(1))),' [deg]'];
toe_txt(4) = ['Rear right , toe = ' , num2str(rad2deg(toe_rr(1))),' [deg]'];
legend(toe_txt,Interpreter='latex',fontsize=14);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Plot aero Forces
figure()
hold on
set(gca,'TickLabelInterpreter','latex');
plot(time,downforce_front,linewidth=1.5);
plot(time,downforce_rear,linewidth=1.5);
plot(time,dragforce,linewidth=1.5);
ylabel('Force\,[N]',Interpreter='latex',fontsize=14);
xlabel('Time\,[s]',Interpreter='latex',fontsize=14);
legend('Front','Rear','Drag');
aero_txt = [' aerobalance = ' , num2str(aero_balance(1))];
subtitle(aero_txt,Interpreter='latex',fontsize=14);

