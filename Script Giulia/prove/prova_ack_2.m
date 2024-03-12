close all
clear
clc

addpath("C:\Users\manue\Desktop\Script Giulia\Functions")


Wf=1.5;
L=3.11;

deltaf_vec=linspace(0,0.2,100);

toe=deg2rad(0); % toe-in (positivo) [ruote convergono]
percentuale=1;

txt = [' $Ackermann = ',num2str(percentuale*100),'\%$'];


[deltal,deltar]=Ackermann(deltaf_vec,toe,percentuale,Wf,L);


figure
hold on
set(gca,'TickLabelInterpreter','latex');
subtitle(txt,Interpreter='latex',fontsize=14);
plot(rad2deg(deltaf_vec),rad2deg(deltal),linewidth=1.5,color='blue');
plot(rad2deg(deltaf_vec),rad2deg(deltar),linewidth=1.5,color='red');
%yline(0,'--g');
yline(rad2deg(toe),'--b');
yline(rad2deg(-toe),'--b');
%xline(0,'--y');
%xline(rad2deg(toe),'--g');
legend('$\delta_{out}$','$\delta_{in}$','',Interpreter='latex',fontsize=16);
xlabel('steering input $[deg]$',Interpreter='latex',fontsize=14);
ylabel('$\delta [deg]$',Interpreter='latex',fontsize=14);

