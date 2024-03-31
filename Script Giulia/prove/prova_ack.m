
addpath('C:\Users\manue\Desktop\Script\Functions')

toe=0;
Wf=1.6;
L=2.82;

deltaf=6.55; % [deg]

percentuali=[-1:0.1:1];

deltal=zeros(1,length(percentuali));
deltar=zeros(1,length(percentuali));

for ii=1:length(percentuali)
    [deltal(ii),deltar(ii)]=Ackermann(deg2rad(deltaf),toe,percentuali(ii),Wf,L);

end

figure
hold on
grid on
set(gca,'TickLabelInterpreter','latex');
plot(percentuali*100,rad2deg(deltal),linewidth=1.5,color='blue');
plot(percentuali*100,rad2deg(deltar),linewidth=1.5,color='red');
yline(deltaf,'--g',LineWidth=1.5)
%yline(max((rad2deg(deltal))),'--b');
%yline(min((rad2deg(deltar))),'--b');
plot(100,deltaf,'xr','MarkerSize',15,LineWidth=1.5)
plot(100,max((rad2deg(deltar))),'xr','MarkerSize',15,LineWidth=1.5);
plot(100,min((rad2deg(deltal))),'xr','MarkerSize',15,LineWidth=1.5);
legend('$\delta_{out}$','$\delta_{in}$',Interpreter='latex',fontsize=16);
xlabel('Ackermann %');
ylabel('$\delta [deg]$',Interpreter='latex',fontsize=14);