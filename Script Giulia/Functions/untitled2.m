
clear
close all
clc

FzC = 4100;
KC = 0;
[Fy,Cy,mu] = F_pacejka_toyo_AlfaGiulia(FzC,KC);

alfa=[-0.3:0.001:0.3];
Force=-Fy(alfa,4100,0);
plot(alfa,Force);
xline(0,'--');
yline(0,'--');