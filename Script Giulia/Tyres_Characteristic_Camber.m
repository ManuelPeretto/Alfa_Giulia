clc
close all
clear


currentFile = mfilename('fullpath');
[pathstr, ~, ~] = fileparts(currentFile);
addpath(genpath(pathstr));

%%
[Tyre.Params_f] = select_file_tir(pathstr);

%%
n = 1000;
kappa = zeros(n,1);
alpha = linspace(-0.2,+0.2,n)';
phit = zeros (n,1);
Vx = ones (n,1)*20;
Fz = zeros (n,1) + 7000;

camber_vec = [-4:2:4];
color_list = colormap(lines(numel(camber_vec)));

leg=string(numel(camber_vec));
alpha_picco=zeros(1,numel(camber_vec));


for ii=1:numel(camber_vec)

camber=ones(n,1)*deg2rad(camber_vec(ii));

outMF = mfeval(Tyre.Params_f , [Fz kappa alpha -camber phit Vx] , 111);
Fy = -outMF(:,2);

figure(1);
hold on
plot(alpha,Fy,LineWidth=1.5,color=color_list(ii,:));
[PiccoFy,IndicePicco] = max(Fy);
alpha_picco(ii) = alpha(IndicePicco);

leg(ii) = strcat('$\gamma\,=\,',num2str(camber_vec(ii)),'^\circ$');



end

figure(1);
hold on
grid on
set(gca,'TickLabelInterpreter','latex');
xline(0,'--');
yline(0,'--');
ylabel('$F_y\,[N]$',Interpreter='latex',fontsize=14);
xlabel('$\alpha\,[rad]$',Interpreter='latex',fontsize=16);
leg(end+1) = '';
leg(end+1) = '';
txt = [' $F_z = ',num2str(Fz(1)),'\,[N]$'];
subtitle(txt,Interpreter='latex',fontsize=12);

legend(leg,Interpreter='latex',fontsize=16);

figure(2)
hold on
grid on
set(gca,'TickLabelInterpreter','latex');
plot(camber_vec,alpha_picco,LineWidth=1.5);
xlabel('$Camber\,[deg]$',Interpreter='latex',fontsize=14);
ylabel('$\alpha_{Fy_{max}}\,[rad]$',Interpreter='latex',fontsize=16);