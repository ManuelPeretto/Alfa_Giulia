clc
close all
clear


currentFile = mfilename('fullpath');
[pathstr, ~, ~] = fileparts(currentFile);
addpath(genpath(pathstr));

%%
Tyre.Params_f = select_file_tir(pathstr);

%%
n = 1000;
kappa = zeros(n,1);
alpha = linspace(0,+0.3,n)';
gamma = zeros(n,1);
phit = zeros (n,1);
Vx = ones (n,1)*20;

Fz_vec = [1000:1000:6000];

leg=string(numel(Fz_vec));
alpha_picco=zeros (numel(Fz_vec),1);
Fy_picco=zeros (numel(Fz_vec),1);
Cs = zeros (numel(Fz_vec),1);
Cs_Fz = zeros (numel(Fz_vec),1);
mu = zeros (numel(Fz_vec),1);

for ii=1:numel(Fz_vec)

Fz = ones(n,1)*Fz_vec(ii);

outMF = mfeval(Tyre.Params_f , [Fz kappa alpha gamma phit Vx] , 111);
Fy = -outMF(:,2);

figure(1);
hold on
plot(alpha,Fy,LineWidth=1.5);
[PiccoFy,IndicePicco] = max(Fy);
Fy_picco(ii) = PiccoFy;
alpha_picco(ii) = alpha(IndicePicco);
%plot(percentuale_Ack_vec(1,IndicePiccoay) , alpha_picco(ii) , 'xr','MarkerSize',20);

leg(ii) = strcat('$F_y (F_z = ',num2str(Fz_vec(ii)),' [N]$');

Cs(ii) = mean(-outMF(:,26));
mu(ii) = mean(outMF(:,18));
Cs_Fz(ii) = Cs(ii)/Fz_vec(ii);

end

figure(1);
hold on
grid on
set(gca,'TickLabelInterpreter','latex');
xline(0,'--');
yline(0,'--');
ylabel('$F_y [N]$',Interpreter='latex',fontsize=14);
xlabel('$\alpha [rad]$',Interpreter='latex',fontsize=16);
lim = Fz_vec(end)+2000;
%set(gca,'Ytick',-lim:1e3:lim,YTickLabelRotation = 0);
leg(end+1) = '';
leg(end+1) = '';
ylim([0 lim]);
plot(alpha_picco , Fy_picco , 'xr','MarkerSize',20);

legend(leg,Interpreter='latex',fontsize=12);


% figure(2)
% hold on
% grid on
% set(gca,'TickLabelInterpreter','latex');
% plot(Fz_vec,alpha_picco,LineWidth=1.5);
% xlabel('$F_z [N]$',Interpreter='latex',fontsize=14);
% ylabel('$\alpha_{Fy_{max}} [rad]$',Interpreter='latex',fontsize=16);

