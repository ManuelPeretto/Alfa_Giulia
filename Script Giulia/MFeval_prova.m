% Verifico se MFeval considera l'effetto della velocità
clc
close all
clear

currentFile = mfilename('fullpath');
[pathstr, ~, ~] = fileparts(currentFile);

%%
folder = strcat(pathstr,'\file tir');
[file,path] = uigetfile('*.tir','Select a file .tir',folder);
if isequal(file,0)
   disp('User selected Cancel');
   return
else
   disp(['User selected ', fullfile(path,file)]);
end
tireParams = mfeval.readTIR(fullfile(path,file));

%%
n = 1000;
kappa = zeros(n,1);
alpha = linspace(-0.3,+0.3,n)';
gamma = zeros(n,1);
phit = zeros (n,1);
Vx_vec = [50:25:150];

Fz = ones(n,1)*4000;


for ii=1:numel(Vx_vec)

%Fz = ones(n,1)*Fz_vec(ii);
Vx = ones (n,1)*(Vx_vec(ii)./3.6);
outMF = mfeval(tireParams , [Fz kappa alpha gamma phit Vx] , 111);
Fy = -outMF(:,2);

figure(1)
hold on
plot(alpha,Fy);

Cs = -outMF(:,26);
mu = outMF(:,18);
end

figure(1);
hold on
set(gca,'TickLabelInterpreter','latex');
%subtitle(file,Interpreter='latex',fontsize=8);
xline(0,'--');
yline(0,'--');
ylabel('$F_y [N]$',Interpreter='latex',fontsize=14);
xlabel('$\alpha [rad]$',Interpreter='latex',fontsize=16);
set(gca,'Ytick',-5e3:1e3:6e3,YTickLabelRotation = 0);
%leg(end+1) = '';
%leg(end+1) = '';


%legend(leg,Interpreter='latex',fontsize=12);
