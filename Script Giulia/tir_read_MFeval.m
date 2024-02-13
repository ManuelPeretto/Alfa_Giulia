clc
close all
clear

%%
[file,path] = uigetfile('*.tir');
if isequal(file,0)
   disp('User selected Cancel');
   return
else
   disp(['User selected ', fullfile(path,file)]);
end
tireParams = mfeval.readTIR(fullfile(path,file));

%%

Fz = ones(100,1)*3553;
kappa = zeros(100,1);
alpha = linspace(-0.3,0.3)';
gamma = zeros(100,1);
phit = zeros (100,1);
Vx = ones (100,1)*20;

inputsMF = [Fz kappa alpha gamma phit Vx];

outMF = mfeval(tireParams , inputsMF , 111);

figure();
plot(alpha,-outMF(:,2));
xline(0,'--');
yline(0,'--');
ylabel('F_y [N]');
xlabel('\alpha [rad]');

Cs = -outMF(:,26);
mu = outMF(:,18);
