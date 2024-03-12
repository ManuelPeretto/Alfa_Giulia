
addpath('C:\Users\manue\Desktop\Script\Functions')

toe=0;
Wf=1.5;
L=3.11;

deltaf=0.145;

percentuali=[-1:0.1:1];

deltal=zeros(1,length(percentuali));
deltar=zeros(1,length(percentuali));

for ii=1:length(percentuali);
    [deltal(ii),deltar(ii)]=Ackermann(deltaf,toe,percentuali(ii),Wf,L);

end

figure
hold on
plot(percentuali,deltal);
plot(percentuali,deltar);
yline(deltaf,'--g')
legend('deltal','deltar');
xlabel('ackermann percent');
ylabel('\delta [rad]');