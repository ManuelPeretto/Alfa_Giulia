



function Create_graph_sterzo_dinamico(Solution,deltaf_vec,Vehicle,mu)

V=Solution.u;
r_vec=Solution.r;

L=Vehicle.L;
Gradiente=Vehicle.Gradiente;

% Plot understeer gradient
ay = r_vec .* V;

% Formula (2) Canton pag.5.
sterzo_din = deltaf_vec - ((L.*ay)./(V.^2));



figure
hold on
plot(ay/9.81,rad2deg(sterzo_din));
set(gca,'TickLabelInterpreter','latex');
ylabel('$\delta_d$ [deg]',Interpreter='latex',fontsize=14);
xlabel('$\frac{a_y}{g}$',Interpreter='latex',fontsize=16);
xlim([0 mu+0.1]);
%ylim([-inf 1.5]);
yline(0,'--');

title('Dinamic steering angle',Interpreter='latex',fontsize=16,LineWidth=5);
txt = [' $\zeta = $',num2str(rad2deg(Gradiente)),' [deg/g]'];
subtitle(txt,Interpreter='latex',fontsize=12);
xline(mu,'--',Color='red');
mu_txt = [' $\mu$ = ' , num2str(mu)];

scatter(ay(1:19)./9.81,diff(rad2deg(sterzo_din(1:20)))./diff(ay(1:20)./9.81),'MarkerEdgeColor','blue','MarkerFaceColor','blue');

Grad = (diff(rad2deg(sterzo_din))./diff(ay./9.81));
Grad_txt = ['$\frac{d(\delta_D)}{d(ay/g)}$ = ' , num2str(Grad(8)),' [deg/g]'];
legend('$\delta_D=\delta - \frac{L}{V^2}*a_y$','',mu_txt,Grad_txt,Interpreter='latex',fontsize=14);


