
leg_V=string(numel(deltaf)*2);
Grad = zeros(numel(deltaf),N-1);
colorlist = colormap(lines(numel(deltaf)));
jj=0;

for ii=1:numel(deltaf)

   deltaf_vec = zeros(1,N) + deltaf(ii);  % vector steer angle fixed
   
    %% Vehicle
    switch choice_model
        case {1 2}
            [Solution] = F_Singletrack_ss(V_vec,deltaf_vec,Vehicle,Tyre,N,choice_model);

        case {3 4}    
            [Solution] = F_Doubletrack_ss(V_vec,deltaf_vec,Vehicle,Tyre,N,choice_model);
    end      
    %%     
    % Plot understeer gradient
    ay = Solution.r .* Solution.u;

    % Formula (2) Canton pag.5.
    sterzo_din = deg2rad(deltaf_vec) - ((Vehicle.L.*ay)./(Solution.u.^2));
    
    Grad(ii,:) = (diff(rad2deg(sterzo_din))./diff(ay./9.81));

    figure(1)
    hold on
    plot(ay/9.81,rad2deg(sterzo_din),'color',colorlist(ii,:));
    
    jj=jj+1;
    leg_V(jj) = strcat('$\delta_D(\delta =',num2str(deltaf(ii)),' [deg])$');

    scatter(ay(1,2)./9.81,mean(Grad(ii,8:30)),'MarkerEdgeColor',colorlist(ii,:),'MarkerFaceColor',colorlist(ii,:),LineWidth=2);
    jj=jj+1;
    leg_V(jj) = strcat('$\frac{d(\delta_D)}{d(ay/g)} = ' , num2str(mean(Grad(ii,8:30))),' [deg/g]$');
end    

%% Graph
figure(1)
grid on
set(gca,'TickLabelInterpreter','latex');
ylabel('$\delta_d$ [deg]',Interpreter='latex',fontsize=14);
xlabel('$\frac{a_y}{g}$',Interpreter='latex',fontsize=16);
xlim([0 Tyre.mu+0.1]);
switch choice_tyres
    case 2
        ylim([0 2]);
    case 3
        ylim([-0.2 1]);
end
yline(0,'--');

title('Dinamic steering angle',Interpreter='latex',fontsize=16,LineWidth=5);
txt = [' $\zeta = ',num2str(rad2deg(Vehicle.Gradiente)),' [deg/g]$'];
subtitle(txt,Interpreter='latex',fontsize=12);
xline(Tyre.mu,'--',Color='red');
mu_txt = [' $\mu$ = ' , num2str(Tyre.mu)];

leg_V(end+1) = '';
leg_V(end+1) = mu_txt;
legend(leg_V,Interpreter='latex',fontsize=12);
