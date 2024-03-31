% Function 
function app=f_runplot(app)

app.XDropDown.Items = app.Solution.Properties.VariableNames;
app.YDropDown.Items = app.Solution.Properties.VariableNames;
app.ColorDropDown.Items = app.Solution.Properties.VariableNames;

app.XDropDown.Value = 'ay_g';
app.YDropDown.Value = 'sterzo_din';
switch app.TestDropDown.Value
    case 1
        app.ColorDropDown.Value='u';
    case 2
        app.ColorDropDown.Value='deltaf_vec';
    case 3
        app.ColorDropDown.Value='r_u';
end
        app = f_update_plot(app);
        app.XMinEditField.Value = app.UIAxes.XLim(1);
        app.XMaxEditField.Value = app.UIAxes.XLim(2);
        app.YMinEditField.Value = app.UIAxes.YLim(1);
        app.YMaxEditField.Value = app.UIAxes.YLim(2);

end

















% leg = string(numel(R)*2 +2);
% Grad = zeros(numel(R),N-1);
% colorlist = colormap(lines(numel(R)));
% jj=0;
% 
% ay = Solution.r .* Solution.u;
% 
% deltaf_vec = Solution.deltaf_vec;
% 
% sterzo_din = deltaf_vec - ((Vehicle.L.*ay)./(Solution.u.^2));
% 
% Grad(ii,:) = (diff(rad2deg(sterzo_din))./diff(ay./9.81));
% 
% figure(1)
% hold on
% plot(ay/9.81,rad2deg(sterzo_din),'color',colorlist(ii,:));
% 
% jj=jj+1;
% leg(jj) = strcat('$\delta_D(R =',num2str(R(ii)),' [m])$');
% 
% scatter(ay(1,10)./9.81,Grad(ii,10),'MarkerEdgeColor',colorlist(ii,:),'MarkerFaceColor',colorlist(ii,:),LineWidth=2);
% jj=jj+1;
% leg(jj) = strcat('$\frac{d(\delta_D)}{d(ay/g)} = ' , num2str(Grad(ii,10)),' [deg/g]$');
% 
% 
% 
% 
% figure(1)
% grid on
% set(gca,'TickLabelInterpreter','latex');
% ylabel('$\delta_d$ [deg]',Interpreter='latex',fontsize=14);
% xlabel('$\frac{a_y}{g}$',Interpreter='latex',fontsize=16);
% xlim([0 Tyre.mu+0.1]);
% ylim([0 2]);
% yline(0,'--');
% 
% title('Dinamic steering angle',Interpreter='latex',fontsize=16,LineWidth=5);
% txt = [' $\zeta = ',num2str(rad2deg(Vehicle.Gradiente)),' [deg/g]$'];
% subtitle(txt,Interpreter='latex',fontsize=12);
% xline(Tyre.mu,'--',Color='red');
% mu_txt = [' $\mu$ = ' , num2str(Tyre.mu)];
% 
% leg(end+1) = '';
% leg(end+1) = mu_txt;
% legend(leg,Interpreter='latex',fontsize=12);
