function [Solution] = F_Doubletrack_ss(V,deltaf_vec,Vehicle,Tyre,N,choice_model)

Solution = table;
toe_f=Vehicle.toe_f;
percentuale_Ack=Vehicle.percentuale_Ack;


alfafl_vec=zeros(N,1);
alfafr_vec=zeros(N,1);
alfarl_vec=zeros(N,1);
alfarr_vec=zeros(N,1);
Fyfl_vec=zeros(N,1);
Fyfr_vec=zeros(N,1);
Fyrl_vec=zeros(N,1);
Fyrr_vec=zeros(N,1);
r_vec=zeros(N,1);
v_vec=zeros(N,1);
Fzfl_vec=zeros(N,1);
Fzfr_vec=zeros(N,1);
Fzrl_vec=zeros(N,1);
Fzrr_vec=zeros(N,1);      
exitflag_vec=zeros(N,1);
mu = zeros(N,1);

tentativo = [ 0.91 , 1.01 , 0.92 , 1.02 , 0.93 , 0.103 , 0.94 , 0.104 , 0.95 , 1.05 , 0.96 , 1.06 , 0.97 , 0.107 , 0.98 , 1.1 ];


x0 = [0.01 0.01 0.002 0.002 0.002 0.002];   %  initial parameters for [ v , omega , αFl , αFr , αRl , αRr ]
options = optimoptions('fsolve','TolFun',1e-15,'TolX',1e-12,'MaxFunEvals',40000000); % specify options for fsolve : FunctionTolerance , StepTolerance , MaxFunctionEvaluations

for ik=1:N
    U=V(ik);
    
    Vehicle.ik=ik;

    [deltal,deltar]=Ackermann(deg2rad(deltaf_vec(ik)),Vehicle);
    
    fun = @(x)F_Doubletrack_solve_ss(x,U,Vehicle,deltal,deltar,Tyre,choice_model);
    [x,fval,exitflag,output] = fsolve(fun,x0,options);                                % solve non linear system

    j=1;
    while exitflag <= 0 && j <= length(tentativo)
        x0 = x.*tentativo(j);
        j=j+1;
        [x,fval,exitflag,output] = fsolve(fun,x0,options);
    end
    
    if exitflag > 0
       x0 = x;
    else
       x0 = [0.01 0.01 0.002 0.002 0.002 0.002];   
    end
    
    exitflag_vec(ik)=exitflag;

    v_vec(ik)=x(1);                                      % lateral speed vector
    r_vec(ik)=x(2);                                      % yaw rate vector

    alfafl_vec(ik)=x(3);                                 % slip angle front left vector
    alfafr_vec(ik)=x(4);                                 % slip angle front right vector
    alfarl_vec(ik)=x(5);                                 % slip angle rear left vector
    alfarr_vec(ik)=x(6);                                 % slip angle rear right vector


    switch choice_model
        case 3
            Fyfl= x(3) * (Tyre.CSf/2);
            Fyfr= x(4) * (Tyre.CSf/2);
            Fyrl= x(5) * (Tyre.CSr/2);
            Fyrr= x(6) * (Tyre.CSr/2);
            
            ay = U * x(2);
            Fzfl = Vehicle.m * 9.81 * Vehicle.b / (2*Vehicle.L) + Vehicle.m * ay * Vehicle.h *    Vehicle.d  / (2*Vehicle.Wf);
            Fzfr = Vehicle.m * 9.81 * Vehicle.b / (2*Vehicle.L) - Vehicle.m * ay * Vehicle.h *    Vehicle.d  / (2*Vehicle.Wf);
            Fzrl = Vehicle.m * 9.81 * Vehicle.a / (2*Vehicle.L) + Vehicle.m * ay * Vehicle.h * (1-Vehicle.d) / (2*Vehicle.Wr);
            Fzrr = Vehicle.m * 9.81 * Vehicle.a / (2*Vehicle.L) - Vehicle.m * ay * Vehicle.h * (1-Vehicle.d) / (2*Vehicle.Wr);

        case 4
  
            ay = U * x(2);           % Lateral accellaration


            camber_fl = Vehicle.camber_fl(ik);
            camber_fr = Vehicle.camber_fr(ik);
            camber_rl = Vehicle.camber_rl(ik);
            camber_rr = Vehicle.camber_rr(ik);

            Fzfl = Vehicle.m * 9.81 * Vehicle.b / (2*Vehicle.L) + Vehicle.m * ay * Vehicle.h *    Vehicle.d  / (2*Vehicle.Wf);   % Radial Force on front left tyre
            Fzfr = Vehicle.m * 9.81 * Vehicle.b / (2*Vehicle.L) - Vehicle.m * ay * Vehicle.h *    Vehicle.d  / (2*Vehicle.Wf);   % Radial Force on front right tyre
            Fzrl = Vehicle.m * 9.81 * Vehicle.a / (2*Vehicle.L) + Vehicle.m * ay * Vehicle.h * (1-Vehicle.d) / (2*Vehicle.Wr);   % Radial Force on rear left tyre
            Fzrr = Vehicle.m * 9.81 * Vehicle.a / (2*Vehicle.L) - Vehicle.m * ay * Vehicle.h * (1-Vehicle.d) / (2*Vehicle.Wr);   % Radial Force on rear right tyre

            outMF_fl = mfeval(Tyre.Params_f , [Fzfl 0 x(3) -camber_fr 0 U] , 211);
            outMF_fr = mfeval(Tyre.Params_f , [Fzfr 0 x(4) +camber_fl 0 U] , 211);
            outMF_rl = mfeval(Tyre.Params_r , [Fzrl 0 x(5) -camber_rr 0 U] , 211);
            outMF_rr = mfeval(Tyre.Params_r , [Fzrr 0 x(6) +camber_rl 0 U] , 211);

            Fyfl = -outMF_fl(:,2);
            Fyfr = -outMF_fr(:,2);
            Fyrl = -outMF_rl(:,2);
            Fyrr = -outMF_rr(:,2);


    end

    Fzfl_vec(ik) = Fzfl;
    Fzfr_vec(ik) = Fzfr;
    Fzrl_vec(ik) = Fzrl;
    Fzrr_vec(ik) = Fzrr;

    Fyfl_vec(ik) = Fyfl;
    Fyfr_vec(ik) = Fyfr;
    Fyrl_vec(ik) = Fyrl;
    Fyrr_vec(ik) = Fyrr;

end

%Solution.delta=deg2rad(deltaf_vec(ik));
% figure
% scatter(exitflag_vec,V.*r_vec./9.81)
% set(gca,'TickLabelInterpreter','latex');
% xlabel('Exitflag',Interpreter='latex',fontsize=12);
% ylabel('$\frac{a_y}{g}$',Interpreter='latex',fontsize=16);


Solution.u = V';
Solution.v = v_vec;
Solution.r = r_vec;
Solution.ay = V' .* r_vec;
Solution.r_u = r_vec./V';

Solution.alfa_fl = alfafl_vec;
Solution.alfa_fr = alfafr_vec;
Solution.alfa_rl = alfarl_vec;
Solution.alfa_rr = alfarr_vec;

Solution.Fy_fl = Fyfl_vec;
Solution.Fy_fr = Fyfr_vec;
Solution.Fy_rl = Fyrl_vec;
Solution.Fy_rr = Fyrr_vec;

Solution.Fz_fl = Fzfl_vec;
Solution.Fz_fr = Fzfr_vec;
Solution.Fz_rl = Fzrl_vec;
Solution.Fz_rr = Fzrr_vec;

Solution.deltaf_vec = deltaf_vec';
Solution.ay_g = Solution.ay./9.81;
Solution.sterzo_din = rad2deg(deg2rad(Solution.deltaf_vec) - ((Vehicle.L.*Solution.ay)./(Solution.u.^2)));
Solution.Grad = [(diff(Solution.sterzo_din)./diff(Solution.ay./9.81));NaN];

end