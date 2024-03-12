function [Alfa,Force,Solution,delta] = F_Doubletrack_VIgrade(V,deltaf_vec,Vehicle,Tyre,N,data)

toe=Vehicle.toe;
percentuale_Ack=Vehicle.percentuale_Ack;


alfafl_vec=zeros(1,N);
alfafr_vec=zeros(1,N);
alfarl_vec=zeros(1,N);
alfarr_vec=zeros(1,N);
Fyfl_vec=zeros(1,N);
Fyfr_vec=zeros(1,N);
Fyrl_vec=zeros(1,N);
Fyrr_vec=zeros(1,N);
r_vec=zeros(1,N);
v_vec=zeros(1,N);
Fzfl_vec=zeros(1,N);
Fzfr_vec=zeros(1,N);
Fzrl_vec=zeros(1,N);
Fzrr_vec=zeros(1,N);      
exitflag_vec=zeros(1,N);
mu = zeros(1,N);

tentativo = [ 0.91 , 1.01 , 0.92 , 1.02 , 0.93 , 0.103 , 0.94 , 0.104 , 0.95 , 1.05 , 0.96 , 1.06 , 0.97 , 0.107 , 0.98 , 1.1 ];


x0 = [0.01 0.01 0.002 0.002 0.002 0.002];   %  initial parameters for [ v , omega , αFl , αFr , αRl , αRr ]
options = optimoptions('fsolve','TolFun',1e-15,'TolX',1e-12,'MaxFunEvals',40000000); % specify options for fsolve : FunctionTolerance , StepTolerance , MaxFunctionEvaluations

for ik=1:N
    U=V(ik);

    [deltal,deltar]=Ackermann(deg2rad(deltaf_vec(ik)),deg2rad(toe),percentuale_Ack,Vehicle.Wf,Vehicle.L);
    
    fun = @(x)F_Doubletrack_solve_VIgrade(x,U,Vehicle,deltal,deltar,Tyre,data,ik);
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

    ay = U * x(2);

    Fzfl = data.Tire_Ground_Surface_Force_Z_L1(ik); %Radial Force on front left tyre
    Fzfr = data.Tire_Ground_Surface_Force_Z_R1(ik); %Radial Force on front right tyre
    Fzrl = data.Tire_Ground_Surface_Force_Z_L2(ik); %Radial Force on rear left tyre
    Fzrr = data.Tire_Ground_Surface_Force_Z_R2(ik); %Radial Force on rear right tyre

    outMF_fl = mfeval(Tyre.Params_f , [Fzfl 0 x(3) 0 0 U] , 211);
    outMF_fr = mfeval(Tyre.Params_f , [Fzfr 0 x(4) 0 0 U] , 211);
    outMF_rl = mfeval(Tyre.Params_r , [Fzrl 0 x(5) 0 0 U] , 211);
    outMF_rr = mfeval(Tyre.Params_r , [Fzrr 0 x(6) 0 0 U] , 211);

    Fyfl = -outMF_fl(:,2);   % Lateral Force Front Left
    Fyfr = -outMF_fr(:,2);   % Lateral Force Front Right
    Fyrl = -outMF_rl(:,2);   % Lateral Force Rear Left
    Fyrr = -outMF_rr(:,2);   % Lateral Force Rear Right

    Fzfl_vec(ik) = Fzfl;
    Fzfr_vec(ik) = Fzfr;
    Fzrl_vec(ik) = Fzrl;
    Fzrr_vec(ik) = Fzrr;

    Fyfl_vec(ik) = Fyfl;
    Fyfr_vec(ik) = Fyfr;
    Fyrl_vec(ik) = Fyrl;
    Fyrr_vec(ik) = Fyrr;

end

delta=deg2rad(deltaf_vec(ik));
% figure
% scatter(exitflag_vec,V.*r_vec./9.81)
% set(gca,'TickLabelInterpreter','latex');
% xlabel('Exitflag',Interpreter='latex',fontsize=12);
% ylabel('$\frac{a_y}{g}$',Interpreter='latex',fontsize=16);


Solution.u = V;
Solution.v = v_vec;
Solution.r = r_vec;
Solution.r_u = r_vec./V;

Alfa.alfafl = alfafl_vec;
Alfa.alfafr = alfafr_vec;
Alfa.alfarl = alfarl_vec;
Alfa.alfarr = alfarr_vec;

Force.Fyfl = Fyfl_vec;
Force.Fyfr = Fyfr_vec;
Force.Fyrl = Fyrl_vec;
Force.Fyrr = Fyrr_vec;

Force.Fzfl = Fzfl_vec;
Force.Fzfr = Fzfr_vec;
Force.Fzrl = Fzrl_vec;
Force.Fzrr = Fzrr_vec;

end