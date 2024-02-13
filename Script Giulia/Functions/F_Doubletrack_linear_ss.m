function [Alfa,Force,Solution,delta] = F_Doubletrack_linear_ss(V,deltaf_vec,Vehicle,Tyre,N)

a=Vehicle.a;
b=Vehicle.b;
m=Vehicle.m;
J=Vehicle.J;
L=Vehicle.L;
Wf=Vehicle.Wf;
Wr=Vehicle.Wr;
h=Vehicle.h;
d=Vehicle.d;
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


x0 = [0.01 0.01 0.2 0.2 0.02 0.02];   %  initial parameters for [ v , omega , αFl , αFr , αRl , αRr ]
options = optimoptions('fsolve','TolFun',1e-15,'TolX',1e-12,'MaxFunEvals',40000000); % specify options for fsolve : FunctionTolerance , StepTolerance , MaxFunctionEvaluations

for ik=1:N
    U=V(ik);

    [deltal,deltar]=Ackermann(deg2rad(deltaf_vec(ik)),deg2rad(toe),percentuale_Ack,Wf,L);
    
    fun = @(x)Doubletrack_solve_steadystate_linear(x,U,Vehicle,deltal,deltar,Tyre);
    [x,fval,exitflag,output] = fsolve(fun,x0,options);  


    v_vec(ik)=x(1);                                      % lateral speed vector
    r_vec(ik)=x(2);                                      % yaw rate vector

    alfafl_vec(ik)=x(3);                                 % slip angle front left vector
    alfafr_vec(ik)=x(4);                                 % slip angle front right vector
    alfarl_vec(ik)=x(5);                                 % slip angle rear left vector
    alfarr_vec(ik)=x(6);                                 % slip angle rear right vector

    Fyfl_vec(ik) = x(3) * (Tyre.CSf/2);  % Lateral Force Front Left vector
    Fyfr_vec(ik) = x(4) * (Tyre.CSf/2);  % Lateral Force Front Right vector
    Fyrl_vec(ik) = x(5) * (Tyre.CSr/2);  % Lateral Force Rear  Left vector
    Fyrr_vec(ik) = x(6) * (Tyre.CSr/2);  % Lateral Force Rear  Right vector
end

delta=deg2rad(deltaf_vec(ik));

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