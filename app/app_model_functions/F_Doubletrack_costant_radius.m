
function [Solution] = F_Doubletrack_costant_radius_app(V,R_vec,Vehicle,Tyre,N,choice_model)

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
deltaf_vec=zeros(1,N);

tentativo = [ 0.91 , 1.01 , 0.92 , 1.02 , 0.93 , 0.103 , 0.94 , 0.104 , 0.95 , 1.05 , 0.96 , 1.06 , 0.97 , 0.107 , 0.98 , 1.1 ];


x0 = [0.01 0.01 0.002 0.002 0.002 0.002];   %  initial parameters for [ v , omega , αFl , αFr , αRl , αRr ]
options = optimoptions('fsolve','TolFun',1e-15,'TolX',1e-12,'MaxFunEvals',40000000); % specify options for fsolve : FunctionTolerance , StepTolerance , MaxFunctionEvaluations
for ik=1:N
    U=V(ik);
    R = R_vec(ik);
            
    fun = @(x)F_Doubletrack_solve_costant_radius_app(x,U,Vehicle,Tyre,R,choice_model);
    
    [x,fval,exitflag,output] = fsolve(fun,x0,options);                         % solve non linear system
    

    j=1;
    while exitflag <= 0 && j <= length(tentativo)
        x0 = x.*tentativo(j);
        j=j+1;
        [x,fval,exitflag,output] = fsolve(fun,x0,options);
    end
    
    if exitflag > 0
       x0=x;
    else
       x0 = [0.01 0.01 0.002 0.002 0.002 0.002];   
    end

    Solution.exitflag_vec(ik)=exitflag;

    v_vec(ik)=x(1);                 % lateral speed vector
    r_vec(ik)= U / R;               % yaw rate vector
    deltaf_vec(ik)=x(2);
    

    alfafl_vec(ik)=x(3);
    alfafr_vec(ik)=x(4);
    alfarl_vec(ik)=x(5);
    alfarr_vec(ik)=x(6);

    ay = U * r_vec(ik);

    switch choice_model
        case 3
            Fyfl= x(3) * (Tyre.CSf/2);
            Fyfr= x(4) * (Tyre.CSf/2);
            Fyrl= x(5) * (Tyre.CSr/2);
            Fyrr= x(6) * (Tyre.CSr/2);

        case 4
            Fzfl = Vehicle.m * 9.81 * Vehicle.b / (2*Vehicle.L) + Vehicle.m * ay * Vehicle.h *    Vehicle.d  / (2*Vehicle.Wf);   % Radial Force on front left tyre
            Fzfr = Vehicle.m * 9.81 * Vehicle.b / (2*Vehicle.L) - Vehicle.m * ay * Vehicle.h *    Vehicle.d  / (2*Vehicle.Wf);   % Radial Force on front right tyre
            Fzrl = Vehicle.m * 9.81 * Vehicle.a / (2*Vehicle.L) + Vehicle.m * ay * Vehicle.h * (1-Vehicle.d) / (2*Vehicle.Wr);   % Radial Force on rear left tyre
            Fzrr = Vehicle.m * 9.81 * Vehicle.a / (2*Vehicle.L) - Vehicle.m * ay * Vehicle.h * (1-Vehicle.d) / (2*Vehicle.Wr);   % Radial Force on rear right tyre

            outMF_fl = mfeval(Tyre.Params_f , [Fzfl 0 x(3) 0 0 U] , 211);
            outMF_fr = mfeval(Tyre.Params_f , [Fzfr 0 x(4) 0 0 U] , 211);
            outMF_rl = mfeval(Tyre.Params_r , [Fzrl 0 x(5) 0 0 U] , 211);
            outMF_rr = mfeval(Tyre.Params_r , [Fzrr 0 x(6) 0 0 U] , 211);

            Fyfl = -outMF_fl(:,2);
            Fyfr = -outMF_fr(:,2);
            Fyrl = -outMF_rl(:,2);
            Fyrr = -outMF_rr(:,2);

            Fzfl_vec(ik) = Fzfl;
            Fzfr_vec(ik) = Fzfr;
            Fzrl_vec(ik) = Fzrl;
            Fzrr_vec(ik) = Fzrr;
    end

    Fyfl_vec(ik) = Fyfl;
    Fyfr_vec(ik) = Fyfr;
    Fyrl_vec(ik) = Fyrl;
    Fyrr_vec(ik) = Fyrr;
end

Solution.u = V;
Solution.v = v_vec;
Solution.r = r_vec;
Solution.r_u = r_vec./V;
Solution.ay = V * r_vec;
Solution.deltaf_vec=deltaf_vec;

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

end