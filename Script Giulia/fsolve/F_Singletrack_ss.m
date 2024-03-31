
function [Solution] = F_Singletrack_ss(V,deltaf_vec,Vehicle,Tyre,N,choice_model)

alfaf_vec=zeros(1,N);
alfar_vec=zeros(1,N);
Fyf_vec=zeros(1,N);
Fyr_vec=zeros(1,N);
r_vec=zeros(1,N);
v_vec=zeros(1,N);
Solution.exitflag_vec=zeros(1,N);
mu = zeros(1,N);

tentativo = [ 1.02 , 0.98 , 1.05 , 0.94 ];

x0 = [0.02 0.02 0.1 0.01];   %  initial parameters for [ αF , αR , v , omega ]
options = optimoptions('fsolve','TolFun',1e-15,'TolX',1e-12,'MaxFunEvals',40000000); % specify options for fsolve : FunctionTolerance , StepTolerance , MaxFunctionEvaluations

for ik=1:N
    U=V(ik);
    delta=deg2rad(deltaf_vec(ik));
            
    fun = @(x)F_Singletrack_solve_ss(x,U,Vehicle,Tyre,delta,choice_model);
    
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
       x0 = [0.02 0.02 0.1 0.01];   
    end

    Solution.exitflag_vec(ik)=exitflag;

    alfaf_vec(ik)=x(1);                                 % slip angle front vector
    alfar_vec(ik)=x(2);                                 % slip angle rear vector

    v_vec(ik)=x(3);                                     % lateral speed vector
    r_vec(ik)=x(4);                                     % yaw rate vector
    
    switch choice_model
        case 1
            Fyf_vec(ik) = x(1) * Tyre.CSf;
            Fyr_vec(ik) = x(2) * Tyre.CSr;

        case 2

            inputsMF_f = [Vehicle.Fzf/2 0 x(1) 0 0 U];
            outMF_f = mfeval(Tyre.Params_f , inputsMF_f , 111);

            inputsMF_r = [Vehicle.Fzr/2 0 x(2) 0 0 U];
            outMF_r = mfeval(Tyre.Params_r , inputsMF_r , 111);

            Fyf_vec(ik) = -2*outMF_f(:,2);               % lateral force front vector
            Fyr_vec(ik) = -2*outMF_r(:,2);               % lateral force rear vector
    end

end

Solution.u = V;
Solution.v = v_vec;
Solution.r = r_vec;
Solution.r_u = r_vec./V;

Solution.alfaf=alfaf_vec;
Solution.alfar=alfar_vec;

Solution.Fyf = Fyf_vec;
Solution.Fyr = Fyr_vec;

end