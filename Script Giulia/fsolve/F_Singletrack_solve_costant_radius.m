
%---------------------------------------------------------------
%  Define Non-Linear sistem
%---------------------------------------------------------------
function F = F_Singletrack_solve_costant_radius(x,U,Vehicle,Tyre,R,choice_model)

m = Vehicle.m;
a = Vehicle.a;
b = Vehicle.b;

switch choice_model
    case 1
        Fyf= x(1) * Tyre.CSf;
        Fyr= x(2) * Tyre.CSr;

    case 2    
        inputsMF_f = [Vehicle.Fzf/2 0 x(1) 0 0 U];
        outMF_f = mfeval(Tyre.Params_f , inputsMF_f , 111);
        Fyf = -2*outMF_f(:,2);

        inputsMF_r = [Vehicle.Fzr/2 0 x(2) 0 0 U];
        outMF_r = mfeval(Tyre.Params_r , inputsMF_r , 111);
        Fyr = -2*outMF_r(:,2);
end

omega = U / R;

switch Vehicle.choice_approx
    case 1
        F = [x(2)+(x(3)-omega*b)/U;                   % 0 = alfa_r + (v - omega*b)/u ;
            x(1)+(x(3)+omega*a)/U-x(4);               % 0 = alfa_f + (v + omega*a)/u - deltafront 
            (Fyr + Fyf)/m-omega*U;
            (-Fyr*b + Fyf*a)];
    case 2
        F = [x(2)+(x(3)-omega*b)/U;                   % 0 = alfa_r + (v - omega*b)/u ;
            x(1)+(x(3)+omega*a)/U-x(4);               % 0 = alfa_f + (v + omega*a)/u - deltafront ;
            (Fyr + Fyf*cos(x(4)))/m-omega*U;         % 0 = (Fyr + Fyf*cos(delta) / m - omega*u ;
            (-Fyr*b + Fyf*a*cos(x(4)))];             % 0 = (Fyr*b - Fyf*a*cos(delta)) / J ;
end       
end