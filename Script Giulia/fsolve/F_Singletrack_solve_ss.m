
%---------------------------------------------------------------
%  Define Non-Linear sistem
%---------------------------------------------------------------
function F = F_Singletrack_solve_ss(x,U,Vehicle,Tyre,delta,choice_model)

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


F = [x(2)+(x(3)-x(4)*b)/U;                   % 0 = alfa_r + (v - omega*b)/u ;
     x(1)+(x(3)+x(4)*a)/U-delta;              % 0 = alfa_f + (v + omega*a)/u - deltafront ;
    %(Fyr + Fyf*cos(delta))/m-x(4)*U;        % 0 = (Fyr + Fyf*cos(delta) / m - omega*u ;
    %(-Fyr*b + Fyf*a*cos(delta))];           % 0 = (Fyr*b - Fyf*a*cos(delta)) / J ;
    (Fyr + Fyf)/m-x(4)*U;                    % 0 = (Fyr + Fyf) / m - omega*u ;
    (-Fyr*b + Fyf*a)];                       % 0 = (Fyr*b - Fyf*a) / J ;

end