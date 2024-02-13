
%---------------------------------------------------------------
%  Define Non-Linear sistem
%---------------------------------------------------------------
function F = F_singletrack_Giulia_solve_steadystate(x,U,Vehicle,Tyre,delta)

m = Vehicle.m;
a = Vehicle.a;
b = Vehicle.b;

inputsMF_f = [Vehicle.Fzf/2 0 x(1) 0 0 U];
outMF_f = mfeval(Tyre.Params_f , inputsMF_f , 111);
Fyf = -2*outMF_f(:,2);

inputsMF_r = [Vehicle.Fzr/2 0 x(2) 0 0 U];
outMF_r = mfeval(Tyre.Params_r , inputsMF_r , 111);
Fyr = -2*outMF_r(:,2);


F = [x(2)+(x(3)-x(4)*b)/U;                   % 0 = alfa_r + (v - omega*b)/u ;
    x(1)+(x(3)+x(4)*a)/U-delta;              % 0 = alfa_f + (v + omega*a)/u - deltafront ;
    (Fyr + Fyf*cos(delta))/m-x(4)*U;         % 0 = (Fyr + Fyf*cos(delta) / m - omega*u ;
    (-Fyr*b + Fyf*a*cos(delta))];            % 0 = (Fyr*b - Fyf*a*cos(delta)) / J ;

end