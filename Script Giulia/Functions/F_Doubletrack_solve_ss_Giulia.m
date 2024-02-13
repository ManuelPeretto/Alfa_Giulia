
%---------------------------------------------------------------
%  Define Non-Linear sistem
%---------------------------------------------------------------
function F = F_Doubletrack_solve_ss_Giulia(x,U,Vehicle,deltal,deltar,Tyre)


ay = U * x(2);           % Lateral accellaration

Fzfl = Vehicle.m * 9.81 * Vehicle.b / (2*Vehicle.L) + Vehicle.m * ay * Vehicle.h *    Vehicle.d  / (2*Vehicle.Wf);   % Radial Force on front left tyre
Fzfr = Vehicle.m * 9.81 * Vehicle.b / (2*Vehicle.L) - Vehicle.m * ay * Vehicle.h *    Vehicle.d  / (2*Vehicle.Wf);   % Radial Force on front right tyre
Fzrl = Vehicle.m * 9.81 * Vehicle.a / (2*Vehicle.L) + Vehicle.m * ay * Vehicle.h * (1-Vehicle.d) / (2*Vehicle.Wr);   % Radial Force on rear left tyre
Fzrr = Vehicle.m * 9.81 * Vehicle.a / (2*Vehicle.L) - Vehicle.m * ay * Vehicle.h * (1-Vehicle.d) / (2*Vehicle.Wr);   % Radial Force on rear right tyre

outMF_fl = mfeval(Tyre.Params_f , [Fzfl 0 x(3) 0 0 U] , 111);
outMF_fr = mfeval(Tyre.Params_f , [Fzfr 0 x(4) 0 0 U] , 111);
outMF_rl = mfeval(Tyre.Params_r , [Fzrl 0 x(5) 0 0 U] , 111);
outMF_rr = mfeval(Tyre.Params_r , [Fzrr 0 x(6) 0 0 U] , 111);

Fyfl = -outMF_fl(:,2);   % Lateral Force Front Left
Fyfr = -outMF_fr(:,2);   % Lateral Force Front Right
Fyrl = -outMF_rl(:,2);   % Lateral Force Rear Left
Fyrr = -outMF_rr(:,2);   % Lateral Force Rear Right

F =[x(3) + ( x(1) + x(2) * Vehicle.a) / (U + x(2) * 2 * Vehicle.Wf) - deltal;    % 0 = alfa_fl + (v + omega*a)/(u + omega*Wf) - deltafrontleft ;
    x(4) + ( x(1) + x(2) * Vehicle.a) / (U - x(2) * 2 * Vehicle.Wf) - deltar;    % 0 = alfa_fr + (v + omega*a)/(u - omega*Wf) - deltafrontright ;
    x(5) + ( x(1) - x(2) * Vehicle.b) / (U + x(2) * 2 * Vehicle.Wr);             % 0 = alfa_rl + (v - omega*b)/(u + omega*Wr)
    x(6) + ( x(1) - x(2) * Vehicle.b) / (U - x(2) * 2 * Vehicle.Wr);             % 0 = alfa_rl + (v - omega*b)/(u - omega*Wr)
    (Fyfl + Fyfr + Fyrl + Fyrr) / Vehicle.m - x(2) * U;                          % 0 = (Fyfl+Fyfr+Fyrl+Fyrr) / m - omega*u ;
    (Fyfl + Fyfr) * Vehicle.a - (Fyrl + Fyrr) * Vehicle.b];                      % 0 = ( (Fyfl + Fyfr)*a - (Fyrl + Fyrr)*b ) / J ;

end