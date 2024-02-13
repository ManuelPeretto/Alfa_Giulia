

function F = Doubletrack_solve_steadystate_linear(x,U,Vehicle,deltal,deltar,Tyre)                                                

a=Vehicle.a;
b=Vehicle.b;
m=Vehicle.m;
J=Vehicle.J;
L=Vehicle.L;
Wf=Vehicle.Wf;
Wr=Vehicle.Wr;


Fyfl= x(3) * (Tyre.CSf/2);  % Lateral Force Front Left
Fyfr= x(4) * (Tyre.CSf/2);  % Lateral Force Front Right
Fyrl= x(5) * (Tyre.CSr/2);  % Lateral Force Rear Left
Fyrr= x(6) * (Tyre.CSr/2);  % Lateral Force Rear Right


F =[x(3)+(x(1)+x(2)*a)/(U+x(2)*(Wf/2))-deltal;                    % 0 = alfa_fl + (v + omega*a)/(u + omega*Wf) - deltafrontleft - toe ;
    x(4)+(x(1)+x(2)*a)/(U-x(2)*(Wf/2))-deltar;                    % 0 = alfa_fr + (v + omega*a)/(u - omega*Wf) - deltafrontright + toe ;
    x(5)+(x(1)-x(2)*b)/(U+x(2)*(Wr/2));                           % 0 = alfa_rl + (v - omega*b)/(u + omega*Wr)
    x(6)+(x(1)-x(2)*b)/(U-x(2)*(Wr/2));                           % 0 = alfa_rr + (v - omega*b)/(u - omega*Wr)
    (Fyfl+Fyfr+Fyrl+Fyrr)/m-x(2)*U;                               % 0 = (Fyfl+Fyfr+Fyrl+Fyrr) / m - omega*u ;
    (Fyfl+Fyfr)*a-(Fyrl+Fyrr)*b];                                 % 0 = ( (Fyfl + Fyfr)*a - (Fyrl + Fyrr)*b ) / J ;


end