
%---------------------------------------------------------------
%  Define Non-Linear sistem
%---------------------------------------------------------------
function F = F_Doubletrack_solve_VIgrade(x,U,Vehicle,deltal,deltar,Tyre,data,ik)


ay = U * x(2);           % Lateral accellaration

Fzfl = data.Tire_Ground_Surface_Force_Z_L1(ik);
Fzfr = data.Tire_Ground_Surface_Force_Z_R1(ik);
Fzrl = data.Tire_Ground_Surface_Force_Z_L2(ik);
Fzrr = data.Tire_Ground_Surface_Force_Z_R2(ik);

outMF_fl = mfeval(Tyre.Params_f , [Fzfl 0 x(3) 0 0 U] , 211);
outMF_fr = mfeval(Tyre.Params_f , [Fzfr 0 x(4) 0 0 U] , 211);
outMF_rl = mfeval(Tyre.Params_r , [Fzrl 0 x(5) 0 0 U] , 211);
outMF_rr = mfeval(Tyre.Params_r , [Fzrr 0 x(6) 0 0 U] , 211);

Fyfl = -outMF_fl(:,2);   % Lateral Force Front Left
Fyfr = -outMF_fr(:,2);   % Lateral Force Front Right
Fyrl = -outMF_rl(:,2);   % Lateral Force Rear Left
Fyrr = -outMF_rr(:,2);   % Lateral Force Rear Right
 
F =[x(3) + ( x(1) + x(2) * Vehicle.a) / (U) - deltal;                            % 0 = alfa_fl + (v + omega*a)/(u + omega*Wf) - deltafrontleft ;
    x(4) + ( x(1) + x(2) * Vehicle.a) / (U) - deltar;                            % 0 = alfa_fr + (v + omega*a)/(u - omega*Wf) - deltafrontright ;
    x(5) + ( x(1) - x(2) * Vehicle.b) / (U);                                     % 0 = alfa_rl + (v - omega*b)/(u + omega*Wr)
    x(6) + ( x(1) - x(2) * Vehicle.b) / (U);                                     % 0 = alfa_rl + (v - omega*b)/(u - omega*Wr)
    (Fyfl + Fyfr + Fyrl + Fyrr) / Vehicle.m - x(2) * U;                          % 0 = (Fyfl+Fyfr+Fyrl+Fyrr) / m - omega*u ;
    (Fyfl + Fyfr) * Vehicle.a - (Fyrl + Fyrr) * Vehicle.b];                      % 0 = ( (Fyfl + Fyfr)*a - (Fyrl + Fyrr)*b ) / J ;

% F =[x(3) + ( x(1) + x(2) * Vehicle.a) / (U) - deltal;       
%     x(4) + ( x(1) + x(2) * Vehicle.a) / (U) - deltar;       
%     x(5) + ( x(1) - x(2) * Vehicle.b) / (U);                
%     x(6) + ( x(1) - x(2) * Vehicle.b) / (U);                
%     (Fyfl*cos(deltal) + Fyfr*cos(deltar) + Fyrl + Fyrr) / Vehicle.m - x(2) * U;     
%     (Fyfl*cos(deltal) + Fyfr*cos(deltar)) * Vehicle.a - (Fyrl + Fyrr) * Vehicle.b]; 
end

%F =[x(3) + ( x(1) + x(2) * Vehicle.a) / (U + x(2) * (Vehicle.Wf/2)) - deltal;        % 0 = alfa_fl + (v + omega*a)/(u + omega*Wf) - deltafrontleft ;
%    x(4) + ( x(1) + x(2) * Vehicle.a) / (U - x(2) * (Vehicle.Wf/2)) - deltar;        % 0 = alfa_fr + (v + omega*a)/(u - omega*Wf) - deltafrontright ;
%    x(5) + ( x(1) - x(2) * Vehicle.b) / (U + x(2) * (Vehicle.Wr/2));                 % 0 = alfa_rl + (v - omega*b)/(u + omega*Wr)
%    x(6) + ( x(1) - x(2) * Vehicle.b) / (U - x(2) * (Vehicle.Wr/2));                 % 0 = alfa_rl + (v - omega*b)/(u - omega*Wr)



