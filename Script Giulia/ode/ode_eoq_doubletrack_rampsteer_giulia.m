function dydt = ode_eoq_doubletrack_rampsteer_giulia(t,y,speed,Vehicle,Tyre,omega_steer,choice_linear)
     
     a=Vehicle.a;
     b=Vehicle.b;
     m=Vehicle.m;
     J=Vehicle.J;
     L=Vehicle.L;
     Wf=Vehicle.Wf;
     Wr=Vehicle.Wr;
     h=Vehicle.h;
     d=Vehicle.d;
     g=9.81;
     toe_r=Vehicle.toe_r;

     delta = omega_steer.*t;

     [deltal,deltar]=Ackermann(delta,Vehicle);
     
     u = speed;  % longitudinal speed
     v = y(1);  % lateral speed
     r = y(2);  % omega
     
     % Define longitudinal Force.
     Fxfl = 0;
     Fxfr = 0;
     Fxrl = 0;
     Fxrr = 0;

     ay = u.*r;
     
     % Calculate slip angle for each tyres.
     alfa_fl = -(v + r*a) ./ u + deltal;
     alfa_fr = -(v + r*a) ./ u + deltar;
     alfa_rl = -(v - r*b) ./ u + deg2rad(toe_r);
     alfa_rr = -(v - r*b) ./ u - deg2rad(toe_r);

     Fzfl = Vehicle.m * 9.81 * Vehicle.b / (2*Vehicle.L) + Vehicle.m * ay * Vehicle.h *    Vehicle.d  / (2*Vehicle.Wf);   % Radial Force on fr
     Fzfr = Vehicle.m * 9.81 * Vehicle.b / (2*Vehicle.L) - Vehicle.m * ay * Vehicle.h *    Vehicle.d  / (2*Vehicle.Wf);   % Radial Force on fr
     Fzrl = Vehicle.m * 9.81 * Vehicle.a / (2*Vehicle.L) + Vehicle.m * ay * Vehicle.h * (1-Vehicle.d) / (2*Vehicle.Wr);   % Radial Force on re
     Fzrr = Vehicle.m * 9.81 * Vehicle.a / (2*Vehicle.L) - Vehicle.m * ay * Vehicle.h * (1-Vehicle.d) / (2*Vehicle.Wr);   % Radial Force on re
     
     switch choice_linear
         
         case 3
             Fyfl = (Tyre.CSf/2) .* alfa_fl;
             Fyfr = (Tyre.CSf/2) .* alfa_fr;
             Fyrl = (Tyre.CSr/2) .* alfa_rl;
             Fyrr = (Tyre.CSr/2) .* alfa_rr;
         case 4      

             Kappa_fl = 0;%Vehicle.Kappa_fl;
             Kappa_fr = 0;%Vehicle.Kappa_fr;
             Kappa_rl = 0;%Vehicle.Kappa_rl;
             Kappa_rr = 0;%Vehicle.Kappa_rr;

             camber_fl = 0;%Vehicle.camber_fl;
             camber_fr = 0;%Vehicle.camber_fr;
             camber_rl = 0;%Vehicle.camber_rl;
             camber_rr = 0;%Vehicle.camber_rr;

             outMF_fl = mfeval(Tyre.Params_f , [Fzfl Kappa_fl alfa_fl camber_fl 0 u] , 111);
             outMF_fr = mfeval(Tyre.Params_f , [Fzfr Kappa_fr alfa_fr camber_fr 0 u] , 111);
             outMF_rl = mfeval(Tyre.Params_r , [Fzrl Kappa_rl alfa_rl camber_rl 0 u] , 111);
             outMF_rr = mfeval(Tyre.Params_r , [Fzrr Kappa_rr alfa_rr camber_rr 0 u] , 111);

             Fyfl = -outMF_fl(:,2);   % Lateral Force Front Left
             Fyfr = -outMF_fr(:,2);   % Lateral Force Front Right
             Fyrl = -outMF_rl(:,2);   % Lateral Force Rear Left
             Fyrr = -outMF_rr(:,2);   % Lateral Force Rear Right
     end

     dydt = zeros(2,1);
     
     % v_dot
     dydt(1) = -r .* u + ( Fyfl + Fyfr + Fyrl + Fyrr + Fxfl + Fxfr) ./ m;
     
     % omega_dot
     dydt(2) = a/J .* (Fyfl + Fyfr + Fxfl + Fxfr)  -  b/J .* (Fyrl+Fyrr);  
end









