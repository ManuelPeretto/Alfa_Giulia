
function dydt = ode_eoq_singletrack_rampsteer_giulia(t,y,Speed,omega_steer,Vehicle,Tyre,choice_linear)
     
     a=Vehicle.a;
     b=Vehicle.b;
     m=Vehicle.m;
     J=Vehicle.J;

     tau_f=0.2;
     tau_r=0.2;

     u = Speed;  % longitudinal speed
     
     v = y(1);  % lateral speed
     r = y(2);  % omega
     
     delta = omega_steer.*t;

     Fxf = 0;
     
     alfa_f = -(v + r*a)./u + delta;
     alfa_r = -(v - r*b)./u ;

     switch choice_linear
         case 1
           Fyf = Tyre.CSf .* alfa_f;
           Fyr = Tyre.CSr .* alfa_r;

         case 2  
           inputsMF_f = [Vehicle.Fzf/2 0 alfa_f 0 0 u];
           outMF_f = mfeval(Tyre.Params_f , inputsMF_f , 111);

           inputsMF_r = [Vehicle.Fzr/2 0 alfa_r 0 0 u];
           outMF_r = mfeval(Tyre.Params_r , inputsMF_r , 111);

           Fyf = -2*outMF_f(:,2) .* (1 - exp(-abs(u) / tau_f));                
           Fyr = -2*outMF_r(:,2) .* (1 - exp(-abs(u) / tau_r));
     end     

     dydt = zeros(2,1);
       
     dydt(1) = -r .* u + (( Fyf + Fyr ) ./ m);  % v_dot
     
     dydt(2) = (a .* Fyf - b .* Fyr) ./ J;   % omega_dot
end