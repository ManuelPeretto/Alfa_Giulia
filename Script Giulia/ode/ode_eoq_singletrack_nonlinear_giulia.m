
function dydt = ode_eoq_singletrack_nonlinear_giulia(t,y,delta_v,Vehicle,Tyre,torque)
     
     a=Vehicle.a;
     b=Vehicle.b;
     m=Vehicle.m;
     J=Vehicle.J;
     k=Vehicle.k;
     tau=Vehicle.tau;
     
     delta_f = (1 + k) * tau * delta_v;
     delta_r =  k * tau * delta_v;

     
     u = y(1);  % longitudinal speed
     v = y(2);  % lateral speed
     r = y(3);  % omega

     Fxf = 0;
     

     alfaf = -(v + r*a)/u + delta_f;
     alfar = -(v - r*b)/u + delta_r;

     Fyf=MagicF( alfaf , Tyre.Bf, Tyre.Cf, Tyre.D(Vehicle.Fzf), Tyre.Ef );
     Fyr=MagicF( alfar , Tyre.Br, Tyre.Cr, Tyre.D(Vehicle.Fzr), Tyre.Er );
     

     dydt = zeros(3,1);
     
     dydt(1) = 0; %r .* v + ( Fxf.*cos(delta_f) + Fxr - Fyf.*sin(delta_f)) ./ m;  % u_dot
     
     dydt(2) = -r .* u + ( Fyf + Fyr) ./ m;  % v_dot
     
     dydt(3) = a/J .* Fyf - b/J .* Fyr;   % omega_dot
end