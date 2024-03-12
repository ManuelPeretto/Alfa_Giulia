
function dydt = ode_eoq_singletrack_linear_giulia(t,y,delta_v,Vehicle,Tyre,index)
     
     a=Vehicle.a(index);
     b=Vehicle.b(index);
     m=Vehicle.m(index);
     J=Vehicle.J(index);
     k=Vehicle.k(index);
     tau=Vehicle.tau(index);
     
     delta_f = (1 + k) * tau * delta_v;
     delta_r =  k * tau * delta_v;
     
     u = y(1);  % longitudinal speed
     v = y(2);  % lateral speed
     r = y(3);  % omega

     Fxf = 0;
     
     alfaf = -(v + r*a)/u + delta_f;
     alfar = -(v - r*b)/u + delta_r;

     Fyf = Tyre.CSf(index) * alfaf;
     Fyr = Tyre.CSr(index) * alfar;
     

     dydt = zeros(3,1);
     
     dydt(1) = 0;                    % u_dot
     
     dydt(2) = -r .* u + ( Fyf.*cos(delta_f) + Fyr.*cos(delta_r) + Fxf.*sin(delta_f)) ./ m;  % v_dot
     
     dydt(3) = a/J .* (Fyf.*cos(delta_f) + Fxf.*sin(delta_f)) - b/J .* Fyr.*cos(delta_r);   % omega_dot
end