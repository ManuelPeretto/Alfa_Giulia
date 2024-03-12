

function [Alfa,Force,Solution] = ode_solve_singletrack(delta,Vehicle,Tyre,torque,tspan, y0,options)
         
[t,Solutions] = ode45(@(t,y) ode_eoq_bicycle(t,y,delta,Vehicle,Tyre,torque), tspan, y0,options);

u=Solutions(:,1);   % Longitudinal speed
v=Solutions(:,2);   % Lateral speed
r=Solutions(:,3);   % Yaw-Rate


a=Vehicle.a;
b=Vehicle.b;

Alfa.alfa_f = -(v + r.*a)./u + delta;
Alfa.alfa_r = -(v - r.*b)./u ;

Force.Fyf_vec=MagicF( Alfa.alfa_f , Tyre.Bf, Tyre.Cf, Tyre.D(Vehicle.Fzf), Tyre.Ef );
Force.Fyr_vec=MagicF( Alfa.alfa_r , Tyre.Br, Tyre.Cr, Tyre.D(Vehicle.Fzr), Tyre.Er );

Solution.u=u;
Solution.v=v;
Solution.r=r;
Solution.t=t; % time

end