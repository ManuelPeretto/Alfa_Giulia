

function [Alfa,Force,Solution] = ode_solve_singletrack_rampsteer_giulia(Speed,omega_steer,Vehicle,Tyre,tspan, y0,options,choice_linear)
         
[t,Solutions] = ode45(@(t,y) ode_eoq_singletrack_rampsteer_giulia(t,y,Speed,omega_steer,Vehicle,Tyre,choice_linear), tspan, y0,options);

u=Speed;
v=Solutions(:,1);   % Lateral speed
r=Solutions(:,2);   % Yaw-Rate


a=Vehicle.a;
b=Vehicle.b;

delta = omega_steer.*t;

alfa_f = -(v + r.*a)./u + delta;
alfa_r = -(v - r.*b)./u ;

     switch choice_linear
         case 3
           Fyf = Tyre.CSf .* alfa_f;
           Fyr = Tyre.CSr .* alfa_r;
         case 1  
           %inputsMF_f = [Vehicle.Fzf/2 0 alfa_f.' 0 0 u];
           %outMF_f = mfeval(Tyre.Params_f , inputsMF_f , 111);
%
           %inputsMF_r = [Vehicle.Fzr/2 0 alfa_r.' 0 0 u];
           %outMF_r = mfeval(Tyre.Params_r , inputsMF_r , 111);
%
           %Fyf = -2*outMF_f(:,2);                
           %Fyr = -2*outMF_r(:,2);
     end     

Alfa.alfa_f = alfa_f;
Alfa.alfa_r = alfa_r;

Force.Fyf = 0; %Fyf;
Force.Fyr = 0; %Fyr;

Solution.delta = delta;
Solution.u=u;
Solution.v=v;
Solution.r=r;
Solution.t=t; % time

end