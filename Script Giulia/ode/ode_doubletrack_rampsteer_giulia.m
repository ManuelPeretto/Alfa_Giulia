

function [Alfa,Force,Solution] = ode_doubletrack_rampsteer_giulia(speed,Vehicle,Tyre,omega_steer,choice_linear,tspan,y0,options)
         
[t,Solutions] = ode45(@(t,y) ode_eoq_doubletrack_rampsteer_giulia(t,y,speed,Vehicle,Tyre,omega_steer,choice_linear),tspan,y0,options);

u = speed;   % Longitudinal speed
v=Solutions(:,1);   % Lateral speed
r=Solutions(:,2);   % Yaw-Rate


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


% Calculate slip angle for each tyres.
alfa_fl = -(v + r*a) ./ u + deltal;
alfa_fr = -(v + r*a) ./ u + deltar;
alfa_rl = -(v - r*b) ./ u + deg2rad(toe_r);
alfa_rr = -(v - r*b) ./ u - deg2rad(toe_r);

ay = u.*r;

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
        n = numel(alfa_fl);
        %kappa = zeros(n,1);
        %gamma = zeros(n,1);
        phit = zeros (n,1);
        U = zeros(n,1) + u;

        Kappa_fl = Vehicle.Kappa_fl;
        Kappa_fr = Vehicle.Kappa_fr;
        Kappa_rl = Vehicle.Kappa_rl;
        Kappa_rr = Vehicle.Kappa_rr;

        camber_fl = Vehicle.camber_fl;
        camber_fr = Vehicle.camber_fr;
        camber_rl = Vehicle.camber_rl;
        camber_rr = Vehicle.camber_rr;

        outMF_fl = mfeval(Tyre.Params_f , [Fzfl Kappa_fl alfa_fl +camber_fr phit U] , 111);
        outMF_fr = mfeval(Tyre.Params_f , [Fzfr Kappa_fr alfa_fr -camber_fl phit U] , 111);
        outMF_rl = mfeval(Tyre.Params_r , [Fzrl Kappa_rl alfa_rl +camber_rr phit U] , 111);
        outMF_rr = mfeval(Tyre.Params_r , [Fzrr Kappa_rr alfa_rr -camber_rl phit U] , 111);

        Fyfl = -outMF_fl(:,2);   % Lateral Force Front Left
        Fyfr = -outMF_fr(:,2);   % Lateral Force Front Right
        Fyrl = -outMF_rl(:,2);   % Lateral Force Rear Left
        Fyrr = -outMF_rr(:,2);   % Lateral Force Rear Right
end


Force.Fzfl = Fzfl;
Force.Fzfr = Fzfr;
Force.Fzrl = Fzrl;
Force.Fzrr = Fzrr;

Force.Fyfl = Fyfl;
Force.Fyfr = Fyfr;
Force.Fyrl = Fyrl;
Force.Fyrr = Fyrr;

Alfa.alfa_fl = alfa_fl;
Alfa.alfa_fr = alfa_fr;
Alfa.alfa_rl = alfa_rl;
Alfa.alfa_rr = alfa_rr;


Solution.u=u;
Solution.v=v;
Solution.r=r;
Solution.t=t; % time
Solution.delta = delta;

end