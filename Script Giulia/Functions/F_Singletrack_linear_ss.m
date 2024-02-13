

function [Alfa,Force,Solution] = F_Singletrack_linear_ss(V,deltaf_vec,Vehicle,Tyre,N)

m = Vehicle.m;
J = Vehicle.J;
a = Vehicle.a;
b = Vehicle.b;
CSf = Tyre.CSf;
CSr = Tyre.CSr;



v_vec=zeros(1,N);         % vector lateral speed Vy
r_vec=zeros(1,N);         % initialize yaw-rate vector
r_u=zeros(1,N);           % initialize r/u vector


for ik=1:N
    U=V(ik);
    delta=deg2rad(deltaf_vec(ik));
    A=[-(CSf+CSr)/(m*U),-(CSf*a-CSr*b)/(m*U)-U;
        -(CSf*a-CSr*b)/(J*U),-(CSf*a^2+CSr*b^2)/(J*U)];

    B=-delta*[CSf/m;a*CSf/J];

    x=A\B;
    v_vec(ik)=x(1);
    r_vec(ik)=x(2);
    r_u(ik)=r_vec(ik)/U;
end


Alfa.alfaf=delta-(v_vec+r_vec.*a)./V;
Alfa.alfar=-(v_vec-r_vec.*b)./V;

Force.Fyf=Alfa.alfaf.*CSf;
Force.Fyr=Alfa.alfar.*CSr;

Solution.u = V;
Solution.v = v_vec;
Solution.r = r_vec;
Solution.r_u = r_u;
