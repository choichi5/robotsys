% Circular task-space trajectory  ---  3 DOF planar manipulator
% Constant orientation phi = pi (tool kept normal to the wall).
%
% IN  = t                                       (1 x 1)
% OUT = [xd yd phid  vx vy vphi  ax ay aphi]    (1 x 9)

function [OUT] = Time(IN)

global x_wall;

now = IN(1);

center=[x_wall 1.5];
r=0.5;
T=4;
phi_d = pi;

if now<=T
    tau = now;
    omega =2*pi*(3*(tau/T)^2-2*(tau/T)^3);
    vomega=2*pi*(6*(tau/(T)^2)-6*((tau)^2/(T)^3));
    aomega=2*pi*((12/(T)^2)-12*((tau)/(T)^3));
else
    tau = T;
    omega =2*pi*(3*(tau/T)^2-2*(tau/T)^3);
    vomega=0;
    aomega=0;
end

dx=r*cos(omega)+center(1);
dy=r*(sin(omega))+center(2);
vx=-r*sin(omega)*vomega;
vy=r*cos(omega)*vomega;
ax=(-r*sin(omega)*aomega)-(r*cos(omega)*vomega^2);
ay=(r*cos(omega)*aomega)-(r*sin(omega)*vomega^2);

%plot(dx, dy);
%hold on
OUT = [dx, dy, phi_d, vx, vy, 0, ax, ay, 0];
