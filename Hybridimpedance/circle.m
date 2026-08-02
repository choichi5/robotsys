function [OUT] = Time(IN)

now = IN(1);

center=[-0.5 1]; 
r=0.5;
T=4;
omega=2*pi*(3*(now/T)^2-2*(now/T)^3);
vomega=2*pi*(6*(now/(T)^2)-6*((now)^2/(T)^3));
aomega=2*pi*((12/(T)^2)-12*((now)/(T)^3));
dx=r*cos(omega)+center(1);
dy=r*(sin(omega))+center(2);
vx=-r*sin(omega)*vomega;
vy=r*cos(omega)*vomega;
ax=(-r*sin(omega)*aomega)-(vomega*cos(omega));
ay=(r*cos(omega)*aomega)+(-vomega*sin(omega));

if now<=4
omega=2*pi*(3*(now/T)^2-2*(now/T)^3);
vomega=2*pi*(6*(now/(T)^2)-6*((now)^2/(T)^3));
aomega=2*pi*((12/(T)^2)-12*((now)/(T)^3));
dx=r*cos(omega)+center(1);
dy=r*(sin(omega))+center(2);
vx=-r*sin(omega)*vomega;
vy=r*cos(omega)*vomega;
ax=(-r*sin(omega)*aomega)-(vomega*cos(omega));
ay=(r*cos(omega)*aomega)+(-vomega*sin(omega));
else
    now=4;
omega=2*pi*(3*(now/T)^2-2*(now/T)^3);
vomega=2*pi*(6*(now/(T)^2)-6*((now)^2/(T)^3));
aomega=2*pi*((12/(T)^2)-12*((now)/(T)^3));
dx=r*cos(omega)+center(1);
dy=r*(sin(omega))+center(2);
vx=-r*sin(omega)*vomega;
vy=r*cos(omega)*vomega;
ax=(-r*sin(omega)*aomega)-(vomega*cos(omega));
ay=(r*cos(omega)*aomega)+(-vomega*sin(omega));
end
%plot(dx, dy);
%hold on
OUT = [dx, dy, vx, vy, ax, ay];