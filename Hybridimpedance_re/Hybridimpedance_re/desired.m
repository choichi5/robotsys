function [OUT] = read(IN)

now = IN(1);
x_0     = cos(45.0*pi/180.0)+cos(45.0*pi/180.0+90*pi/180.0);
y_0     = sin(45.0*pi/180.0)+sin(45.0*pi/180.0+90*pi/180.0);
T_final = 2; 

if now<=2
dx=(-1-x_0)*(-2*(now^3/T_final^3)+3*(now^2/T_final^2))+x_0;
dy=(-y_0)*(-2*(now^3/T_final^3)+3*(now^2/T_final^2))+y_0;
vx=(-1-x_0)*(-6*(now^2)/(T_final^3)+6*(now/(T_final^2)));
vy=(-y_0)*(-6*(now^2)/(T_final^3)+6*(now/(T_final^2)));
ax=(-1-x_0)*(-12*now/(T_final^3)+6/(T_final^2));
ay=(-y_0)*(-12*now/(T_final^3)+6/(T_final^2));
else
    now=2;
dx=(-1-x_0)*(-2*(now^3/T_final^3)+3*(now^2/T_final^2))+x_0;
dy=(-y_0)*(-2*(now^3/T_final^3)+3*(now^2/T_final^2))+y_0;
vx=(-1-x_0)*(-6*(now^2)/(T_final^3)+6*(now/(T_final^2)));
vy=(-y_0)*(-6*(now^2)/(T_final^3)+6*(now/(T_final^2)));
ax=(-1-x_0)*(-12*now/(T_final^3)+6/(T_final^2));
ay=(-y_0)*(-12*now/(T_final^3)+6/(T_final^2));
end
    
OUT = [dx, dy, vx, vy, ax, ay];