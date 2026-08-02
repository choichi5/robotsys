% Desired task-space trajectory  ---  3 DOF planar manipulator
% Cubic (3rd order) polynomial in x and y, constant orientation phi = pi
% so that the end-effector stays normal to the wall at x = -0.5.
%
% IN  = t                                       (1 x 1)
% OUT = [xd yd phid  vx vy vphi  ax ay aphi]    (1 x 9)

function [OUT] = read(IN)

now = IN(1);

% Initial pose from init.m : q = (30, 60, 110) deg
q1_0    = 30.0*pi/180.0;
q12_0   = 30.0*pi/180.0+60.0*pi/180.0;
q123_0  = 30.0*pi/180.0+60.0*pi/180.0+110.0*pi/180.0;

x_0     = cos(q1_0)+cos(q12_0)+cos(q123_0);     % -0.0737
y_0     = sin(q1_0)+sin(q12_0)+sin(q123_0);     %  1.1580
phi_0   = q123_0;                               %  200 deg

% Final pose : 0.5 m into the wall (x = -0.5), slid down along it,
% tool rotated to phi = 180 deg so it ends up normal to the wall.
x_f     = -1.0;
y_f     =  0.5;
phi_f   = pi;

T_final = 2;

if now<=T_final
    tau = now;
    % Cubic blending polynomial and its derivatives
    s   = -2*(tau^3/T_final^3)+3*(tau^2/T_final^2);
    ds  = -6*(tau^2)/(T_final^3)+6*(tau/(T_final^2));
    dds = -12*tau/(T_final^3)+6/(T_final^2);
else
    % Hold the final pose. NOTE: unlike the 2 DOF desired.m, the velocity and
    % acceleration feed-forward are forced to zero here. Evaluating the cubic
    % at tau = T_final leaves dds = -1.5 ~= 0, i.e. a constant acceleration
    % command that never stops acting once the motion is finished.
    s   = 1;
    ds  = 0;
    dds = 0;
end

dx   = (x_f-x_0)*s+x_0;
dy   = (y_f-y_0)*s+y_0;
dphi = (phi_f-phi_0)*s+phi_0;

vx    = (x_f-x_0)*ds;
vy    = (y_f-y_0)*ds;
vphi  = (phi_f-phi_0)*ds;

ax    = (x_f-x_0)*dds;
ay    = (y_f-y_0)*dds;
aphi  = (phi_f-phi_0)*dds;

OUT = [dx, dy, dphi, vx, vy, vphi, ax, ay, aphi];
