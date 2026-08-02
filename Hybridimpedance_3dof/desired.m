% Desired task-space trajectory  ---  3 DOF planar manipulator
% Quintic (5th order) polynomial in x, y and phi.
%
% The profile is quintic rather than cubic because the arm starts fully
% extended, which is a singular configuration. A quintic has zero velocity AND
% zero acceleration at t = 0, so no task-space acceleration is demanded while
% the Jacobian is still rank deficient.
%
% IN  = t                                       (1 x 1)
% OUT = [xd yd phid  vx vy vphi  ax ay aphi]    (1 x 9)

function [OUT] = read(IN)

global x_wall;

now = IN(1);

% Initial pose from init.m : q = (90, 0, 0) deg, arm straight up
q1_0    = 90.0*pi/180.0;
q12_0   = 90.0*pi/180.0;
q123_0  = 90.0*pi/180.0;

x_0     = cos(q1_0)+cos(q12_0)+cos(q123_0);     %  0.0
y_0     = sin(q1_0)+sin(q12_0)+sin(q123_0);     %  3.0
phi_0   = q123_0;                               %  90 deg

% Final pose : commanded 0.5 m past the wall so the force loop has something
% to push against, slid down to y = 1.5, tool rotated normal to the wall.
x_f     = x_wall-0.5;
y_f     =  1.5;
phi_f   = pi;

T_final = 2;

if now<=T_final
    tau = now/T_final;
    % Quintic blending polynomial and its derivatives
    s   = 10*tau^3-15*tau^4+6*tau^5;
    ds  = (30*tau^2-60*tau^3+30*tau^4)/T_final;
    dds = (60*tau-180*tau^2+120*tau^3)/T_final^2;
else
    % Hold the final pose with zero velocity / acceleration feed-forward.
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
