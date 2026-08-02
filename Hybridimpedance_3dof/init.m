% initial Program  ---  3 DOF planar manipulator
% Task space : X = [x ; y ; phi],  phi = q11+q12+q13 (end-effector orientation)

%kp      = 1000.0;
kg      = 1;
g       = 9.81;

% Link lengths [m]
Link11  = 1.00;
Link12  = 1.00;
Link13  = 1.00;

% Distance joint -> link centre of mass [m]
Link11g = Link11/2.0;
Link12g = Link12/2.0;
Link13g = Link13/2.0;

% Link masses [kg]
Mass11  = 0.50;
Mass12  = 0.50;
Mass13  = 0.50;

% Slender-rod inertia about the centre of mass [kg m^2]
I11     = Mass11*Link11^2/12.0;
I12     = Mass12*Link12^2/12.0;
I13     = Mass13*Link13^2/12.0;

% ----- Environment ------------------------------------------------------
% Wall : vertical plane at x = x_wall. Moved out from -0.5 (the 2 DOF value)
% because a 3 link arm reaches 3 m and was starting almost on top of it.
x_wall   = -1.5;
% Ground : horizontal plane at y = y_ground. No part of the arm may go below
% it; enforced by the penalty contact model in Dynamics.m.
y_ground =  0.0;

%%% straight line
% (90, 0, 0) deg -> arm fully extended along +y,  X0 = [0 ; 3 ; 90 deg]
% NOTE: a fully extended planar arm is a SINGULAR configuration (det J = 0,
% the arm cannot accelerate along its own axis). Two things make this a legal
% starting point: desired.m uses a quintic profile, whose acceleration is zero
% at t = 0, and Dynamics.m inverts the Jacobian by damped least squares.
q11     = 90.0*pi/180.0;
q12     =  0.0*pi/180.0;
q13     =  0.0*pi/180.0;

%%% circle input
%q11     = 90.0*pi/180.0;
%q12     =  0.0*pi/180.0;
%q13     =  0.0*pi/180.0;

Q      = [ q11
           q12
           q13
          ];
dQ     = [ 0.0
           0.0
           0.0
          ];

%%% END %%%
