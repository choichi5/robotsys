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

%%% straight line
% (30, 60, 110) deg  ->  X0 = [-0.074 ; 1.158 ; 200 deg]
% Starts clear of the wall (x0 > -0.5) and 20 deg off normal, so the third
% DOF has to rotate the tool to phi = 180 deg while the arm approaches.
q11     = 30.0*pi/180.0;
q12     = 60.0*pi/180.0;
q13     = 110.0*pi/180.0;

%%% circle input
%q11     = 30.0*pi/180.0;
%q12     = 60.0*pi/180.0;
%q13     = 110.0*pi/180.0;

Q      = [ q11
           q12
           q13
          ];
dQ     = [ 0.0
           0.0
           0.0
          ];

%%% END %%%
