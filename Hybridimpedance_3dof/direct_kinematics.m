% Direct kinematics M-file  ---  3 DOF planar manipulator
% IN  = [ q11 q12 q13 dq11 dq12 dq13 ]      (6 x 1)
% OUT = [ X ; dX ]  with X = [x ; y ; phi]  (6 x 1)

function OUT = read(IN)
q11     =IN(1);
q12     =IN(2);
q13     =IN(3);
dq11    =IN(4);
dq12    =IN(5);
dq13    =IN(6);
dQ      = [ dq11
            dq12
            dq13
          ];

global Link11;
global Link12;
global Link13;
global Mass11;
global Mass12;
global Mass13;
global Link11g;
global Link12g;
global Link13g;
global I11;
global I12;
global I13;
global g;

% Cumulative joint angles %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
q1      = q11;
q12s    = q11+q12;
q123    = q11+q12+q13;

% Direct kinematics %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
x     = Link11*cos(q1)+Link12*cos(q12s)+Link13*cos(q123);
y     = Link11*sin(q1)+Link12*sin(q12s)+Link13*sin(q123);
phi   = q123;                       % end-effector orientation

X       = [ x
            y
            phi ];
% Jacobian J(q) [3 X 3] %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
J11=(-1).*Link11.*sin(q1)+(-1).*Link12.*sin(q12s)+(-1).*Link13.*sin(q123);
J12=(-1).*Link12.*sin(q12s)+(-1).*Link13.*sin(q123);
J13=(-1).*Link13.*sin(q123);
J21=Link11.*cos(q1)+Link12.*cos(q12s)+Link13.*cos(q123);
J22=Link12.*cos(q12s)+Link13.*cos(q123);
J23=Link13.*cos(q123);
J31=1;
J32=1;
J33=1;

J    = [ J11 J12 J13
         J21 J22 J23
         J31 J32 J33
        ];

dX   = J*dQ;

% OUTPUT
OUT = [ X       % 3X1 matrix
        dX      % 3X1 matrix
      ];

        % Number of output = 6

% End
