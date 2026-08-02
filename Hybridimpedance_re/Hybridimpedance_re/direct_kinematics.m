% Dynamics M-file




% Éf?[?^éÊÇËçûÇ›
function OUT = read(IN)
q11     =IN(1);
q12     =IN(2);
dq11    =IN(3);
dq12    =IN(4);
%q       = [ q11
%            q12
%          ];
dQ      = [ dq11
            dq12
          ];

global Link11;   
global Link12;
global Mass11;
global Mass12;
global Link11g;
global Link12g;
global I11;  
global I12;
global g;

% Direct kinematics  %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
x     = Link11*cos(q11)+Link12*cos(q11+q12);
y     = Link11*sin(q11)+Link12*sin(q11+q12);

X       = [ x
            y ];
% Jacobian J(q) %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
J11=(-1).*Link11.*sin(q11)+(-1).*Link12.*sin(q11+q12);
J12=(-1).*Link12.*sin(q11+q12);
J21=Link11.*cos(q11)+Link12.*cos(q11+q12);
J22=Link12.*cos(q11+q12);

J    = [ J11 J12
         J21 J22
        ];
    
dX   = J*dQ;

% OUTPUT 
OUT = [ X       % 2X1 matrix
        dX      % 2X1 matrix
      ];   
        
        % Number of output = 4

% End