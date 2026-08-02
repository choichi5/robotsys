% control Program
function OUT    = read(IN)
Xd       =IN(1:2);
dXd      =IN(3:4);
ddXd     =IN(5:6);
X        =IN(7:8);
dX       =IN(9:10);
Q        =IN(11:12);
dQ       =IN(13:14);
Intf     =IN(15);
q11      =Q(1);
q12      =Q(2);
dq11     =dQ(1);
dq12     =dQ(2);
global Link11;   
global Link12;
global Mass11;
global Mass12;
global Link11g;
global Link12g;
global I11;  
global I12;  
global g;
% Inetia Matrix [2 X 2]  %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
H11     = I11+I12+Link11g.^2.*Mass11+Link11.^2.*Mass12+Link12g.^2.*Mass12+2.*Link11.*Link12g.*Mass12.*cos(q12) ...
  ;
H12     = I12+Link12g.^2.*Mass12+Link11.*Link12g.*Mass12.*cos(q12);
H21     = I12+Link12g.^2.*Mass12+Link11.*Link12g.*Mass12.*cos(q12);
H22     = I12+Link12g.^2.*Mass12;
Matrix_H    = [ H11 H12
                H21 H22
              ];
% Nonlinear Matrix [2 X 1] %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
S1      = (-2).*dq11.*dq12.*Link11.*Link12g.*Mass12.*sin(q12)+(-1).*dq12.^2.*Link11.*Link12g.*Mass12.*sin(q12) ...
  ;
S2      = dq11.^2.*Link11.*Link12g.*Mass12.*sin(q12);
Matrix_S    = [ S1
                S2
              ];
% Gravity Matrix [2 X 1] %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% 
G1      = g.*Link11g.*Mass11.*cos(q11)+g.*Mass12.*(Link11.*cos(q11)+Link12g.*cos(q11+q12));
G2      = g.*Link12g.*Mass12.*cos(q11+q12);
Matrix_G    = [ G1
                G2
              ];
% Jacobian Matrix_J [2 X 2] %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%        
J11     = (-1).*Link11.*sin(q11)+(-1).*Link12.*sin(q11+q12);
J12     = (-1).*Link12.*sin(q11+q12);
J21     = Link11.*cos(q11)+Link12.*cos(q11+q12);
J22     = Link12.*cos(q11+q12);
Matrix_J     = [ J11 J12
                 J21 J22
               ];
% Dot Jacobian Matrix [2 X 2] %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
dJ11    = (-1).*dq11.*Link11.*cos(q11)+(-1).*(dq11+dq12).*Link12.*cos(q11+q12);
dJ12    = (-1).*(dq11+dq12).*Link12.*cos(q11+q12);
dJ21    = (-1).*dq11.*Link11.*sin(q11)+(-1).*(dq11+dq12).*Link12.*sin(q11+q12);
dJ22    = (-1).*(dq11+dq12).*Link12.*sin(q11+q12);
Matrix_dJ    = [  dJ11 dJ12
                  dJ21 dJ22
               ];
% Inverse Jacobian Matrix [2 X 2] %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
Matrix_INVJ    = inv(Matrix_J);
% Proportional Gain Matrix [2 X 2] %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%K_d    = [ 5 0
%           0 5
%           ];
% Derivative Gain Matrix [2 X 2] %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%  
%K_p    = [ 5 0
%           0 5
%           ];
% Input Torque Matrix [2 X 1]
%Matrix_Torque    = Matrix_H * (Matrix_INVJ*((ddXd+K_d*(dXd-dX)+K_p*(Xd-X))-Matrix_dJ*dQ))+Matrix_S+Matrix_G;
% OUTPUT 
OUT = [ Xd   
        dXd
        ddXd
        X
        dX
        Q
        dQ
        Intf
        ];
% ######  END  #####