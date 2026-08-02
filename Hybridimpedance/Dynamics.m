% Dynamics M-file

function OUT = read(IN)
%Matrix_Torque =IN(1:2);
%Matrix_H = IN(2:6);
%Matrix_S = IN(7:8);
%Matrix_G = IN(9:10);
Xd       =IN(1:2);
dXd      =IN(3:4);
ddXd     =IN(5:6);
X        =IN(7:8);
dX       =IN(9:10);
Q        =IN(11:12);
dQ       =IN(13:14);
Intf     =IN(15);
xf        =X(1);
xp        =X(2);
dxf       =dX(1);
dxp       =dX(2);
ddxdf     =ddXd(1);
ddxdp     =ddXd(2);
dxdf     =dXd(1);
dxdp     =dXd(2);
xdf      = Xd(1);
xdp      = Xd(2);

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
% Gravity Matrix_DJ [2 X 1] %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% 
G1      = g.*Link11g.*Mass11.*cos(q11)+g.*Mass12.*(Link11.*cos(q11)+Link12g.*cos(q11+q12));
G2      = g.*Link12g.*Mass12.*cos(q11+q12);
Matrix_G    = [ G1
                G2
              ];
% Jacobian Matrix_DJ [2 X 2] %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%        
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
%Matrix_INVJ    = inv(Matrix_J);
% Proportional Gain Matrix [2 X 2] %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
K_d    = [ 1 0
           0 1
           ];
K_df   = 1;
K_dp   = 1;
% Derivative Gain Matrix [2 X 2] %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%  
K_p    = [ 100 0
           0 100
           ];
K_pp   = 100;
K_pf   = 100;
% Mass Matrix [2 X 2] %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%  
K_m    = [ 1 0
           0 1
           ];
K_mf   = 1;
K_mp   = 1;
A_f    = 10;
B_f    = 100;
F_d    = 10;
% External Force Matrix [2 X 1] %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% 
if xf<-0.5
    f_ext_x = -((10^6)*(xf+0.5)+(10^2)*dxf);
    f_ext_y = 0;
    %F_ext  = [ f_ext_x
    %           f_ext_y
    %           ];
    %dd_xf   = ddxdf+K_mf*(K_df*(dxdf-dxf)+K_pf*(xdf-xf)-f_ext_x);
    dd_xf   = ddxdf+K_mf*(K_df*(dxdf-dxf)-A_f*(F_d-f_ext_x)-B_f*Intf);
    dd_xp   = ddxdp+K_mp*(K_dp*(dxdp-dxp)+K_pp*(xdp-xp));
    dd_x    = [ dd_xf
                dd_xp
                ];
    F_ext  = [ f_ext_x
               f_ext_y  
           ];
    Intf = F_d-f_ext_x;
else
   F_ext  = [ 0
             0  
           ];
   ddXd   = [ ddxdf
              ddxdp
              ];
   dXd    = [ dxdf
              dxdp
              ];
   dX     = [ dxf
              dxp
              ];
   Xd     = [ xdf
              xdp
              ];
   X      = [ xf
              xp
              ];
   dd_x   = ddXd+K_m*(K_d*(dXd-dX)+K_p*(Xd-X)-F_ext);
   Intf = F_d;
end
% Robot Dynamics [2 X 1] %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

dd_q   = Matrix_J\(dd_x-Matrix_dJ*dQ);
Matrix_Torque = Matrix_H*dd_q  +Matrix_S+Matrix_G-Matrix_J'*F_ext ;
FF = Matrix_J'\(Matrix_H*dd_q  +Matrix_S+Matrix_G-Matrix_Torque) ;

% Angular Acceleration [2 X 1] %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
ddQ     = Matrix_H\(Matrix_Torque-Matrix_S-Matrix_G+Matrix_J'*F_ext);

% OUTPUT 
OUT     = [ X        % 2X1 matrix
            Xd       % 2X1 matix
            Q        % 2X1 matrix
            F_ext    % 2X1 matrix
            Intf
            FF
            ddQ      % 2X1 matrix            
        ];         % Number of output = 10
% End