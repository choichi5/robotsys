% control Program  ---  3 DOF planar manipulator
% Pass-through block: it rebuilds the model matrices for inspection and
% forwards the full state vector to the Robot Dynamics block.
%
% IN  = [ Xd(3) dXd(3) ddXd(3) X(3) dX(3) Q(3) dQ(3) Intf(1) ]   22 x 1
% OUT = same 22 x 1 vector

function OUT    = read(IN)
Xd       =IN(1:3);
dXd      =IN(4:6);
ddXd     =IN(7:9);
X        =IN(10:12);
dX       =IN(13:15);
Q        =IN(16:18);
dQ       =IN(19:21);
Intf     =IN(22);
q11      =Q(1);
q12      =Q(2);
q13      =Q(3);
dq11     =dQ(1);
dq12     =dQ(2);
dq13     =dQ(3);
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

% Cumulative angles and their rates %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
q1      = q11;
q12s    = q11+q12;
q123    = q11+q12+q13;
w1      = dq11;
w12     = dq11+dq12;
w123    = dq11+dq12+dq13;

% Lumped inertia parameters %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
a1      = I11+Mass11.*Link11g.^2+(Mass12+Mass13).*Link11.^2;
a2      = I12+Mass12.*Link12g.^2+Mass13.*Link12.^2;
a3      = I13+Mass13.*Link13g.^2;
b1      = Link11.*(Mass12.*Link12g+Mass13.*Link12);
b2      = Mass13.*Link12.*Link13g;
b3      = Mass13.*Link11.*Link13g;

c2      = cos(q12);
c3      = cos(q13);
c23     = cos(q12+q13);
s2      = sin(q12);
s3      = sin(q13);
s23     = sin(q12+q13);

% Inertia Matrix [3 X 3]  %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
H11     = a1+a2+a3+2.*b1.*c2+2.*b2.*c3+2.*b3.*c23;
H12     = a2+a3+b1.*c2+2.*b2.*c3+b3.*c23;
H13     = a3+b2.*c3+b3.*c23;
H21     = H12;
H22     = a2+a3+2.*b2.*c3;
H23     = a3+b2.*c3;
H31     = H13;
H32     = H23;
H33     = a3;
Matrix_H    = [ H11 H12 H13
                H21 H22 H23
                H31 H32 H33
              ];
% Nonlinear Matrix [3 X 1] %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
dH1     = zeros(3,3);
dH2     = [ -2.*b1.*s2-2.*b3.*s23   -b1.*s2-b3.*s23   -b3.*s23
            -b1.*s2-b3.*s23          0                 0
            -b3.*s23                 0                 0       ];
dH3     = [ -2.*b2.*s3-2.*b3.*s23   -2.*b2.*s3-b3.*s23  -b2.*s3-b3.*s23
            -2.*b2.*s3-b3.*s23      -2.*b2.*s3          -b2.*s3
            -b2.*s3-b3.*s23         -b2.*s3              0             ];
dH      = cat(3, dH1, dH2, dH3);

Matrix_C = zeros(3,3);
for i = 1:3
    for j = 1:3
        cij = 0;
        for k = 1:3
            cij = cij+0.5*(dH(i,j,k)+dH(i,k,j)-dH(j,k,i))*dQ(k);
        end
        Matrix_C(i,j) = cij;
    end
end
Matrix_S    = Matrix_C*dQ;
% Gravity Matrix [3 X 1] %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
G1      = g.*(Mass11.*Link11g.*cos(q1) ...
             +Mass12.*(Link11.*cos(q1)+Link12g.*cos(q12s)) ...
             +Mass13.*(Link11.*cos(q1)+Link12.*cos(q12s)+Link13g.*cos(q123)));
G2      = g.*(Mass12.*Link12g.*cos(q12s) ...
             +Mass13.*(Link12.*cos(q12s)+Link13g.*cos(q123)));
G3      = g.*Mass13.*Link13g.*cos(q123);
Matrix_G    = [ G1
                G2
                G3
              ];
% Jacobian Matrix_J [3 X 3] %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
J11     = (-1).*Link11.*sin(q1)+(-1).*Link12.*sin(q12s)+(-1).*Link13.*sin(q123);
J12     = (-1).*Link12.*sin(q12s)+(-1).*Link13.*sin(q123);
J13     = (-1).*Link13.*sin(q123);
J21     = Link11.*cos(q1)+Link12.*cos(q12s)+Link13.*cos(q123);
J22     = Link12.*cos(q12s)+Link13.*cos(q123);
J23     = Link13.*cos(q123);
J31     = 1;
J32     = 1;
J33     = 1;
Matrix_J     = [ J11 J12 J13
                 J21 J22 J23
                 J31 J32 J33
               ];
% Dot Jacobian Matrix [3 X 3] %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
dJ11    = (-1).*Link11.*cos(q1).*w1+(-1).*Link12.*cos(q12s).*w12+(-1).*Link13.*cos(q123).*w123;
dJ12    = (-1).*Link12.*cos(q12s).*w12+(-1).*Link13.*cos(q123).*w123;
dJ13    = (-1).*Link13.*cos(q123).*w123;
dJ21    = (-1).*Link11.*sin(q1).*w1+(-1).*Link12.*sin(q12s).*w12+(-1).*Link13.*sin(q123).*w123;
dJ22    = (-1).*Link12.*sin(q12s).*w12+(-1).*Link13.*sin(q123).*w123;
dJ23    = (-1).*Link13.*sin(q123).*w123;
dJ31    = 0;
dJ32    = 0;
dJ33    = 0;
Matrix_dJ    = [ dJ11 dJ12 dJ13
                 dJ21 dJ22 dJ23
                 dJ31 dJ32 dJ33
               ];
% Inverse Jacobian Matrix [3 X 3] %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
Matrix_INVJ    = inv(Matrix_J);
% Proportional Gain Matrix [3 X 3] %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%K_d    = [ 5 0 0
%           0 5 0
%           0 0 5
%           ];
% Derivative Gain Matrix [3 X 3] %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%K_p    = [ 5 0 0
%           0 5 0
%           0 0 5
%           ];
% Input Torque Matrix [3 X 1]
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
