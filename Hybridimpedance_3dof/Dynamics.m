% Dynamics M-file  ---  3 DOF planar manipulator
%
% Task space  X   = [ x ; y ; phi ]   (phi = q11+q12+q13)
% Joint space Q   = [ q11 ; q12 ; q13 ]
% The Jacobian is square (3x3), so the 2 DOF resolved-acceleration structure
% carries over unchanged: dd_q = J \ (dd_x - dJ*dQ).
%
% Hybrid control split while in contact with the wall at x = -0.5 :
%   x   -> force control   (regulate the contact force to F_d)
%   y   -> position control (slide along the wall)
%   phi -> position control (keep the tool normal to the wall)
%
% IN  = [ Xd(3) dXd(3) ddXd(3) X(3) dX(3) Q(3) dQ(3) Intf(1) ]   22 x 1
% OUT = [ X(3) Xd(3) Q(3) F_ext(3) Intf(1) FF(3) ddQ(3) ]        19 x 1

function OUT = read(IN)
Xd       =IN(1:3);
dXd      =IN(4:6);
ddXd     =IN(7:9);
X        =IN(10:12);
dX       =IN(13:15);
Q        =IN(16:18);
dQ       =IN(19:21);
Intf     =IN(22);

xf        =X(1);          % force-controlled direction (x)
xp        =X(2);          % position-controlled direction (y)
xo        =X(3);          % orientation (phi)
dxf       =dX(1);
dxp       =dX(2);
dxo       =dX(3);
ddxdf     =ddXd(1);
ddxdp     =ddXd(2);
ddxdo     =ddXd(3);
dxdf     =dXd(1);
dxdp     =dXd(2);
dxdo     =dXd(3);
xdf      = Xd(1);
xdp      = Xd(2);
xdo      = Xd(3);

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
b1      = Link11.*(Mass12.*Link12g+Mass13.*Link12);      % couples q11-q12, cos(q12)
b2      = Mass13.*Link12.*Link13g;                       % couples q12-q13, cos(q13)
b3      = Mass13.*Link11.*Link13g;                       % couples q11-q13, cos(q12+q13)

c2      = cos(q12);
c3      = cos(q13);
c23     = cos(q12+q13);
s2      = sin(q12);
s3      = sin(q13);
s23     = sin(q12+q13);

% Inertia Matrix [3 X 3] %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
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
% Nonlinear (Coriolis / centrifugal) Matrix [3 X 1] %%%%%%%%%%%%%%%%%%%%%%
% Built from the Christoffel symbols of the first kind,
%   c_ijk = 1/2 ( dH_ij/dq_k + dH_ik/dq_j - dH_jk/dq_i ),  S_i = sum c_ijk dq_j dq_k
% H does not depend on q11, so dH/dq11 = 0.
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
% Gravity Matrix [3 X 1] %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
G1      = g.*(Mass11.*Link11g.*cos(q1) ...
             +Mass12.*(Link11.*cos(q1)+Link12g.*cos(q12s)) ...
             +Mass13.*(Link11.*cos(q1)+Link12.*cos(q12s)+Link13g.*cos(q123)));
G2      = g.*(Mass12.*Link12g.*cos(q12s) ...
             +Mass13.*(Link12.*cos(q12s)+Link13g.*cos(q123)));
G3      = g.*Mass13.*Link13g.*cos(q123);
Matrix_G    = [ G1
                G2
                G3 ];

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
                 J31 J32 J33 ];

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
                 dJ31 dJ32 dJ33 ];
% Inverse Jacobian Matrix [3 X 3] %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%Matrix_INVJ    = inv(Matrix_J);
% Velocity Gain Matrix [3 X 3] %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% NOTE: gains carried over unchanged from the 2 DOF model. K_d multiplies the
% velocity error and K_p the position error, i.e. the header comments in the
% original file are swapped relative to how the gains are actually used.
K_d    = [ 100 0   0
             0 100 0
             0   0 100 ];
K_df   = 1;
K_dp   = 1;
K_do   = 1;
% Position Gain Matrix [3 X 3] %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
K_p    = [ 1 0 0
           0 1 0
           0 0 1 ];
K_pp   = 100;
K_po   = 100;

K_pf   = 100;
% Mass Matrix [3 X 3] %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
K_m    = [ 1 0 0
           0 1 0
           0 0 1 ];

K_mf   = 1;
K_mp   = 1;
K_mo   = 1;
A_f    = 10;
B_f    = 100;
F_d    = 10;

% % External Force = 0 (Matrix [3 X 1]) %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


% 이거 쓰면 벽이 없어서 일반 ctm 제어
    % F_ext  = [ 0
    %            0
    %            0 ];
    % dd_x   = ddXd+K_m*(K_d*(dXd-dX)+K_p*(Xd-X)-F_ext); %하이브리드 힘 위치제어 속도게인+위치게인 더하기때문
    % Intf = F_d;

%External Force Matrix [3 X 1] %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
if xf<-0.5
    f_ext_x = -((10^6)*(xf+0.5)+(10^2)*dxf); %10^6인 이유는 벽의 딱딱한 강성

    mu = 20;  % 마찰 계수
    f_ext_y = -mu * dxp; %y방향이 0인 이유는 마찰을 안쓰겠다는것
    n_ext_z = 0;         % 점접촉이라 접촉 모멘트는 0으로 둔다
%힘제어 게인 K_mf A_f를 바꿔보기도하고 마찰을 넣어보기도하고 노이즈 난수생성해서 너ㅓㅎ어보기도하고
%
% 실제에서는 채터링 문제가 생길수있다 필터치는거는 2차 제어 문ㄴ제가 겹친다
    dd_xf   = ddxdf+K_mf*(K_df*(dxdf-dxf)-A_f*(F_d-f_ext_x)-B_f*Intf);
    dd_xp   = ddxdp+K_mp*(K_dp*(dxdp-dxp)+K_pp*(xdp-xp));
    dd_xo   = ddxdo+K_mo*(K_do*(dxdo-dxo)+K_po*(xdo-xo));
    dd_x    = [ dd_xf
                dd_xp
                dd_xo ];
    F_ext  = [ f_ext_x
               f_ext_y
               n_ext_z ];
    Intf = F_d-f_ext_x;
else
   F_ext  = [ 0
              0
              0 ];

   dd_x   = ddXd+K_m*(K_d*(dXd-dX)+K_p*(Xd-X)-F_ext);
   Intf = F_d;
end
% 이거 주석풀고 쓰면 벽이 생겨서 외부의 힘이 계측이된다

% Robot Dynamics [3 X 1] %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% dd_x = 0;
dd_q   = Matrix_J\(dd_x-Matrix_dJ*dQ);
Matrix_Torque = Matrix_H*dd_q  +Matrix_S+Matrix_G-Matrix_J'*F_ext ;

FF = Matrix_J'\(Matrix_H*dd_q  +Matrix_S+Matrix_G-Matrix_Torque) ;

% Angular Acceleration [3 X 1] %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
ddQ     = Matrix_H\(Matrix_Torque-Matrix_S-Matrix_G+Matrix_J'*F_ext);

% OUTPUT
OUT     = [ X        % 3X1 matrix
            Xd       % 3X1 matrix
            Q        % 3X1 matrix
            F_ext    % 3X1 matrix
            Intf     % 1X1
            FF       % 3X1 matrix
            ddQ      % 3X1 matrix
        ];         % Number of output = 19
% End
