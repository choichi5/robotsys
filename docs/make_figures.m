% make_figures.m --- README 용 그림 생성 스크립트
%
% Hybridimpedance_3dof/data.txt 를 읽어서 docs/img/ 에 PNG / GIF 를 만든다.
% 시뮬레이션을 다시 돌릴 필요는 없다 (data.txt 만 있으면 된다).
%
%   >> cd docs; make_figures
%
% data 열 구성
%   1     t
%   2: 4  x, y, phi        실제 작업공간 자세
%   5: 7  xd, yd, phid     목표 작업공간 자세
%   8:10  q11, q12, q13    [deg]
%  11:13  Fx, Fy, Mz       접촉 렌치

clear; close all;

here    = fileparts(mfilename('fullpath'));
if isempty(here), here = pwd; end
outdir  = fullfile(here,'img');
if ~exist(outdir,'dir'), mkdir(outdir); end

data = load(fullfile(here,'..','Hybridimpedance_3dof','data.txt'));

t    = data(:,1);
x    = data(:,2);   y    = data(:,3);   phi  = data(:,4)*180/pi;
xd   = data(:,5);   yd   = data(:,6);   phid = data(:,7)*180/pi;
q    = data(:,8:10);
Fx   = data(:,11);  Fy   = data(:,12);

Lk       = [1 1 1];
x_wall   = -1.5;
y_ground =  0.0;
F_d      = 10;

% ---- 색 -----------------------------------------------------------------
C.x     = [0.85 0.20 0.35];      % x  : crimson
C.y     = [0.13 0.60 0.72];      % y  : teal
C.phi   = [0.55 0.34 0.85];      % phi: purple
C.des   = [0.55 0.57 0.62];      % 목표값 : gray
C.ink   = [0.13 0.14 0.18];
C.grid  = [0.80 0.82 0.86];
C.acc   = [0.96 0.62 0.11];      % amber
C.link  = [0.96 0.62 0.11
           0.13 0.72 0.68
           0.60 0.36 0.87];
C.jnt   = [0.97 0.98 0.99];
C.base  = [0.24 0.26 0.32];
C.grip  = [0.30 0.33 0.40];
C.env   = [0.62 0.64 0.68];
C.bg    = [1 1 1];

% 접촉 시각
ic      = find(abs(Fx) > 1e-6, 1, 'first');
t_c     = t(ic);
y_c     = y(ic);

fprintf('contact at t = %.2f s,  y = %.4f m\n', t_c, y_c);
fprintf('slide     = %.4f m\n', y_c - y(end));
fprintf('peak Fx   = %.2f N\n', max(Fx));
fprintf('final Fx  = %.4f N,  penetration = %.3e m\n', Fx(end), x_wall-x(end));

set(0,'defaultAxesFontName','Segoe UI');
set(0,'defaultTextFontName','Segoe UI');

%% ======================================================================
%  1. 시간 응답 : 위치 / 자세
%% ======================================================================
f = figure('Color',C.bg,'Position',[80 80 900 620],'Visible','off');

ax1 = subplot(2,1,1); hold(ax1,'on');
area(ax1,[t_c t(end)],[4 4],-1,'FaceColor',C.acc,'FaceAlpha',0.07,'EdgeColor','none','ShowBaseLine','off');
plot(ax1,t,xd,'--','Color',C.des,'LineWidth',1.6);
plot(ax1,t,yd,'--','Color',C.des,'LineWidth',1.6);
plot(ax1,t,x ,'-' ,'Color',C.x  ,'LineWidth',2.4);
plot(ax1,t,y ,'-' ,'Color',C.y  ,'LineWidth',2.4);
yline(ax1,x_wall,':','Color',C.ink,'LineWidth',1.6,'Label','wall  x = -1.5', ...
      'LabelHorizontalAlignment','left','FontSize',10);
xline(ax1,t_c,'-','Color',C.acc,'LineWidth',1.6);
text(ax1,t_c+0.05,2.55,sprintf('contact  t = %.2f s',t_c),'Color',C.acc*0.75, ...
     'FontSize',10,'FontWeight','bold');
legend(ax1,{'','x_d , y_d','','x','y'},'Location','northeast','Box','off','FontSize',10);
ylabel(ax1,'position [m]'); ylim(ax1,[-2.4 3.4]);
title(ax1,'Task-space position','FontSize',13,'FontWeight','bold','Color',C.ink);
grid(ax1,'on'); set(ax1,'GridColor',C.grid,'GridAlpha',0.9,'Box','off','XColor',C.ink,'YColor',C.ink);

ax2 = subplot(2,1,2); hold(ax2,'on');
area(ax2,[t_c t(end)],[200 200],0,'FaceColor',C.acc,'FaceAlpha',0.07,'EdgeColor','none','ShowBaseLine','off');
plot(ax2,t,phid,'--','Color',C.des,'LineWidth',1.6);
plot(ax2,t,phi ,'-' ,'Color',C.phi,'LineWidth',2.4);
yline(ax2,180,':','Color',C.ink,'LineWidth',1.4,'Label','\phi = 180\circ (tool \perp wall)', ...
      'LabelHorizontalAlignment','left','FontSize',10);
xline(ax2,t_c,'-','Color',C.acc,'LineWidth',1.6);
legend(ax2,{'','\phi_d','\phi'},'Location','southeast','Box','off','FontSize',10);
xlabel(ax2,'time [s]'); ylabel(ax2,'orientation [deg]'); ylim(ax2,[80 200]);
title(ax2,'Tool orientation','FontSize',13,'FontWeight','bold','Color',C.ink);
grid(ax2,'on'); set(ax2,'GridColor',C.grid,'GridAlpha',0.9,'Box','off','XColor',C.ink,'YColor',C.ink);

exportgraphics(f,fullfile(outdir,'fig_pose.png'),'Resolution',150,'BackgroundColor',C.bg);
close(f);

%% ======================================================================
%  2. 접촉력
%% ======================================================================
f = figure('Color',C.bg,'Position',[80 80 900 620],'Visible','off');

ax1 = subplot(2,1,1); hold(ax1,'on');
area(ax1,[t_c t(end)],[140 140],-30,'FaceColor',C.acc,'FaceAlpha',0.07,'EdgeColor','none','ShowBaseLine','off');
plot(ax1,t,Fx,'-','Color',C.x,'LineWidth',2.4);
plot(ax1,t,Fy,'-','Color',C.y,'LineWidth',1.8);
yline(ax1,F_d,'--','Color',C.ink,'LineWidth',1.6,'Label','F_d = 10 N', ...
      'LabelHorizontalAlignment','left','FontSize',10);
plot(ax1,t(ic),Fx(ic),'o','MarkerSize',8,'LineWidth',2,'Color',C.acc);
text(ax1,t_c+0.06,max(Fx),sprintf('impact peak  %.0f N',max(Fx)),'Color',C.ink, ...
     'FontSize',10,'FontWeight','bold','VerticalAlignment','top');
legend(ax1,{'','F_x  (normal, force-controlled)','F_y  (friction)'}, ...
       'Location','northeast','Box','off','FontSize',10);
ylabel(ax1,'contact force [N]'); ylim(ax1,[-20 135]);
title(ax1,'Contact wrench','FontSize',13,'FontWeight','bold','Color',C.ink);
grid(ax1,'on'); set(ax1,'GridColor',C.grid,'GridAlpha',0.9,'Box','off','XColor',C.ink,'YColor',C.ink);

ax2 = subplot(2,1,2); hold(ax2,'on');
plot(ax2,t,Fx,'-','Color',C.x,'LineWidth',2.4);
yline(ax2,F_d,'--','Color',C.ink,'LineWidth',1.6);
xlim(ax2,[t_c 3]); ylim(ax2,[8 14]);
xlabel(ax2,'time [s]'); ylabel(ax2,'F_x [N]');
title(ax2,sprintf('zoom : F_x settles on F_d  (final %.3f N)',Fx(end)), ...
      'FontSize',13,'FontWeight','bold','Color',C.ink);
grid(ax2,'on'); set(ax2,'GridColor',C.grid,'GridAlpha',0.9,'Box','off','XColor',C.ink,'YColor',C.ink);

exportgraphics(f,fullfile(outdir,'fig_force.png'),'Resolution',150,'BackgroundColor',C.bg);
close(f);

%% ======================================================================
%  3. 관절 각도
%% ======================================================================
f = figure('Color',C.bg,'Position',[80 80 900 400],'Visible','off');
ax = axes('Parent',f); hold(ax,'on');
area(ax,[t_c t(end)],[200 200],-80,'FaceColor',C.acc,'FaceAlpha',0.07,'EdgeColor','none','ShowBaseLine','off');
for k = 1:3
    plot(ax,t,q(:,k),'-','Color',C.link(k,:),'LineWidth',2.4);
end
xline(ax,t_c,'-','Color',C.acc,'LineWidth',1.6);
legend(ax,{'','q_1  (shoulder)','q_2  (elbow)','q_3  (wrist)'}, ...
       'Location','east','Box','off','FontSize',10);
xlabel(ax,'time [s]'); ylabel(ax,'joint angle [deg]'); ylim(ax,[-60 160]);
title(ax,'Joint angles','FontSize',13,'FontWeight','bold','Color',C.ink);
grid(ax,'on'); set(ax,'GridColor',C.grid,'GridAlpha',0.9,'Box','off','XColor',C.ink,'YColor',C.ink);
exportgraphics(f,fullfile(outdir,'fig_joints.png'),'Resolution',150,'BackgroundColor',C.bg);
close(f);

%% ======================================================================
%  4. 스트로보 : 여러 시점의 자세를 한 그림에
%% ======================================================================
f = figure('Color',C.bg,'Position',[80 80 760 660],'Visible','off');
ax = axes('Parent',f); hold(ax,'on');
drawEnv(ax,x_wall,y_ground,C);

idx = [1 16 31 46 61 ic 81 101 151];
idx = unique(min(max(idx,1),numel(t)));
for m = 1:numel(idx)
    i  = idx(m);
    al = 0.16+0.84*(m-1)/(numel(idx)-1);
    drawArm(ax,q(i,:)*pi/180,Lk,C,al,false);
end
plot(ax,xd,yd,':','Color',C.des,'LineWidth',1.6);
plot(ax,x ,y ,'-','Color',C.x  ,'LineWidth',2.0);
plot(ax,x(ic),y(ic),'o','MarkerSize',9,'LineWidth',2,'Color',C.acc, ...
     'MarkerFaceColor',[1 1 1]);
text(ax,x(ic)+0.13,y(ic)+0.12,sprintf('contact  t = %.2f s',t_c), ...
     'FontSize',10,'FontWeight','bold','Color',C.ink);
text(ax,x(end)+0.13,y(end)-0.16,sprintf('hold 10 N  @ y = %.2f',y(end)), ...
     'FontSize',10,'FontWeight','bold','Color',C.ink);
finishScene(ax,C);
title(ax,'Approach \rightarrow contact \rightarrow slide  (stroboscopic)', ...
      'FontSize',13,'FontWeight','bold','Color',C.ink);
exportgraphics(f,fullfile(outdir,'fig_strobe.png'),'Resolution',150,'BackgroundColor',C.bg);
close(f);

%% ======================================================================
%  5. 필름스트립 : 4 컷
%% ======================================================================
shots = [1, 51, ic, numel(t)];
lbl   = {'t = 0.00 s   start (singular, fully extended)', ...
         't = 1.00 s   approach, tool rotating', ...
         sprintf('t = %.2f s   contact, force loop takes over',t_c), ...
         sprintf('t = %.2f s   sliding done, F_x = %.1f N',t(end),Fx(end))};

f = figure('Color',C.bg,'Position',[60 60 1360 420],'Visible','off');
for m = 1:4
    ax = subplot(1,4,m); hold(ax,'on');
    drawEnv(ax,x_wall,y_ground,C);
    i = shots(m);
    plot(ax,xd,yd,':','Color',C.des,'LineWidth',1.2);
    plot(ax,x(1:i),y(1:i),'-','Color',C.x,'LineWidth',1.8);
    drawArm(ax,q(i,:)*pi/180,Lk,C,1.0,true);
    if abs(Fx(i)) > 1e-6
        quiver(ax,x(i),y(i),0.55,0,0,'Color',C.acc,'LineWidth',2.5,'MaxHeadSize',1.2);
        text(ax,x(i)+0.1,y(i)+0.30,sprintf('%.0f N',Fx(i)),'Color',C.acc*0.8, ...
             'FontSize',11,'FontWeight','bold');
    end
    finishScene(ax,C);
    title(ax,lbl{m},'FontSize',10.5,'FontWeight','bold','Color',C.ink);
end
exportgraphics(f,fullfile(outdir,'fig_filmstrip.png'),'Resolution',140,'BackgroundColor',C.bg);
close(f);

%% ======================================================================
%  6. 애니메이션 GIF
%% ======================================================================
f = figure('Color',C.bg,'Position',[80 80 620 560],'Visible','off');
step = 2;
frames = 1:step:numel(t);
for m = 1:numel(frames)
    i = frames(m);
    clf(f); ax = axes('Parent',f); hold(ax,'on');
    drawEnv(ax,x_wall,y_ground,C);
    plot(ax,xd,yd,':','Color',C.des,'LineWidth',1.2);
    plot(ax,x(1:i),y(1:i),'-','Color',C.x,'LineWidth',1.8);
    drawArm(ax,q(i,:)*pi/180,Lk,C,1.0,true);
    if abs(Fx(i)) > 1e-6
        quiver(ax,x(i),y(i),0.5,0,0,'Color',C.acc,'LineWidth',2.5,'MaxHeadSize',1.2);
    end
    finishScene(ax,C);
    title(ax,sprintf('t = %5.2f s      F_x = %6.2f N',t(i),Fx(i)), ...
          'FontSize',12,'FontWeight','bold','Color',C.ink,'FontName','Consolas');
    im = print(f,'-RGBImage','-r70');
    [A,map] = rgb2ind(im,128,'nodither');
    if m == 1
        imwrite(A,map,fullfile(outdir,'anim.gif'),'gif', ...
                'LoopCount',Inf,'DelayTime',step*0.02);
    else
        imwrite(A,map,fullfile(outdir,'anim.gif'),'gif', ...
                'WriteMode','append','DelayTime',step*0.02);
    end
end
close(f);

disp('done');

%% ===== local functions ==================================================
function drawEnv(ax,x_wall,y_ground,C)
    patch(ax,[x_wall-0.45 x_wall x_wall x_wall-0.45],[y_ground y_ground 3.6 3.6], ...
          C.env,'EdgeColor','none','FaceAlpha',0.30);
    plot(ax,[x_wall x_wall],[y_ground 3.6],'Color',C.ink,'LineWidth',2.5);
    for j = y_ground+0.2:0.2:3.55
        plot(ax,[x_wall x_wall-0.4],[j j-0.22],'Color',C.env,'LineWidth',0.9);
    end
    patch(ax,[x_wall 2.4 2.4 x_wall],[y_ground y_ground -0.45 -0.45], ...
          C.env,'EdgeColor','none','FaceAlpha',0.30);
    plot(ax,[x_wall 2.4],[y_ground y_ground],'Color',C.ink,'LineWidth',2.5);
    for j = x_wall+0.2:0.2:2.38
        plot(ax,[j j-0.22],[y_ground y_ground-0.4],'Color',C.env,'LineWidth',0.9);
    end
end

function drawArm(ax,a3,Lk,C,alpha,withGrip)
    a = [a3(1), a3(1)+a3(2), a3(1)+a3(2)+a3(3)];
    P = zeros(2,4);
    for k = 1:3
        P(:,k+1) = P(:,k)+Lk(k)*[cos(a(k)); sin(a(k))];
    end
    wLink = [0.150 0.130 0.105];
    rJnt  = [0.115 0.100 0.085];
    if alpha >= 0.999
        patch(ax,[-0.36 0.36 0.24 -0.24],[-0.14 -0.14 0.05 0.05],C.base, ...
              'EdgeColor',C.ink,'LineWidth',1.5);
    end
    for k = 1:3
        capsule(ax,P(:,k),P(:,k+1),wLink(k),C.link(k,:),C.ink,alpha);
    end
    if withGrip
        u3 = [cos(a(3)); sin(a(3))]; n3 = [-u3(2); u3(1)];
        palm = P(:,4);
        capsule(ax,palm-0.10*n3,palm+0.10*n3,0.075,C.grip,C.ink,alpha);
        for sgn = [-1 1]
            f0 = palm+sgn*0.10*n3;
            capsule(ax,f0,f0+0.17*u3,0.055,C.grip,C.ink,alpha);
        end
    end
    for k = 1:3
        th = linspace(0,2*pi,48);
        patch(ax,P(1,k)+rJnt(k)*cos(th),P(2,k)+rJnt(k)*sin(th),C.jnt, ...
              'EdgeColor',C.ink,'LineWidth',1.4,'FaceAlpha',alpha,'EdgeAlpha',alpha);
        patch(ax,P(1,k)+0.34*rJnt(k)*cos(th),P(2,k)+0.34*rJnt(k)*sin(th),C.ink, ...
              'EdgeColor','none','FaceAlpha',alpha);
    end
end

function capsule(ax,p0,p1,w,faceCol,edgeCol,alpha)
    p0 = p0(:); p1 = p1(:);
    d  = p1-p0; Ln = hypot(d(1),d(2));
    if Ln < 1e-9, return; end
    u  = d/Ln; nv = [-u(2); u(1)];
    th = linspace(0,pi,16);
    Pg = [p1+(w/2)*( u*sin(th)+nv*cos(th)), p0+(w/2)*(-u*sin(th)-nv*cos(th))];
    patch(ax,Pg(1,:),Pg(2,:),faceCol,'EdgeColor',edgeCol,'LineWidth',1.2, ...
          'FaceAlpha',alpha,'EdgeAlpha',alpha);
end

function finishScene(ax,C)
    axis(ax,'equal'); axis(ax,[-2.5 2.3 -0.5 3.6]);
    set(ax,'XGrid','on','YGrid','on','GridColor',C.grid,'GridAlpha',0.9, ...
           'Layer','top','Color',C.bg,'Box','off','XColor',C.ink,'YColor',C.ink);
    xlabel(ax,'x [m]'); ylabel(ax,'y [m]');
end
