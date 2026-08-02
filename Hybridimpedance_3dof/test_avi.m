% === 3자유도 로봇팔 애니메이션 ===
% 링크마다 다른 색을 주고, 관절/베이스/그리퍼를 그려서 실제 로봇팔처럼 보이게 한다.
% run.m 에서 sim 이 끝난 뒤 호출되며 작업공간 변수 Q (Nx3, rad) 를 사용한다.
% data 가 있으면 엔드이펙터 궤적과 접촉력도 같이 표시한다.

Link11 = 1.0;
Link12 = 1.0;
Link13 = 1.0;

x_wall_p   = -1.5;      % 벽 위치 (init.m 의 x_wall 과 같게)
y_ground_p =  0.0;      % 지면 높이 (init.m 의 y_ground 과 같게)

% --- 색 팔레트 -----------------------------------------------------------
colLink  = [ 0.96 0.62 0.11      % 링크1 : 앰버
             0.13 0.72 0.68      % 링크2 : 청록
             0.60 0.36 0.87 ];   % 링크3 : 보라
colJoint = [0.93 0.94 0.96];     % 관절 캡
colEdge  = [0.13 0.14 0.18];     % 외곽선
colBase  = [0.24 0.26 0.32];     % 베이스
colGrip  = [0.30 0.33 0.40];     % 그리퍼
colEnv   = [0.62 0.64 0.68];     % 벽 / 지면
colTrail = [0.85 0.20 0.35];     % 엔드이펙터 궤적

wLink = [0.150 0.130 0.105];     % 링크 두께
rJnt  = [0.115 0.100 0.085];     % 관절 반지름

n = size(Q, 1);
hasData = exist('data','var') && size(data,1) == n;

v = VideoWriter('mymovie.avi');
v.FrameRate = 50;
open(v);

fig = figure('Color',[0.98 0.98 0.99],'Position',[100 100 760 640]);

for i = 1:n
    clf(fig); ax = axes('Parent',fig); hold(ax,'on');

    a  = [Q(i,1), Q(i,1)+Q(i,2), Q(i,1)+Q(i,2)+Q(i,3)];
    Lk = [Link11 Link12 Link13];
    P  = zeros(2,4);
    for k = 1:3
        P(:,k+1) = P(:,k) + Lk(k)*[cos(a(k)); sin(a(k))];
    end

    % ---------- 환경 : 벽과 지면 ----------
    patch([x_wall_p-0.45 x_wall_p x_wall_p x_wall_p-0.45], ...
          [y_ground_p y_ground_p 3.6 3.6], colEnv, ...
          'EdgeColor','none','FaceAlpha',0.35);
    plot([x_wall_p x_wall_p],[y_ground_p 3.6],'Color',colEdge,'LineWidth',3);
    for j = y_ground_p+0.2:0.2:3.55
        plot([x_wall_p x_wall_p-0.4],[j j-0.22],'Color',colEnv,'LineWidth',1);
    end

    patch([x_wall_p 2.4 2.4 x_wall_p],[y_ground_p y_ground_p -0.45 -0.45], ...
          colEnv,'EdgeColor','none','FaceAlpha',0.35);
    plot([x_wall_p 2.4],[y_ground_p y_ground_p],'Color',colEdge,'LineWidth',3);
    for j = x_wall_p+0.2:0.2:2.38
        plot([j j-0.22],[y_ground_p y_ground_p-0.4],'Color',colEnv,'LineWidth',1);
    end

    % ---------- 엔드이펙터 궤적 ----------
    if hasData
        plot(data(1:i,2), data(1:i,3), '-', 'Color', colTrail, 'LineWidth', 1.4);
        plot(data(:,5), data(:,6), ':', 'Color', [0.55 0.55 0.60], 'LineWidth', 1.1);
    end

    % ---------- 베이스 ----------
    patch([-0.36 0.36 0.24 -0.24],[-0.14 -0.14 0.05 0.05], colBase, ...
          'EdgeColor',colEdge,'LineWidth',1.5);

    % ---------- 링크 ----------
    for k = 1:3
        drawCapsule(P(:,k), P(:,k+1), wLink(k), colLink(k,:), colEdge);
    end

    % ---------- 그리퍼 ----------
    u3 = [cos(a(3)); sin(a(3))];  n3 = [-u3(2); u3(1)];
    palm = P(:,4);
    drawCapsule(palm-0.10*n3, palm+0.10*n3, 0.075, colGrip, colEdge);
    for sgn = [-1 1]
        f0 = palm + sgn*0.10*n3;
        drawCapsule(f0, f0+0.17*u3, 0.055, colGrip, colEdge);
    end

    % ---------- 관절 ----------
    for k = 1:3
        drawJoint(P(:,k), rJnt(k), colJoint, colEdge);
    end

    % ---------- 라벨 ----------
    if hasData
        txt = sprintf('t = %.2f s      F_x = %6.2f N      F_y = %6.2f N', ...
                      data(i,1), data(i,11), data(i,12));
        if abs(data(i,11)) > 1e-6
            txt = [txt '      [CONTACT]'];
        end
    else
        txt = sprintf('frame %d / %d', i, n);
    end
    text(-2.3, 3.42, txt, 'FontSize', 11, 'FontWeight', 'bold', ...
         'Color', colEdge, 'FontName', 'Consolas');

    axis(ax,'equal'); axis(ax,[-2.4 2.4 -0.5 3.6]);
    set(ax,'XGrid','on','YGrid','on','GridAlpha',0.12,'Layer','top', ...
           'Color',[0.98 0.98 0.99]);
    xlabel(ax,'x [m]'); ylabel(ax,'y [m]');

    writeVideo(v, getframe(fig));
end

close(v);
close(fig);

% ===== 로컬 함수 =========================================================
function drawCapsule(p0, p1, w, faceCol, edgeCol)
% 두 점을 잇는 둥근 막대(캡슐) 모양의 링크를 그린다
    p0 = p0(:); p1 = p1(:);
    d  = p1 - p0;
    Ln = hypot(d(1), d(2));
    if Ln < 1e-9, return; end
    u  = d/Ln;  nv = [-u(2); u(1)];
    t  = linspace(0, pi, 16);
    capEnd   = p1 + (w/2)*( u*sin(t) + nv*cos(t));
    capStart = p0 + (w/2)*(-u*sin(t) - nv*cos(t));
    Pg = [capEnd, capStart];
    patch(Pg(1,:), Pg(2,:), faceCol, 'EdgeColor', edgeCol, 'LineWidth', 1.3);
end

function drawJoint(p, r, faceCol, edgeCol)
% 관절 캡 : 밝은 원 + 가운데 축 표시
    t = linspace(0, 2*pi, 48);
    patch(p(1)+r*cos(t), p(2)+r*sin(t), faceCol, ...
          'EdgeColor', edgeCol, 'LineWidth', 1.5);
    patch(p(1)+0.34*r*cos(t), p(2)+0.34*r*sin(t), edgeCol, 'EdgeColor','none');
end
