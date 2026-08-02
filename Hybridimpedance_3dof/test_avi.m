% === 사전 설정 ===
% 링크 길이, 자세, etc. 정의 필요
Link11 = 1.0;
Link12 = 1.0;
Link13 = 1.0;

% Q = [theta1 theta2 theta3] over time (Nx3)
% 예시: Q = [linspace(0, pi, 100)' linspace(0, pi/2, 100)' linspace(0, pi/2, 100)'];
n = size(Q, 1);

% === 비디오 객체 생성 (VideoWriter 사용) ===
v = VideoWriter('mymovie.avi');
v.FrameRate = 50;
open(v);

% === 애니메이션 루프 ===
for i = 1:n
    theta1 = Q(i,1);
    theta2 = Q(i,2);
    theta3 = Q(i,3);

    x11 = Link11 * cos(theta1);
    y11 = Link11 * sin(theta1);
    x12 = x11 + Link12 * cos(theta1 + theta2);
    y12 = y11 + Link12 * sin(theta1 + theta2);
    x13 = x12 + Link13 * cos(theta1 + theta2 + theta3);
    y13 = y12 + Link13 * sin(theta1 + theta2 + theta3);

    % === 그림 그리기 ===
    plot([0 x11 x12 x13], [0 y11 y12 y13], 'ko-', 'LineWidth', 2, 'MarkerFaceColor','k');
    hold on

    % 배경 구조물 그리기
    plot([-0.5 -0.5],[3.0 0],'k','LineWidth',3);
    for j = 0.25:0.25:3.0
        plot([-0.5 -1],[j j-0.5],'k','LineWidth',1);
    end

    axis([-2.0 2.0 0.0 3.5]);
    axis square
    grid on

    frame = getframe(gcf);
    writeVideo(v, frame);

    hold off
end

close(v);
