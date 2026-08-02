clear all
close all

%$
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
%global k_p;
%global k_d;
global g;

%%%%%%% 파라미터 읽어들이기 %%%%%%%
init
%Damping

%%%%%%% 시뮬레이션 시작 %%%%%%%
sim('Impedance_model')

%%%%%%% 보존 %%%%%%%
save('data.txt','data','-ASCII','-tabs');

%%%%%%% 애니메이션 %%%%%%%
test_avi

% data columns:
%   1     : t
%   2: 4  : x, y, phi          (actual task-space pose)
%   5: 7  : xd, yd, phid       (desired task-space pose)
%   8:10  : q11, q12, q13      [deg]
%  11:13  : Fx, Fy, Mz         (contact wrench)

figure;
subplot(3,1,1);
plot(data(:,1), data(:,2),'-r',data(:,1), data(:,3),'-b', ...
     data(:,1), data(:,5),'--r',data(:,1), data(:,6),'--b');
%title('Position trajectory');
%ylim([-30 30]);
legend('x','y','x_{d}','y_{d}');
xlabel('time[sec]');
ylabel('Position trajectories[m]');
grid on;

subplot(3,1,2);
plot(data(:,1), data(:,4)*180/pi,'-k', data(:,1), data(:,7)*180/pi,'--k');
legend('\phi','\phi_{d}');
xlabel('time[sec]');
ylabel('Orientation[deg]');
grid on;

subplot(3,1,3);
plot(data(:,1), data(:,11),'-r', data(:,1), data(:,12),'-b');
xlabel('time[sec]');
ylabel('Contact Force[N]');
legend('x direction','y direction');
ylim([-10 110]);
grid on;
