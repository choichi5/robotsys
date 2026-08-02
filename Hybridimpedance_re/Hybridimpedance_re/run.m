clear all
close all

%$
global Link11;   
global Link12;
global Mass11;
global Mass12;
global Link11g;
global Link12g;
global I11;  
global I12;  
%global k_p;
%global k_d;
global g;

%%%%%%% 둫긬깋긽?[?^궻벶귒뜛귒 %%%%%%%
init
%Damping

%%%%%%% 긘깄?~깒?[긘깈깛둎럑 %%%%%%%
sim('Impedance_model')

%%%%%%% 뺎뫔 %%%%%%%
save('data.txt','data','-ASCII','-tabs');

%%%%%%% ???[긮?[ %%%%%%%
test_avi



figure;
subplot(2,1,1);
plot(data(:,1), data(:,2),'-r',data(:,1), data(:,3),'-b', data(:,1), data(:,4), '-g', data(:,1), data(:,5), '-y');
%title('Position trajectory');
%ylim([-30 30]);
legend('x','y','x_{d}','y_{d}');
xlabel('time[sec]');
ylabel('Position trajectories[m]');
grid on;

subplot(2,1,2);
plot(data(:,1), data(:,8),'-r', data(:,1), data(:,9),'-b');
xlabel('time[sec]');
ylabel('Contact Force[N]');
legend('x direction','y direction');
ylim([-10 110]);
grid on;

