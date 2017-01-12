%% Modeling the nonlinear state space in simulink

noStimulus = xlsread('pendulum1_testcases.xlsx', 1, 'A2:E101');
impulseXdd = xlsread('pendulum1_testcases.xlsx', 2, 'A2:E101');
impulseYdd = xlsread('pendulum1_testcases.xlsx', 3, 'A2:E101');
impulseZdd = xlsread('pendulum1_testcases.xlsx', 4, 'A2:E101');
Input = impulseYdd;
simTime = length(Input);
%simTime = 20;
sim('pendulum1.slx')

% graphs
fig1 = figure;
hold on
grid on
box on
subplot(3,1,1)
plot(tout, r, 'r', tout, s, 'b', 'LineWidth', 3)
title('Linear position', 'FontSize', 20)
xlabel('Time', 'FontSize', 16)
ylabel('Position', 'FontSize', 16)
leg1 = legend('r', 's');
set(leg1, 'FontSize', 16)
subplot(3,1,2)
plot(tout, r_d, 'r', tout, s_d, 'b', 'LineWidth', 3)
title('Linear velocity', 'FontSize', 20)
xlabel('Time', 'FontSize', 16)
ylabel('Velocity', 'FontSize', 16)
leg2 = legend('r_d', 's_d');
set(leg2, 'FontSize', 16)
subplot(3,1,3)
plot(tout, r_dd, 'r', tout, s_dd, 'b', 'LineWidth', 3)
title('Linear acceleration', 'FontSize', 20)
xlabel('Time', 'FontSize', 16)
ylabel('Acceleration', 'FontSize', 16)
leg3 = legend('r dd', 's dd');
set(leg3, 'FontSize', 16)
set (fig1, 'Units', 'normalized', 'outerposition', [0,0,1,1])

% fig2 = figure;
% hold on
% grid on
% box on
% subplot(2,1,1)
% plot(tout, s_dd, 'b', tout, r_dd, 'r', 'LineWidth', 3)
% title('Linear acceleration', 'FontSize', 20)
% xlabel('Time', 'FontSize', 16)
% ylabel('Acceleration', 'FontSize', 16)
% leg3 = legend('s dd', 'r dd');
% set(leg3, 'FontSize', 16)
% subplot(2,1,2)
% plot(tout, x_dd_out, 'b', tout, y_dd_out, 'r', tout, z_dd_out, 'g', 'LineWidth', 3)
% title('Input acceleration', 'FontSize', 20)
% xlabel('Time', 'FontSize', 16)
% ylabel('Acceleration', 'FontSize', 16)
% leg4 = legend('x dd', 'y dd', 'z dd');
% set(leg4, 'FontSize', 16)
% set (fig2, 'Units', 'normalized', 'outerposition', [0,0,1,1])