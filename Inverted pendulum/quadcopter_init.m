%% Modeling the nonlinear state space in simulink

noStimulus = xlsread('quadcopter2_testcases.xlsx', 1, 'A2:E51');
impulseOmega13 = xlsread('quadcopter2_testcases.xlsx', 2, 'A2:E51');
impulseOmega24 = xlsread('quadcopter2_testcases.xlsx', 3, 'A2:E51');
impulsePitchY = xlsread('quadcopter2_testcases.xlsx', 4, 'A2:E51');
impulseRollX = xlsread('quadcopter2_testcases.xlsx', 5, 'A2:E51');
impulseOmegaALL = xlsread('quadcopter2_testcases.xlsx', 6, 'A2:E51');
Input = impulsePitchY;
%simTime = length(Input);
simTime = 7;
sim('quadcopter2.slx')

% graphs
fig1 = figure;
hold on
grid on
box on
subplot(2,1,1)
plot(tout, alpha, 'r', tout, beta, 'b', tout, gamma, 'g', 'LineWidth', 3)
title('Angular position', 'FontSize', 20)
xlabel('Time', 'FontSize', 16)
ylabel('Angular position', 'FontSize', 16)
leg1 = legend('alpha', 'beta', 'gamma');
set(leg1, 'FontSize', 16)
subplot(2,1,2)
plot(tout, alpha_d, 'r', tout, beta_d, 'b', tout, gamma_d, 'g', 'LineWidth', 3)
title('Angular velocity', 'FontSize', 20)
xlabel('Time', 'FontSize', 16)
ylabel('Angular velocity', 'FontSize', 16)
leg2 = legend('alpha_d', 'beta_d', 'gamma_d');
set(leg2, 'FontSize', 16)
set (fig1, 'Units', 'normalized', 'outerposition', [0,0,1,1])

fig2 = figure;
hold on
grid on
box on
subplot(2,1,1)
plot(tout, z, 'r', tout, y, 'b', tout, x, 'g', 'LineWidth', 3)
title('Linear position', 'FontSize', 20)
xlabel('Time', 'FontSize', 16)
ylabel('Linear position', 'FontSize', 16)
leg3 = legend('z', 'y', 'x');
set(leg3, 'FontSize', 16)
subplot(2,1,2)
plot(tout, z_d, 'r', tout, y_d, 'b', tout, x_d, 'g', 'LineWidth', 3)
title('Linear velocity', 'FontSize', 20)
xlabel('Time', 'FontSize', 16)
ylabel('Linear velocity', 'FontSize', 16)
leg4 = legend('z_d', 'y_d', 'x_d');
set(leg4, 'FontSize', 16)
set (fig2, 'Units', 'normalized', 'outerposition', [0,0,1,1])

% fig3 = figure;
% hold on
% grid on
% box on
% plot(tout, z, 'r', tout, z_d, 'b', tout, z_dd, 'g', 'LineWidth', 3)
% title('Linear position', 'FontSize', 20)
% xlabel('Time', 'FontSize', 16)
% ylabel('Linear position', 'FontSize', 16)
% leg5 = legend('z', 'z d', 'z dd');
% set(leg5, 'FontSize', 16)
% set (fig3, 'Units', 'normalized', 'outerposition', [0,0,1,1])
