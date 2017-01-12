%% Script Information
%
% Filename: pendulum_init.m
% Description: inverted pendulum
%
% Created By: Ashlin Kanawaty
% Last Modified By: Ashlin Kanawaty
% Mondification Date: 09/21/2015

%% System paramters

M = 0.5;        % mass of the cart, kg
m = 0.2;        % mass of the pendulumn, kg
b = 0.1;        % coefficient of friction for cart, N/m/sec
l = 0.3;        % length to pendulum center of mass, m
I = 0.006;      % mass moment of inertia of the pendulum, kg.m^2
g = 9.81;        % gravitational constant, m/s^2

% input variable:
%   F, force applied to cart
% measured variables:
%   x, position of the cart
%   theta, pendulum angle from horizontal

%% Force

Force = xlsread('force.xlsx',1,'A2:B11');
time = length(Force);

%% Run Simulation

%set_param('pendulum_model','AlgebraicLoopSolver','LineSearch')
%sim('Pend_Model_Simscape')
sym('system_model');

%% Plot

% fig1 = figure;
% hold on
% grid on
% box on
% subplot(2,1,1)
% plot(tout, CurrentGear.signals.values, 'r', tout, DesiredGear.signals.values, 'b', tout, Trans_GearRequest.signals.values, 'g', 'LineWidth', 3)
% title('Gears', 'FontSize', 20)
% xlabel('Time', 'FontSize', 16)
% ylabel('Gear', 'FontSize', 16)
% leg1 = legend('Current gear', 'Desired gear', 'Transmission gear req');
% set(leg1, 'FontSize', 16)
% subplot(2,1,2)
% plot(tout, FLAG_ShiftInProgress.signals.values, 'r', tout, FLAG_TCLockup.signals.values, 'm', tout, FLAG_TCTempHigh.signals.values, 'g', tout, FLAG_ATFTempHigh.signals.values, 'b', tout, FLAG_ShiftFault.signals.values, 'k', tout, FLAG_DisengageTC.signals.values, 'c', 'LineWidth', 3)
% title('Flags', 'FontSize', 20)
% xlabel('Time', 'FontSize', 16)
% ylabel('FLAG value', 'FontSize', 16)
% leg2 = legend('ShiftInProgress', 'TCLockup', 'TCTempHigh', 'ATFTempHigh', 'ShiftFault', 'DisengageTC');
% set(leg2, 'FontSize', 16)
% set (fig1, 'Units', 'normalized', 'outerposition', [0,0,1,1])