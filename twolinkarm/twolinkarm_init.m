%% Motor constants

Ra = 4;              % armature resistance
La = 2.75E-6;        % armature inductance
Kemf = 0.0274;         % back emf constant
Jm = 3.2284E-6;      % rotor inertia
bm = 3.5077E-6;      % rotor damping

Tstall = 4.903325; % N.m
Wnoload = 58.82; % rpm
V = 7.4; % V
Kt = 1.20;        % determined experimentally from simple_motor

%% Optimal trajectory

f = 0.1;
tDelta = 1/f;
tTrajectory = 30;
tTotal = tTrajectory + 1;
numPoints = 999;
t = linspace(0, tTrajectory, numPoints);
q_max = [pi pi/2];
q_min = [-pi -pi/2];
q0 = [0 0];
q1 = q_max(1)*sin(2*pi*f*t) + q0(1);
q2 = q_max(2)*sin(2*pi*f*t) + q0(2);
% add zeros to stop velocity
q1 = [q1 0];
q2 = [q2 0];
tSize = size(t,2);
t = [t t(tSize)+tDelta];

q1_traj = timeseries(q1, t);
q2_traj = timeseries(q2, t);

% subplot(2,1,1);
% plot(t, q1);
% subplot(2,1,2);
% plot(t, q2);

ts = 0.1; % sample time, sec

%% Load dynamic model

load('twolinkarm_DPI.mat')

rLink = 0.005/2;
a1_val = 0.05;
a2_val = 0.05;
lC1_val = a1_val/2;
lC2_val = a2_val/2;
k1_val = 100;
k2_val = 100;
mL1_val = 0.01;
mL2_val = 0.01;
density = mL1_val / (pi * rLink^2 * a1_val); % kg/m^3

% sub values into DPI model
% must set params lC1, lC2, a1, a2, k1, k2
Y = subs(Y, ...
    [a1, a2, lC1, lC2, k1, k2], ...
    [a1_val, a2_val, lC1_val, lC2_val, k1_val, k2_val]);

%% Simulation

sim('twolinkarm')

%% Calculate PI from measured q, q_d, q_dd, I

numDataPts = size(theta1.data,1);

T1 = Kt * I1.data;
T2 = Kt * I2.data;

Ym = zeros(numDataPts*size(Y,1), size(Y,2));
TAUm = zeros(numDataPts*size(tau,1), size(tau,2));

% weighting matrix to account for max torques of each joint
W = [1/Tstall 0 ; 0 1/Tstall];

for i = 1:numDataPts
    
    row = i*size(Y,1)-1;
    
    Ymi = subs(Y, ...
        [th1, th1_d, th1_dd, th2, th2_d, th2_dd, g], ...
        [theta1.data(i), theta1_d.data(i), theta1_dd.data(i), ...
        theta2.data(i), theta2_d.data(i), theta2_dd.data(i), 9.81]);  
    Ymi = W * Ymi;
    Ym(row:row+1, :) = Ymi;
    
    TAUmi = subs(tau, ...
        [tau1, tau2], ...
        [T1(i), T2(i)]);
    TAUmi = W * TAUmi;
    TAUm(row:row+1) = TAUmi;
    
end

% remove null columns of Ym
% must repeat since some params may not be solvable through this path
p = size(Ym,2);
PIm = PI;
i = 1;
TOL = 1e-12;

while i < p
    
    if norm(Ym(:,i)) < TOL   % remove that column
        
        Ym = [Ym(:,1:i-1) Ym(:, i+1:p)];
        PIm = [PIm(1:i-1); PIm(i+1:p)];
        p = p-1;
        
    else
        
        i = i+1;
        
    end   
        
end

result = ((Ym.' * Ym) \ Ym.') * TAUm;

% print results to console 
for i = 1:size(PIm)
    fprintf('%s = %.15f\n', PIm(i), result(i));
end