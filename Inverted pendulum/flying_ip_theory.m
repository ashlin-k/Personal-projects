% State variables
syms x x_d y y_d z z_d alpha alpha_d beta beta_d gamma gamma_d alpha_dd beta_dd gamma_dd;    % quadcopter
syms r s r_d s_d;                                                                            % pendulum

% Input variables
syms omega_x omega_y omega_z thrust;

% Output variables
syms x_dd y_dd z_dd;                                                                         % quadcopter
syms r_dd s_dd;                                                                              % pendulum

% Constants
syms g L;
g = 9.81;
%L = 1;
zeta = sqrt(L^2 - r^2 - s^2);

%% Relationships from flying IP paper

% Quadcopter, translational acceleration
Rx(gamma) = [1 0 0; 0 cos(gamma) -sin(gamma); 0 sin(gamma) cos(gamma)];
Ry(beta) = [cos(beta) 0 sin(beta); 0 1 0; -sin(beta) 0 cos(beta)];
Rz(alpha) = [cos(alpha) -sin(alpha) 0; sin(alpha) cos(alpha) 0; 0 0 1];
R(alpha, beta, gamma) = Rz(alpha) * Ry(beta) * Rx(gamma);
A = mtimes(R(alpha, beta, gamma), [0; 0; thrust]) + [0; 0; -g];
x_dd = A(1);
y_dd = A(2);
z_dd = A(3);
trans_acc = [x_dd; y_dd; z_dd];

% Quadcopter, Euler angles
B = inv([(cos(beta)*cos(gamma)) (-sin(gamma)) 0; (cos(beta)*sin(gamma)) cos(gamma) 0; (-sin(beta)) 0 1]);
% this B is from another paper, but i think the original is right
%B = [1 0 (-sin(beta)); 0 cos(gamma) sin(gamma)*cos(gamma); 0 (-sin(gamma)) cos(gamma)*cos(beta)];      
C = mtimes(B, [omega_x; omega_y; omega_z]);
gamma_d = C(1);
beta_d = C(2);
alpha_d = C(3);
euler_ang = [gamma_d; beta_d; alpha_d];

% Pendulum
r_dd = (1/((L^2 - s^2)*zeta)) * ( -r^4*x_dd - (L^2 - s^2)^2*x_dd - 2*r^2*(s*r_d*s_d + (-L^2 + s^2)*x_dd) + r^3*(s_d^2 + s*s_dd - zeta*(g+z_dd)) + r*(-L^2*s*s_dd + s^3*s_dd + s^2*(r_d^2 - zeta*(g+z_dd)) + L^2*(-r_d^2 - s_d^2 + zeta*(g+z_dd))) );
s_dd = (1/((L^2 - s^2)*zeta)) * ( -s^4*y_dd - (L^2 - r^2)^2*y_dd - 2*s^2*(r*r_d*s_d + (-L^2 + r^2)*y_dd) + s^3*(r_d^2 + r*r_dd - zeta*(g+z_dd)) + s*(-L^2*r*r_dd + r^3*r_dd + r^2*(s_d^2 - zeta*(g+z_dd)) + L^2*(-r_d^2 - s_d^2 + zeta*(g+z_dd))) );
pend_acc = [r_dd; s_dd];

%solve_rs = solve([r_dd == ((1/((L^2 - s^2)*zeta)) * ( -r^4*x_dd - (L^2 - s^2)^2*x_dd - 2*r^2*(s*r_d*s_d + (-L^2 + s^2)*x_dd) + r^3*(s_d^2 + s*s_dd - zeta*(g+z_dd)) + r*(-L^2*s*s_dd + s^3*s_dd + s^2*(r_d^2 - zeta*(g+z_dd)) + L^2*(-r_d^2 - s_d^2 + zeta*(g+z_dd))) )), s_dd == ((1/((L^2 - s^2)*zeta)) * ( -s^4*y_dd - (L^2 - r^2)^2*y_dd - 2*s^2*(r*r_d*s_d + (-L^2 + r^2)*y_dd) + s^3*(r_d^2 + r*r_dd - zeta*(g+z_dd)) + s*(-L^2*r*r_dd + r^3*r_dd + r^2*(s_d^2 - zeta*(g+z_dd)) + L^2*(-r_d^2 - s_d^2 + zeta*(g+z_dd))) ))], [r_dd, s_dd]);
%solve_rs.r_dd
%solve_rs.s_dd
% too long to solve

% two eqns two unknowns, solving for r_dd and s_dd
[r_dd2, s_dd2] = solve_rs(); 

% r_dd2 = (x_dd*(L^2 - s^2)^2 - 2*r^2*(x_dd*(L^2 - s^2) - r_d*s*s_d) - r^3*(s_d^2 - (g + z_dd)*(L^2 - r^2 - s^2)^(1/2) + (s*(r^3*((g + z_dd)*(L^2 - r^2 - s^2)^(1/2) - s_d^2) - 2*r^2*(x_dd*(L^2 - s^2) - r_d*s*s_d) + x_dd*(L^2 - s^2)^2 + r^4*x_dd + r*(s^2*((g + z_dd)*(L^2 - r^2 - s^2)^(1/2) - r_d^2) + L^2*(r_d^2 - (g + z_dd)*(L^2 - r^2 - s^2)^(1/2) + s_d^2))))/(r^3*s + r*s^3 - L*r*s)) + r*(s^2*((g + z_dd)*(L^2 - r^2 - s^2)^(1/2) - r_d^2) + L^2*(r_d^2 - (g + z_dd)*(L^2 - r^2 - s^2)^(1/2) + s_d^2) - (s^3*(r^3*((g + z_dd)*(L^2 - r^2 - s^2)^(1/2) - s_d^2) - 2*r^2*(x_dd*(L^2 - s^2) - r_d*s*s_d) + x_dd*(L^2 - s^2)^2 + r^4*x_dd + r*(s^2*((g + z_dd)*(L^2 - r^2 - s^2)^(1/2) - r_d^2) + L^2*(r_d^2 - (g + z_dd)*(L^2 - r^2 - s^2)^(1/2) + s_d^2))))/(r^3*s + r*s^3 - L*r*s) + (L^2*s*(r^3*((g + z_dd)*(L^2 - r^2 - s^2)^(1/2) - s_d^2) - 2*r^2*(x_dd*(L^2 - s^2) - r_d*s*s_d) + x_dd*(L^2 - s^2)^2 + r^4*x_dd + r*(s^2*((g + z_dd)*(L^2 - r^2 - s^2)^(1/2) - r_d^2) + L^2*(r_d^2 - (g + z_dd)*(L^2 - r^2 - s^2)^(1/2) + s_d^2))))/(r^3*s + r*s^3 - L*r*s)) + r^4*x_dd)/((L^2 - s^2)*((r*((s^3*(L^2 - s^2)*(L^2 - r^2 - s^2)^(1/2))/(r^3*s + r*s^3 - L*r*s) - (L^2*s*(L^2 - s^2)*(L^2 - r^2 - s^2)^(1/2))/(r^3*s + r*s^3 - L*r*s)) + (r^3*s*(L^2 - s^2)*(L^2 - r^2 - s^2)^(1/2))/(r^3*s + r*s^3 - L*r*s))/((L^2 - s^2)*(L^2 - r^2 - s^2)^(1/2)) - 1)*(L^2 - r^2 - s^2)^(1/2))
% s_dd2 = ((x_dd*(L^2 - s^2)^2 - 2*r^2*(x_dd*(L^2 - s^2) - r_d*s*s_d) - r^3*(s_d^2 - (g + z_dd)*(L^2 - r^2 - s^2)^(1/2) + (s*(r^3*((g + z_dd)*(L^2 - r^2 - s^2)^(1/2) - s_d^2) - 2*r^2*(x_dd*(L^2 - s^2) - r_d*s*s_d) + x_dd*(L^2 - s^2)^2 + r^4*x_dd + r*(s^2*((g + z_dd)*(L^2 - r^2 - s^2)^(1/2) - r_d^2) + L^2*(r_d^2 - (g + z_dd)*(L^2 - r^2 - s^2)^(1/2) + s_d^2))))/(r^3*s + r*s^3 - L*r*s)) + r*(s^2*((g + z_dd)*(L^2 - r^2 - s^2)^(1/2) - r_d^2) + L^2*(r_d^2 - (g + z_dd)*(L^2 - r^2 - s^2)^(1/2) + s_d^2) - (s^3*(r^3*((g + z_dd)*(L^2 - r^2 - s^2)^(1/2) - s_d^2) - 2*r^2*(x_dd*(L^2 - s^2) - r_d*s*s_d) + x_dd*(L^2 - s^2)^2 + r^4*x_dd + r*(s^2*((g + z_dd)*(L^2 - r^2 - s^2)^(1/2) - r_d^2) + L^2*(r_d^2 - (g + z_dd)*(L^2 - r^2 - s^2)^(1/2) + s_d^2))))/(r^3*s + r*s^3 - L*r*s) + (L^2*s*(r^3*((g + z_dd)*(L^2 - r^2 - s^2)^(1/2) - s_d^2) - 2*r^2*(x_dd*(L^2 - s^2) - r_d*s*s_d) + x_dd*(L^2 - s^2)^2 + r^4*x_dd + r*(s^2*((g + z_dd)*(L^2 - r^2 - s^2)^(1/2) - r_d^2) + L^2*(r_d^2 - (g + z_dd)*(L^2 - r^2 - s^2)^(1/2) + s_d^2))))/(r^3*s + r*s^3 - L*r*s)) + r^4*x_dd)/((r*((s^3*(L^2 - s^2)*(L^2 - r^2 - s^2)^(1/2))/(r^3*s + r*s^3 - L*r*s) - (L^2*s*(L^2 - s^2)*(L^2 - r^2 - s^2)^(1/2))/(r^3*s + r*s^3 - L*r*s)) + (r^3*s*(L^2 - s^2)*(L^2 - r^2 - s^2)^(1/2))/(r^3*s + r*s^3 - L*r*s))/((L^2 - s^2)*(L^2 - r^2 - s^2)^(1/2)) - 1) + r^3*((g + z_dd)*(L^2 - r^2 - s^2)^(1/2) - s_d^2) - 2*r^2*(x_dd*(L^2 - s^2) - r_d*s*s_d) + x_dd*(L^2 - s^2)^2 + r^4*x_dd + r*(s^2*((g + z_dd)*(L^2 - r^2 - s^2)^(1/2) - r_d^2) + L^2*(r_d^2 - (g + z_dd)*(L^2 - r^2 - s^2)^(1/2) + s_d^2)))/(r^3*s + r*s^3 - L*r*s)

%% Nonlinear state space of QC
% state_d = f(state) + g(state)*input
% output = output_vector (a portion of the state_d vector)

qc_state = [x; x_d; y; y_d; z; z_d; gamma; beta; alpha];
qc_state_d = [x_d; x_dd; y_d; y_dd; z_d; z_dd; gamma_d; beta_d; alpha_d];
qc_input = [omega_x; omega_y; omega_z];
qc_output = [x_dd; y_dd; z_dd];
% f1
% f_qc = [x_d; thrust*(sin(alpha)*sin(gamma) + cos(alpha)*cos(gamma)*sin(beta)); y_d; -thrust*(cos(alpha)*sin(gamma) - cos(gamma)*sin(alpha)*sin(beta)); z_d; (thrust*cos(beta)*cos(gamma) - 9.81); 0; 0; 0];
% g1
% g_qc = [0 0 0; 0 0 0; 0 0 0; 0 0 0; 0 0 0; 0 0 0; (cos(gamma)/(cos(beta)*cos(gamma)^2 + cos(beta)*sin(gamma)^2)) (sin(gamma)/(cos(beta)*cos(gamma)^2 + cos(beta)*sin(gamma)^2)) 0; (-sin(gamma)/(cos(gamma)^2 + sin(gamma)^2)) (cos(gamma)/(cos(gamma)^2 + sin(gamma)^2)) 0; ((cos(gamma)*sin(beta))/(cos(beta)*cos(gamma)^2 + cos(beta)*sin(gamma)^2)) ((sin(beta)*sin(gamma))/(cos(beta)*cos(gamma)^2 + cos(beta)*sin(gamma)^2)) 1];
% g2
% g_qc = [0 0 0; 0 0 0; 0 0 0; 0 0 0; 0 0 0; 0 0 0; 1 ((sin(beta)*sin(gamma))/(cos(beta)*cos(gamma)^2 + cos(gamma)*sin(gamma)^2)) (sin(beta)/(sin(gamma)^2 + cos(beta)*cos(gamma))); 0 (cos(beta)/(sin(gamma)^2 + cos(beta)*cos(gamma))) ((-sin(gamma))/(sin(gamma)^2 + cos(beta)*cos(gamma))); 0 (sin(gamma)/(cos(beta)*cos(gamma)^2 + cos(gamma)*sin(gamma)^2)) (1/(sin(gamma)^2 + cos(beta)*cos(gamma)))];

% CURRENT NONLINEAR MODEL
qc_FofXU = [x_d; -U1/m*(cos(gamma)*sin(beta)*cos(alpha) + sin(gamma)*sin(alpha)); y_d; -U1/m * (cos(gamma)*sin(alpha)*sin(beta) - sin(gamma)*cos(alpha)); z_d; ((U1/m)*cos(beta)*cos(gamma) + g); gamma_d; (beta_d*alpha_d*((Iy-Iz)/Ix) - (Jr/Ix)*beta_d*omega_r + (L/Ix)*U2); beta_d; (gamma_d*alpha_d*((Iz-Ix)/Iy) + (Jr/Iy)*gamma_d*omega_r + (L/Iy)*U3); alpha_d; (gamma_d*beta_d*((Ix-Iy)/Iz) + (1/Iz)*U4)];

%% Linear transfer function of QC

% approximations around the equilibrium point (theta = 0, vertical up), for a small angle phi:
% cos(theta) = cos(phi) ~ 1
% sin(theta) = sin(phi) ~phi
% theta_d^2 = phi_d^2 ~ 0

x_dd_lin = thrust*(alpha*gamma + beta);
y_dd_lin = -thrust*(gamma - alpha*beta);
z_dd_lin = thrust - 981/100;

% can't take the Laplace transform of 3 variables, and taking the LT in
% terms of time is kind of pointless (doesn't help simplify)

%% Linearization of the nonlinear state space of QC

% quadcopter 
syms alpha_0 beta_0 gamma_0;
temp_f_xu = mtimes(g_qc, input);
f_xu = f_qc + temp_f_xu;               % this is equal to state_d
A_qc = [0 1 0 0 0 0 0 0 0; 0 0 0 0 0 0 (thrust*(cos(gamma_0)*sin(alpha) - cos(alpha)*sin(beta)*sin(gamma_0))) thrust*cos(alpha)*cos(beta_0)*cos(gamma) (thrust*(cos(alpha_0)*sin(gamma) - cos(gamma)*sin(alpha_0)*sin(beta))); 0 0 0 1 0 0 0 0 0; 0 0 0 0 0 0 (-thrust*(cos(alpha)*cos(gamma_0) + sin(alpha)*sin(beta)*sin(gamma_0))) thrust*cos(beta_0)*cos(gamma)*sin(alpha) (thrust*(sin(alpha_0)*sin(gamma) + cos(alpha_0)*cos(gamma)*sin(beta))); 0 0 0 0 0 1 0 0 0; 0 0 0 0 0 0 -thrust*cos(beta)*sin(gamma_0) -thrust*cos(gamma)*sin(beta_0) 0; 0 0 0 0 0 0 ((omega_y*cos(gamma_0)*sin(beta))/(cos(beta)*cos(gamma_0)^2 + cos(beta)*sin(gamma_0)^2) - (omega_x*sin(beta)*sin(gamma_0))/(cos(beta)*cos(gamma_0)^2 + cos(beta)*sin(gamma_0)^2)) ((omega_x*cos(beta_0)*cos(gamma))/(cos(beta_0)*cos(gamma)^2 + cos(beta_0)*sin(gamma)^2) + (omega_y*cos(beta_0)*sin(gamma))/(cos(beta_0)*cos(gamma)^2 + cos(beta_0)*sin(gamma)^2) + (omega_x*cos(gamma)*sin(beta_0)*(sin(beta_0)*cos(gamma)^2 + sin(beta_0)*sin(gamma)^2))/(cos(beta_0)*cos(gamma)^2 + cos(beta_0)*sin(gamma)^2)^2 + (omega_y*sin(beta_0)*sin(gamma)*(sin(beta_0)*cos(gamma)^2 + sin(beta_0)*sin(gamma)^2))/(cos(beta_0)*cos(gamma)^2 + cos(beta_0)*sin(gamma)^2)^2) 0; 0 0 0 0 0 0 (-(omega_x*cos(gamma_0))/(cos(gamma_0)^2 + sin(gamma_0)^2) - (omega_y*sin(gamma_0))/(cos(gamma_0)^2 + sin(gamma_0)^2)) 0 0; 0 0 0 0 0 0 ((omega_y*cos(gamma_0))/(cos(beta)*cos(gamma_0)^2 + cos(beta)*sin(gamma_0)^2) - (omega_x*sin(gamma_0))/(cos(beta)*cos(gamma_0)^2 + cos(beta)*sin(gamma_0)^2)) ((omega_x*cos(gamma)*(sin(beta_0)*cos(gamma)^2 + sin(beta_0)*sin(gamma)^2))/(cos(beta_0)*cos(gamma)^2 + cos(beta_0)*sin(gamma)^2)^2 + (omega_y*sin(gamma)*(sin(beta_0)*cos(gamma)^2 + sin(beta_0)*sin(gamma)^2))/(cos(beta_0)*cos(gamma)^2 + cos(beta_0)*sin(gamma)^2)^2) 0];
B_qc = [0 0 0; 0 0 0; 0 0 0; 0 0 0; 0 0 0; 0 0 0; cos(gamma_0)/(cos(beta)*cos(gamma_0)^2 + cos(beta)*sin(gamma_0)^2) sin(gamma)/(cos(beta_0)*cos(gamma)^2 + cos(beta_0)*sin(gamma)^2) 0; -sin(gamma_0)/(cos(gamma_0)^2 + sin(gamma_0)^2) cos(gamma)/(cos(gamma)^2 + sin(gamma)^2) 0; (cos(gamma_0)*sin(beta))/(cos(beta)*cos(gamma_0)^2 + cos(beta)*sin(gamma_0)^2) (sin(beta_0)*sin(gamma))/(cos(beta_0)*cos(gamma)^2 + cos(beta_0)*sin(gamma)^2) 1];
g_xu = [gamma_d; beta_d; alpha_d];                                          % assumes the model outputs the angular velocities, with x_dd, y_dd, z_dd still going to pendulum
C_qc = [0 0 0 0 0 0 1 0 0; 0 0 0 0 0 0 0 1 0; 0 0 0 0 0 0 0 0 1];           
D_qc = [0 0 0; 0 0 0; 0 0 0];

% % Solving for H(s)
% tf('s');
% i = eye(9);
% x1 = s*i - A_qc;
% x2 = mtimes(C_qc, x1);
% H = mtimes(x2, B_qc) + D_qc;        % returns a 3x3 matrix --> what does that mean?

% this won't really simplify the problem since the A, B, C and D matrices 
% will have the three angle varialbes in them
% also, because C = 0 and D = 0, the TF H(s) = C * (sI - A)^(-1) * B + D = 0

%% Current quadcopter model parameters

l_qa = 0.25;
l_r = 0.1;
m_arm = 0.01;
m_body = 0.5;
m_rotor = 0.05;
h_a = 0.0025;
h_b = 0.05;
h_r = 0.001;
r = 0.025;

m = m_arm + m_body;
Ixx = (4*m_arm/3) * (h_a^3*(l_qa-r) + h_a*(l_qa^3 - r^3)) + (m_body/3) * (r^3*h_b + r*h_b^3);
Iyy = Ixx;
Izz = (8*m_arm/3) * (l_qa^4 + r^4 - l_qa^3*r - l_qa*r^3) + (2*m_body/3)*r^4;

%Ix = [Ixx 0 0; 0 0 0; 0 0 0];
%Iy = [0 0 0; 0 Iyy 0; 0 0 0];
%Iz = [0 0 0; 0 0 0; 0 0 Izz];
%J = Ix + Iy + Iz;  
Ix = Ixx;
Iy = Iyy;
Iz = Izz;             
J = [Ixx 0 0; 0 Iyy 0; 0 0 Izz];                        % inertia tensor of quadcopter
Jr_x = (m_rotor/3)*(l_r^3*h_r + l_r*h_r^3);
Jr_y = (m_rotor/3)*(l_r^3*h_r + l_r*h_r^3);
Jr_z = (2*m_rotor/3)*l_r^4;
Jr_vector = 4*[Jr_x 0 0; 0 Jr_y 0; 0 0 Jr_z];           % intertia tensor of all 4 rotors
Jr = sqrt(Jr_vector(1,1)^2 + Jr_vector(2,2)^2 + Jr_vector(3,3)^2);

%% Motor constants

% F_rotor = k*omega_i^2
% k = (Ke*Ktau *sqrt(2*p*pi*l_r^2)/Kt)^2;
% where 
%   Ke = back EMF constant (V/RPM)
%   Kt = motor/torque constant (N.m/A)
%   Ktau = some other proportionality constant related to bland
%   configuration (sqrt(N.m)*s / (rad * sqrt(kg)))
% 
% I've experimentally determined through the model that equilibrium thrust
% per rotor is 1908.925 RPM = 200 rad/s
% From this and other paramters:
%   l_r = 0.1;      % m
%   p = 1.225;      % kg/m^3
%   Kv = 1/5700;    %V/rad/s
%   Kt = Kv;
% I determined:
%   k = 4.35e-5
%   Ktau = 0.02377

%% Nonlinear model of pendulum

r_dd = (1/((L^2 - s^2)*zeta)) * ( -r^4*x_dd - (L^2 - s^2)^2*x_dd - 2*r^2*(s*r_d*s_d + (-L^2 + s^2)*x_dd) + r^3*(s_d^2 + s*s_dd - zeta*(g+z_dd)) + r*(-L^2*s*s_dd + s^3*s_dd + s^2*(r_d^2 - zeta*(g+z_dd)) + L^2*(-r_d^2 - s_d^2 + zeta*(g+z_dd))) );
s_dd = (1/((L^2 - s^2)*zeta)) * ( -s^4*y_dd - (L^2 - r^2)^2*y_dd - 2*s^2*(r*r_d*s_d + (-L^2 + r^2)*y_dd) + s^3*(r_d^2 + r*r_dd - zeta*(g+z_dd)) + s*(-L^2*r*r_dd + r^3*r_dd + r^2*(s_d^2 - zeta*(g+z_dd)) + L^2*(-r_d^2 - s_d^2 + zeta*(g+z_dd))) );

pend_state = [r; r_d; s; s_d];
pend_state_d = [r_d; r_dd; s_d; s_dd];
pend_input = [x_dd; y_dd; z_dd];
pend_output = [r; s];

pend_FofXU = [r_d; r_dd; s_d; s_dd];