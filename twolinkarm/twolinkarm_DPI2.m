%% COG and inertias

%% Define joint variables and constants

syms th1 th1_d th1_dd th2 th2_d th2_dd

q = [th1; th2];
q_d = [th1_d; th2_d];
q_dd = [th1_dd; th2_dd];

z0 = [0; 0; 1];

% number of augmented links
n = 2;

% link dimensions
syms lC1 lC2 a1 a2

% rotation matrices from frame i-1 to i
% note in this case 0 is the world frame, 1 is frame at M1
R = sym(zeros(3,3, n));
R(:,:,1) = Rz(th1);
R(:,:,2) = Rz(th2);
R(:,:,3) = eye(3,3);

% displacements from frame i to CoMs[i] in frame i
syms r1c1x r1c1y r1c1z r2c2x r2c2y r2c2z
r1c1 = [r1c1x; r1c1y; r1c1z]; % L1
r2c2 = [r2c2x; r2c2y; r2c2z]; % L2
rCi = sym(zeros(3,1,n));
rCi(:,:,1) = r1c1;
rCi(:,:,2) = r2c2;

% displacements from frame i to CoMs[i] in frame i
r1c1_relation = [-lC1; 0; 0]; % L1
r2c2_relation = [-lC2; 0; 0]; % L2
rCi_relation(:,:,1) = r1c1_relation;
rCi_relation(:,:,2) = r2c2_relation;

% displacements from frame i-1 to frame i in frame i
r01 = [a1; 0; 0];
r12 = [a2; 0; 0];
ri = sym(zeros(3,1,n));
ri(:,:,1) = r01;
ri(:,:,2) = r12;

% gear ratios, starting at frame 1
% added extra entry for forloop
syms k1 k2
k = [k1; k2; 0];

% motor axes of motor i relative to frame i-1
% added extra entry for forloop
zm1 = z0;
zm2 = z0;
zmi(:,:,1) = zm1;
zmi(:,:,2) = zm2;
zmi(:,:,3) = zeros(3,1);

% joint type, 1 = rev, 0 = pris
jointType = [1; 1];

%% Get masses and params from laparobot

% syms for augmented links
syms m1 m1_lC1x m1_lC1y m1_lC1z I1xx I1xy I1xz I1yy I1yz I1zz IM1;
syms m2 m2_lC2x m2_lC2y m2_lC2z I2xx I2xy I2xz I2yy I2yz I2zz IM2;

% syms for links and masses
syms mL1 mL2 mM1 mM2
syms IL1xx IL1xy IL1xz IL1yy IL1yz IL1zz
syms IL2xx IL2xy IL2xz IL2yy IL2yz IL2zz

% masses of augmented links
mi = [m1; m2];
mi_relation = [mL1 + mM2; mL2];

% inertias of motors
% ignore Imxx and Imyy components since not needed for DPI
% added extra entry for forloop
IMi = sym(zeros(3,3, n+1));
IMi(:,:,1) = [0 0 0; 0 0 0; 0 0 IM1];
IMi(:,:,2) = [0 0 0; 0 0 0; 0 0 IM2];
IMi(:,:,3) = zeros(3,3);

% inertias of links
ILi = sym(zeros(3,3, n));
ILi(:,:,1) = [IL1xx -IL1xy -IL1xz; -IL1xy IL1yy -IL1yz; -IL1xz -IL1yz IL1zz];
ILi(:,:,2) = [IL2xx -IL2xy -IL2xz; -IL2xy IL2yy -IL2yz; -IL2xz -IL2yz IL2zz];

% inertias of augmented links
Ii = sym(zeros(3,3, n));
Ii(:,:,1) = [I1xx -I1xy -I1xz; -I1xy I1yy -I1yz; -I1xz -I1yz I1zz];
Ii(:,:,2) = [I2xx -I2xy -I2xz; -I2xy I2yy -I2yz; -I2xz -I2yz I2zz];

% relationship of inertias of augmented links
% Ii = ILi + IMi+1 + mi*(S(rCi).'*S(rCi))
Ii_relation = sym(zeros(3,3, n));
Ii_relation(:,:,1) = ILi(:,:,1) + IMi(:,:,2) ...
    + mi_relation(1)*(getS(rCi(:,:,1)).' * getS(rCi(:,:,1)));
Ii_relation(:,:,2) = ILi(:,:,2) ...
    + mi_relation(1)*(getS(rCi(:,:,2)).' * getS(rCi(:,:,2)));

%% Forward recursion - calculate w, w_d, pi_dd, pCi_dd, wm_d

% variables to calculate
w = sym(zeros(3,1,n)); % ang velocity of aug links relative to frame i
w_d = sym(zeros(3,1,n)); % ang acceleration of aug links relative to frame i
pi_dd = sym(zeros(3,1,n)); % linear acceleration of origin of frame i relative to frame i
pCi_dd = sym(zeros(3,1,n)); % linear acceleration of aug link CoM relative to frame i
wm_d = sym(zeros(3,1,n)); % ang acceleration of motor i relative to frame i-1

% initial conditions
syms g
w00 = zeros(3,1);
w00_d = zeros(3,1);
p00_dd = zeros(3,1) - [0; g; 0];

for i = 1:n
    
    if i == 1            
        w_prev = w00;   
        w_d_prev = w00_d;
        pi_dd_prev = p00_dd;
    else            
        w_prev = w(:,:,i-1);
        w_d_prev = w_d(:,:,i-1);
        pi_dd_prev = pi_dd(:,:,i-1);
    end
    
    % diff calculations for diff joint types
    if jointType(i) == 1        % revolute        
        
        w(:,:,i) = R(:,:,i).' * (w_prev + q_d(i) * z0);
        w_d(:,:,i) = R(:,:,i).' * (w_d_prev + q_dd(i) * z0 ...
            + cross(q_d(i)*w_prev, z0)); 
        pi_dd(:,:,i) = R(:,:,i).' * pi_dd_prev ...
            + cross(w_d(:,:,i), ri(:,:,i)) ...
            + cross(w(:,:,i), cross(w(:,:,i), ri(:,:,i)));
        
    else                        % prismatic
        
        w(:,:,i) = R(:,:,i).' * w_prev;
        w_d(:,:,i) = R(:,:,i).' * w_d_prev;
        pi_dd(:,:,i) = R(:,:,i).' * (pi_dd_prev + q_dd(i)*z0) ... 
            + cross(2*q_d(i)*w(:,:,i), R(:,:,i).'*z0) ...
            + cross(w_d(:,:,i), ri(:,:,i)) ...
            + cross(w(:,:,i), cross(w(:,:,i), ri(:,:,i)));    
        
        
    end
    
    % same calculations for all joint types
    pCi_dd(:,:,i) = pi_dd(:,:,i) ...
        + cross(w_d(:,:,i), rCi(:,:,i)) ...
        + cross(w(:,:,i), cross(w(:,:,i), rCi(:,:,i)));
    wm_d(:,:,i) = w_d_prev + k(i)*q_dd(i)*zmi(:,:,i) ...
        + cross(k(i)*q_d(i)*w_prev, zmi(:,:,i));
        
end

%% Backward recursion, find fi, ui, ti

% variables to calculate
fi = sym(zeros(3,1,n)); % the force exerted by link i-1 on link i, in frame i
ui = sym(zeros(3,1,n)); % the moment exerted by link i-1 on link i, in frame i
ti = sym(zeros(n,1));  % the generalized force at joint i, 
                    % in frame i (fi or ui + rotor inertia trq)

% initial conditions
fEE = zeros(3,1); % f at n+1, at end effector
uEE = zeros(3,1); % u at n+1, at end effector

% undetermined symbolic variables
syms fv1 fv2 fs1 fs2
Fv = [fv1; fv2];
Fs = [fs1; fs2];

for i = n:-1:1
    
    if i == n
        f_prev = fEE;
        u_prev = uEE;
        q_dd_prev = 0;
        q_d_prev = 0;
    else
        f_prev = fi(:,:,i+1);
        u_prev = ui(:,:,i+1);
        q_dd_prev = q_dd(i+1);
        q_d_prev = q_d(i+1);
    end
   
    % same calculations for all joint types
    fi(:,:,i) = R(:,:,i) * f_prev + mi(i) * pCi_dd(:,:,i);
    ui(:,:,i) = cross(-fi(:,:,i), ri(:,:,i) + rCi(:,:,i)) ...
        + R(:,:,i+1) *  u_prev ...
        + cross(R(:,:,i+1) * f_prev, rCi(:,:,i)) ...
        + Ii(:,:,i) * w_d(:,:,i) ...
        + cross(w(:,:,i), Ii(:,:,i) * w(:,:,i)) ... + cross(w(:,:,i), Ii(:,:,i) * w(:,:,i)) ...
        + k(i+1)*q_dd_prev*IMi(3,3,i+1)*zmi(:,:,i+1) ...
        + cross(k(i+1)*q_d_prev*IMi(3,3,i+1)*w(:,:,i),zmi(:,:,i+1));
    
    if jointType(i) == 1                % revolute
        
        ti(i) = ui(:,:,i).' * R(:,:,i).' * z0 ...
            + k(i) * IMi(3,3,i) * wm_d(:,:,i).' * zmi(:,:,i) ...
            + Fv(i)*q_d(i) + Fs(i)*sign(q_d(i));
        
    else                                % prismatic
        
        ti(i) = fi(:,:,i).' * R(:,:,i).' * z0 ...
            + k(i) * IMi(3,3,i) * wm_d(:,:,i).' * zmi(:,:,i) ...
            + Fv(i)*q_d(i) + Fs(i)*sign(q_d(i));
        
    end
    
end

% expand ti so that we can find mi*riCi terms in next section
ti = expand(ti);

%% Linearize

pi1 = [m1; m1_lC1x; m1_lC1y; m1_lC1z; I1xx; I1xy; I1xz; I1yy; I1yz; I1zz; IM1; fv1; fs1];
pi2 = [m2; m2_lC2x; m2_lC2y; m2_lC2z; I2xx; I2xy; I2xz; I2yy; I2yz; I2zz; IM2; fv2; fs2];

% replace mi*rCi with symbol mi_lCi in order to linearize
ti = subs(ti, ...
    [m1*rCi(1,1,1), m1*rCi(2,1,1), m1*rCi(3,1,1), ...
    m2*rCi(1,1,2), m2*rCi(2,1,2), m2*rCi(3,1,2)], ...
    [m1_lC1x, m1_lC1y, m1_lC1z, ...
    m2_lC2x, m2_lC2y, m2_lC2z]);

PI = vertcat(pi1,pi2);

syms tau1 tau2

eqn1 = ti(1) == tau1;
eqn2 = ti(2) == tau2;

[Y, tau] = equationsToMatrix([eqn1, eqn2], PI);

% Y = simplify(Y);
% tau = simplify(tau);

%% Remove columns with zeros

p = size(Y,2);
i = 1;

while i < p
    
    if norm(Y(:,i)) == 0   % remove that column
        
        Y = [Y(:,1:i-1) Y(:, i+1:p)];
        PI = [PI(1:i-1); PI(i+1:p)];
        p = p-1;
        
    else
        
        i = i+1;
        
    end   
        
end

%% Get back original terms

% sub in mi*rCi
Y = subs(Y, ...
    [m1_lC1x, m1_lC1y, m1_lC1z, ...
    m2_lC2x, m2_lC2y, m2_lC2z], ...
    [m1*rCi(1,1,1), m1*rCi(2,1,1), m1*rCi(3,1,1), ...
    m2*rCi(1,1,2), m2*rCi(2,1,2), m2*rCi(3,1,2)]);
Y = subs(Y, ...
    [r1c1x r1c1y r1c1z r2c2x r2c2y r2c2z], ...
    [rCi_relation(1,1,1), rCi_relation(2,1,1), rCi_relation(3,1,1), ...
    rCi_relation(1,1,2), rCi_relation(2,1,2), rCi_relation(3,1,2)]);  
Y=subs(Y, [lC1, lC2],[0.5*a1, 0.5*a2]);

% PI = subs(PI, ...
%     [m1_lC1x, m1_lC1y, m1_lC1z, ...
%     m2_lC2x, m2_lC2y, m2_lC2z], ...
%     [m1*rCi(1,1,1), m1*rCi(2,1,1), m1*rCi(3,1,1), ...
%     m2*rCi(1,1,2), m2*rCi(2,1,2), m2*rCi(3,1,2)]);
% PI = subs(PI, ...
%     [r1c1x r1c1y r1c1z r2c2x r2c2y r2c2z], ...
%     [rCi_relation(1,1,1), rCi_relation(2,1,1), rCi_relation(3,1,1), ...
%     rCi_relation(1,1,2), rCi_relation(2,1,2), rCi_relation(3,1,2)]); 

% % sub in masses
% Y = subs(Y, [m1, m2], [mi_relation(1), mi_relation(2)]);
% PI = subs(PI, [m1, m2], [mi_relation(1), mi_relation(2)]);
% 
% % sub in inertias
% Y = subs(Y, ...
%     [I1xx, I1xy, I1xz, I1yy, I1yz, I1zz, ...
%     I2xx, I2xy, I2xz, I2yy, I2yz, I2zz], ...
%     [Ii_relation(1,1,1), Ii_relation(1,2,1), Ii_relation(1,3,1), ...
%     Ii_relation(2,2,1), Ii_relation(2,3,1), Ii_relation(3,3,1), ...
%     Ii_relation(1,1,2), Ii_relation(1,2,2), Ii_relation(1,3,2), ...
%     Ii_relation(2,2,2), Ii_relation(2,3,2), Ii_relation(3,3,2)]);
% PI = subs(PI, ...
%     [I1xx, I1xy, I1xz, I1yy, I1yz, I1zz, ...
%     I2xx, I2xy, I2xz, I2yy, I2yz, I2zz], ...
%     [Ii_relation(1,1,1), Ii_relation(1,2,1), Ii_relation(1,3,1), ...
%     Ii_relation(2,2,1), Ii_relation(2,3,1), Ii_relation(3,3,1), ...
%     Ii_relation(1,1,2), Ii_relation(1,2,2), Ii_relation(1,3,2), ...
%     Ii_relation(2,2,2), Ii_relation(2,3,2), Ii_relation(3,3,2)]);