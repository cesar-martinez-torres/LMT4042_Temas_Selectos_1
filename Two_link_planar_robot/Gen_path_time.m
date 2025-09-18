%% Cubic trajectory for Revolute Joint
clear; clc;

% ===== Parameters =====
tf = 1500;       % number of points
T  = 30;         % TOTAL DURATION in seconds

% Time vector in seconds (column) with 'tf' samples from 0 to T
t = linspace(0, T, tf)';     % [tf x 1]

% ===== Boundary conditions =====
xd  = 45;                    % desired final position (degrees)
xpd = 0;                     % desired final velocity (degrees/s) -> we will use rad/s = 0
a0  = 0;                     % initial position condition (rad)
a1  = 0;                     % initial velocity condition (rad/s)

qf  = deg2rad(xd);           % convert to rad
dqf = 0;                     % rad/s

% ===== System for cubic polynomial coefficients =====
% q(t) = a0 + a1*t + a2*t^2 + a3*t^3
A = [ T^2    T^3 ;
      2*T  3*T^2 ];

B = [ qf - (a0 + a1*T) ;
      dqf - a1 ];

coef = inv(A) * B;          
a2 = coef(1);
a3 = coef(2);

% ===== Path generation =====
% We use the REAL time 't' in seconds (not 1..tf)
q  = a0 + a1*t + a2*t.^2 + a3*t.^3;         % position [rad] (tf x 1)
dq = a1 + 2*a2*t + 3*a3*t.^2;               % velocity [rad/s] (tf x 1)

% Signal for From Workspace (matrix [time, data])
path = [t q];    % <- Use 'path' in the From Workspace block

% ===== Quick plots =====
figure('Name','Trajectories in th1','NumberTitle','off');
subplot(2,1,1);
plot(t, q); grid on;
xlabel('Time [s]'); ylabel('q [rad]'); title('Position');
subplot(2,1,2);
plot(t, dq); grid on;
xlabel('Time [s]'); ylabel('dq/dt [rad/s]'); title('Velocity');


