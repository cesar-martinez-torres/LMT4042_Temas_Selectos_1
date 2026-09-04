%% Mass-Spring-Damper System with ode45
clear; clc; close all;

m = 1;
c = 5;
k = 400;

F = @(t) 10 * (t >= 0.25);
dt = 0.002;
tspan = 0:dt:5;

ecuaciones = @(t, x) [
    x(2);
    (F(t) - c*x(2) - k*x(1))/m
];

[t, x] = ode45(ecuaciones, tspan, [0; 0]);

simOut = sim("MRA.slx");
x_sim = simOut.x_sim;
error_x = x(:,1) - x_sim.Data(:);

subplot(3,1,1)
plot(t, error_x, 'LineWidth', 1.5)
xlabel('Time [s]')
ylabel('Error [m]')
grid on

subplot(3,1,2)
plot(t, x(:,1), 'LineWidth', 1.5)
xlabel('Time [s]')
ylabel('MATLAB x [m]')
grid on

subplot(3,1,3)
plot(x_sim.Time, x_sim.Data, 'LineWidth', 1.5)
xlabel('Time [s]')
ylabel('Simulink x [m]')
grid on
