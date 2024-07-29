
% Main script for simulating the pendulum system
clc; clear;
% Parameters
global u;global Q; global R; global w;
m = 1.0;  % Mass of the pendulum in kg
g = 9.81; % Gravitational acceleration in m/s^2
l = 1.0;  % Length of the pendulum rod in meters
B = 0.5;  % Damping coefficient
Q = [1 0;0 1]; R = 1;
w = [0 2 0]';
ww = [w];
% Initial conditions [theta, theta_dot]
initial_conditions = [pi/6, 0,0];  % Initial angle (45 degrees) and angular velocity (0)
tt = 0;
xx = [initial_conditions];
j = 0;
nop = 3;
Fsamples = 300;
T = 0.03;
uu = 0;
for k=1:Fsamples
    j = j + 1;
    % Time span for simulation
    t_span = [0, T];  % Time range to simulate from 0 to 10 seconds
    % ODE solver
    [t, x] = ode45(@(t, x) pendulum_dynamics(t, x, m, g, l, B), t_span, initial_conditions);
    tt = [tt; t + T*(k-1)];
    xx = [xx; x];
    phi = phi_fn(initial_conditions);
    phi_next = phi_fn(x(length(t),1:2));
    X(j,:) = phi - phi_next;
    Y(j,:) = x(length(t),3) - x(1,3);
    initial_conditions = [x(length(t),1:2),x(length(t),3)];

    if mod(k,nop) == 0 && norm(X) > 0.0001
        w = pinv(X)*Y;
        X = zeros(nop,3);
        Y = zeros(nop,1);
        j = 0;
    end
    ww = [ww w];
    uu = [uu u];
end

% Plotting the results
figure(1);
subplot(2, 1, 1);
plot(tt, xx(:, 1), "linewidth", 1.5);
grid on
title('Угловое положение (theta)');
xlabel('Time (s)');
ylabel('rad');

subplot(2, 1, 2);
plot(tt, xx(:, 2), "linewidth", 1.5);
grid on
title('Угловая скорость (theta dot)');
xlabel('Время (с)');
ylabel('rad/s');
figure(1);

figure(2),  % weight
plot(T*((1:size(ww,2))-1),ww(1,:),'linewidth',1.5)
hold on
plot(T*((1:size(ww,2))-1),ww(2,:),'linewidth',1.5)
plot(T*((1:size(ww,2))-1),ww(3,:),'linewidth',1.5) 
grid on
title("Сходимости веса W")
xlabel('Время (с)');
% ylabel('Веса','Interpreter','latex');
legendEntries = {'$\\w_1$', '$\\w_2$', '$\\w_3$'};
legend(legendEntries, 'interpreter', 'latex','FontSize', 10);
figure(2), hold off;

figure(3),  % weight
plot(T*((1:size(uu,2))-1),uu(1,:),'linewidth',1.5)
grid on
xlabel('Время (с)');
ylabel('Сигнал управления','Interpreter','latex');
legendEntries = {'$\\u$'};
legend(legendEntries, 'interpreter', 'latex','FontSize', 10);
figure(3), hold off

figure(4),
plot(tt, xx(:,3), "linewidth", 1.5)
grid on
xlabel('Время (с)');
ylabel('$J(x,u)$','Interpreter','latex');
legendEntries = {'$\\J(x,u)$'};
legend(legendEntries, 'interpreter', 'latex','FontSize', 10);
figure(4), hold off;


function dxdt = pendulum_dynamics(t, x, m, g, l, B)
global u; global Q; global R; global w;

    x = [x(1) x(2)]';
    % State variables
    theta = x(1);      % angular position
    theta_dot = x(2);   % angular velocity
    gx = [0; 1/(m*l^2)];
    dphi = phi_dot(x);
%     u = 0;
    u = -0.5*R^(-1)*gx'*dphi'*w;

    % Derivative of the state variables
    dxdt = [
        theta_dot;  % d(theta)/dt
        (-m*g*l*sin(theta) - B*theta_dot) / (m*l^2)  % d(theta_dot)/dt
    ]+ gx*u;
    dxdt = [dxdt; x'*Q*x + u'*R*u];
end