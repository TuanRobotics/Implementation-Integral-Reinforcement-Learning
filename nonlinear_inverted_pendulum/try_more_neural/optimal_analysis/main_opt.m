clc; clear;
% Parameters
global u; global Q; global R;
global M; global m; global g; global L; global weights_opt; global weights;

M = 2.4;
m = 0.23;
g = 9.81;
L = 0.46;

A = [0 1 0 0;
    (M+m)*g/(M*L) 0 0 0;
    0 0 0 1;
    -m*g/M 0 0 0];

B = [     0;
     -1/(M*L);
          0;
        1/M];

Q = [1 0.1 0 0;
    0.1 1 0 0;
    0 0 1 0.1;
    0 0 0.1 1];

R = 1;
% x0 = [-pi/15, pi, pi, 1, 0];  % x5 is value of cost function
% x0 = [pi/10, pi, 10, 2, 0];
% x0 = [pi/20, pi, pi, 1, 0];
x0 = [-pi/15, pi, pi, 1, 0];
 
weights_opt = [407.77 171.9187 26.0863 71.7377 17.7814 5.7069 15.2537 2.8321 7.5740 9.6739 4.2314 -0.1121 0 -0.011 536.8 4.8117 -0.2249 -0.4019]';
%% Simulation with weight optima - IRL
Fsamples = 400;
T = 0.05;
uu = 0;
t_span = [0, Fsamples*T];
[t, x] = ode45("irl_2ode_nonlinear_opt", t_span, x0);
%% Simulation with initial weights - no learning
g2 = cos(x0(1))/(m*L*cos(x0(1))^2 - (M+m)*L);
g4 = 1/(M + m - m*cos(x0(1))^2);
g0 = [0;
    g2;
    0; 
    g4];
K = [-60.55 -8.789 -2.01 -1.6];
% this weight is not optimal, this is initial admissible weight for
% conducting algorithm
weights = [0;2*K(1)/g0(2); 0; 0; K(2)/g0(2); 0; 0; 0; 2*K(3)/g0(4); K(4)/g0(4)]; % 14x1
Fsamples = 400;
T = 0.05;
uu = 0;
t_span = [0, Fsamples*T];
[t1, x1] = ode45("irl_2ode_nonlinear", t_span, x0);
%% plot with no learning
%% plot state optimal
% figure("Name", "State of system")
% plot(t,x(:, 1:4), "linewidth", 1.5)
% grid on
% xlabel('Time (s)');
% ylabel('rad');
% legendEntries = {'$\\\theta_{IRL}$', '$\dot{\theta}_{IRL}$', '$\\x_{IRL}$','$\dot{x}_{IRL}$'};
% legend(legendEntries, 'interpreter', 'latex','FontSize', 14);
% %% Plot J with optimal control
% figure("Name", "Cost function J(x,u)"),
% plot(t, x(:,5), "linewidth", 1.5)
% grid on
% xlabel('Время (с)');
% title('$J(x,u)$','Interpreter','latex');
% legendEntries = {'с обучением'};
% legend(legendEntries, 'interpreter', 'latex','FontSize', 10);
%% plot state
%% plot J(x,u) comparision
figure("Name", "Cost function J(x,u)"),
plot(t, x(:,5), "linewidth", 1.5)
hold on
plot(t1, x1(:,5), "linewidth", 1.5)
grid on
xlabel('Время (с)');
title('$J(x,u)$','Interpreter','latex');
legendEntries = {'с обучением', "без обучения"};
legend(legendEntries, 'interpreter', 'latex','FontSize', 10);
%% plot comparision state
figure("Name", "State compare");
subplot(2, 2, 1);
plot(t, x(:, 1), "linewidth", 1.5);
hold on
plot(t1, x1(:,1), "linewidth", 1.5)
grid on
title('$\theta$', "Interpreter","Latex", 'FontSize', 14);
xlabel('Время (с)')
legendEntries = {'с обучением', "без обучения"};
legend(legendEntries, 'interpreter', 'latex','FontSize', 10);
% ylabel('rad');

subplot(2, 2, 2);
plot(t, x(:, 2), "linewidth", 1.5);
hold on
plot(t1, x1(:,2), "linewidth", 1.5)
grid on
title('$\dot{\theta}$', "Interpreter","latex", 'FontSize', 14);
xlabel('Время (с)');
legendEntries = {'с обучением', "без обучения"};
legend(legendEntries, 'interpreter', 'latex','FontSize', 10);
% ylabel('rad/s');

subplot(2, 2, 3);
plot(t, x(:, 3), "linewidth", 1.5);
hold on
plot(t1, x1(:,3), "linewidth", 1.5)
grid on
title('x', "Interpreter","latex", 'FontSize', 14);
xlabel('Время (с)');
legendEntries = {'с обучением', "без обучения"};
legend(legendEntries, 'interpreter', 'latex','FontSize', 10);
% ylabel('m');

subplot(2, 2, 4);
plot(t, x(:, 4), "linewidth", 1.5);
hold on
plot(t1, x1(:,4), "linewidth", 1.5)
grid on
title('$\dot{x}$', "Interpreter","latex", 'FontSize', 14);
xlabel('Время (с)');
legendEntries = {'с обучением', "без обучения"};
legend(legendEntries, 'interpreter', 'latex','FontSize', 10);
% ylabel('m/s');







