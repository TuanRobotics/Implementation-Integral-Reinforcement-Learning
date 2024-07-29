clc; clear;
% Parameters
global u; global Q; global R;
global M; global m; global g; global L; global weights;

M = 2.4;
m = 0.23;
g = 9.81;
L = 0.36;

A = [0 1 0 0;
    (M+m)*g/(M*L) 0 0 0;
    0 0 0 1;
    -m*g/M 0 0 0];

B = [     0;
     -1/(M*L);
          0;
        1/M];

% Q = [1 0.1 0 0;
%     0.1 1 0 0;
%     0 0 1 0.1;
%     0 0 0.1 1];
Q = [1 0 0 0;
    0 1 0 0;
    0 0 1 0;
    0 0 0 1];

R = 1;
x0 = [-pi, pi, 0.4, 0.2, 0];  % x5 is value of cost function

xx = [x0]; % tracking x

% cacl g(x) in x0
g2 = cos(x0(1))/(m*L*cos(x0(1))^2 - (M+m)*L);
g4 = 1/(M + m - m*cos(x0(1))^2);
g0 = [0;
    g2;
    0; 
    g4];
disp("g0 in points x0")
disp(g0)

K = -[25 5 10 5]; % 
disp("Feasibility of stable gain K" )
pole = eig(A + B*K);
disp(pole)

weights = [0;2*K(1)/g0(2); 0; 0; K(2)/g0(2); 0; 0; 0; 2*K(3)/g0(4); K(4)/g0(4)]; % 14x1
% ww = [weights]; % tracking weights
weights = [1 0 0 0 1 0 0 1 0 1]';
ww = [weights]; % tracking weights
%% Check fesibility w0
% dphi0 = nn_dphi(x0);
% u = -0.5*inv(R)*g0'*dphi0'*weights;
% 
% % P= [weights(1) weights(2)/2  weights(3)/2  weights(4)/2;...
% %     weights(2)/2  weights(5)    weights(6)/2  weights(7)/2;...
% %     weights(3)/2  weights(6)/2  weights(8)    weights(9)/2;...
% %     weights(4)/2  weights(7)/2  weights(9)/2  weights(10)];
% % K = inv(R)*g0'*P;
% 
% disp("Eig of system when apply gain K0: ")
% disp(eig(A+B*u))
%% Find addmisible control with using coeff Kp, Kd
Fsamples = 1000;
T = 0.1;
uu = 0;
t_span = [0, Fsamples*T];
[t, x] = ode45("irl_2ode_nonlinear", t_span, x0);
%% Plot state of system nonlinear inverted pendulum
figure("Name", "State of system")
plot(t,x(:, 1:4), "linewidth", 1.5)
grid on
legend("\theta", "\dot{\theta}", "x", "\dot{x}")
xlabel('Time (s)');
ylabel('rad');
%% Plot J
figure("Name", "J")
plot(t,x(:,5), "linewidth", 1.5)
grid on
legend("J")
title ("J")
xlabel('Time (s)');
ylabel('rad');