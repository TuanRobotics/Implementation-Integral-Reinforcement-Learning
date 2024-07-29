
% Main script for simulating the pendulum system
clc; clear;
% Parameters
global u; global Q; global R;
global M; global m; global g; global L; global weights;

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
x0 = [pi/50, 0, 0, 0, 0];  % x5 is value of cost function
x1 = [pi/25, 0.01, 0.01, 0.01, 0];
x2 = [pi/100, 0.001, 0, 0.01, 0];
x3 = [pi/30, 0.01, 0.001, 0.01, 0];
x4 = [pi/45, 0.002, 0, 0.007, 0];
initial_condition = [x0;x1;x2;x3;x4];
% initial_condition = [x0;x1;x2];
x0 = x2;
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

K = [-60.55 -8.789 -2.01 -1.6];
disp("Feasibility of stable gain K" )
pole = eig(A - B*K);
disp(pole)

weights = [0;2*K(1)/g0(2); 0; 0; K(2)/g0(2); 0; 0; 0; 2*K(3)/g0(4); K(4)/g0(4); 0; 0; 0; 0; 0; 0; 0; 0]; % 14x1
ww = [weights]; % tracking weights

%% Check fesibility w0
P= [weights(1) weights(2)/2  weights(3)/2  weights(4)/2;...
    weights(2)/2  weights(5)    weights(6)/2  weights(7)/2;...
    weights(3)/2  weights(6)/2  weights(8)    weights(9)/2;...
    weights(4)/2  weights(7)/2  weights(9)/2  weights(10)];

K = inv(R)*g0'*P;
disp("Eig of system when apply gain K0: ")
disp(eig(A-B*K))
%% Simulation
j = 0;
nop = 100;
Fsamples = 1000;
T = 0.08;
SS = T/1000;
uu = 0;
tt = 0; % tracking time

rpb_k = Fsamples - 200;
x00 = x0;

mm = 0; % chon intial condition

for k=1:Fsamples
    j = j + 1;
    tspan = 0:SS:T;  % Time range to simulate from 0 to 10 seconds
    % probing noise
%     if norm(x0(1:4),2) <= 0.2 && k <= rpb_k
%         x0 = [x0(1)+0.2*(rand(1,1)-0.5);x0(2);x0(3)+(rand(1,1)-0.5);x0(4); 0];
%     end
    [t,x]= ode45('irl_2ode_nonlinear',tspan,x0);
    tt = [tt; t + T*(k-1)];
    xx = [xx; x];
    phi = phi_fn(x0);
    phi_next = phi_fn(x(length(t),1:4));
    X(j,:) = phi - phi_next;
    Y(j,:) = x(length(t),5) - x(1,5);

    x0 = [x(length(t),1:4),x(length(t),5)];

    if mod(k,nop) == 0 && norm(X) > 0.001  && k < 400
        mm = mm + 1;
        w = inv(X'*X)*X'*Y;
        size_w = size(ww);
        % dieu kien update W
        if norm(w - ww(:, size_w(2))) > 0.5
            weights = w;
        end
        X = zeros(nop,18);
        Y = zeros(nop,1);
        j = 0;
        if mm < 5
            x0 = initial_condition(mm,:);
        end
    end
    ww = [ww weights];
    uu = [uu u];
end

%%
figure("Name", "State of system")
plot(tt,xx(:, 1:4), "linewidth", 1.5)
grid on
xlabel('Time (s)');
ylabel('rad');

% % Plotting the results
% figure(1);
% subplot(2, 1, 1);
% plot(tt, xx(:, 1), "linewidth", 1.5);
% grid on
% title('Угловое положение (theta)');
% xlabel('Time (s)');
% ylabel('rad');
% 
% subplot(2, 1, 2);
% plot(tt, xx(:, 2), "linewidth", 1.5);
% grid on
% title('Угловая скорость (theta dot)');
% xlabel('Время (с)');
% ylabel('rad/s');
% figure(1);
% 
%%
figure(2),  % weight
plot(T*((1:size(ww,2))-1),ww(1,:),'linewidth',1.5)
hold on
plot(T*((1:size(ww,2))-1),ww(2,:),'linewidth',1.5)
plot(T*((1:size(ww,2))-1),ww(3,:),'linewidth',1.5) 
plot(T*((1:size(ww,2))-1),ww(4,:),'linewidth',1.5)
plot(T*((1:size(ww,2))-1),ww(5,:),'linewidth',1.5)
plot(T*((1:size(ww,2))-1),ww(6,:),'linewidth',1.5)
plot(T*((1:size(ww,2))-1),ww(7,:),'linewidth',1.5)
plot(T*((1:size(ww,2))-1),ww(8,:),'linewidth',1.5)
plot(T*((1:size(ww,2))-1),ww(9,:),'linewidth',1.5)
plot(T*((1:size(ww,2))-1),ww(10,:),'linewidth',1.5)
plot(T*((1:size(ww,2))-1),ww(11,:),'linewidth',1.5)
plot(T*((1:size(ww,2))-1),ww(12,:),'linewidth',1.5)
plot(T*((1:size(ww,2))-1),ww(13,:),'linewidth',1.5)
plot(T*((1:size(ww,2))-1),ww(14,:),'linewidth',1.5)
plot(T*((1:size(ww,2))-1),ww(15,:),'linewidth',1.5)
plot(T*((1:size(ww,2))-1),ww(16,:),'linewidth',1.5)
plot(T*((1:size(ww,2))-1),ww(17,:),'linewidth',1.5)
plot(T*((1:size(ww,2))-1),ww(18,:),'linewidth',1.5)
grid on
title("Сходимости веса W")
xlabel('Время (с)');
% ylabel('Веса','Interpreter','latex');
legendEntries = {'$\\w_1$', '$\\w_2$', '$\\w_3$', '$\\w_4$', '$\\w_5$', '$\\w_6$', '$\\w_7$', '$\\w_8$', '$\\w_9$', '$\\w_{10}$', '$\\w_{11}$', '$\\w_{12}$', '$\\w_{13}$', '$\\w_{14}$', '$\\w_{15}$', '$\\w_{16}$', '$\\w_{17}$', '$\\w_{18}$'};
legend(legendEntries, 'interpreter', 'latex','FontSize', 10);
figure(2), hold off;
% 
% figure(3),  % weight
% plot(T*((1:size(uu,2))-1),uu(1,:),'linewidth',1.5)
% grid on
% xlabel('Время (с)');
% ylabel('Сигнал управления','Interpreter','latex');
% legendEntries = {'$\\u$'};
% legend(legendEntries, 'interpreter', 'latex','FontSize', 10);
% figure(3), hold off
% 
figure(4),
plot(tt, xx(:,5), "linewidth", 1.5)
grid on
xlabel('Время (с)');
ylabel('$J(x,u)$','Interpreter','latex');
legendEntries = {'$\\J(x,u)$'};
legend(legendEntries, 'interpreter', 'latex','FontSize', 10);
figure(4), hold off;