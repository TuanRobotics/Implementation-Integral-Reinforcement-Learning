% doi voi he nonlinear ap dung IRL phai them noise khi x xap xi bang 0 de
% no co the hoc duoc mot cach tot nhat
% Main script for simulating the pendulum system
% Neu khong them noise: Mac du he thong on dinh nhung gia tri dieu khien
% khong hoi tu ve gia tri toi uu

% Dung quadratoc cost thi cac nghim cua phuon trinh hoi tu ve gia tri toi
% uu nhu doi voi he tuyen tinh
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
% Q = [1 0.1 0 0;
%     0.1 1 0 0;
%     0 0 1 0.1;
%     0 0 0.1 1];
Q = [1 0 0 0;
    0 1 0 0;
    0 0 1 0;
    0 0 0 1];
R = 1;
% x0 = [-pi, pi, 0.4, 0.8, 0];  % x5 is value of cost function

xx = [x0]; % tracking x

% % cacl g(x) in x0
g2 = cos(x0(1))/(m*L*cos(x0(1))^2 - (M+m)*L);
g4 = 1/(M + m - m*cos(x0(1))^2);
g0 = [0;
    g2;
    0; 
    g4];
disp("g0 in points x0")
disp(g0)

% K = [25 5 10 5]; % 

% weights = [0;2*K(1)/g0(2); 0; 0; K(2)/g0(2); 0; 0; 0; 2*K(3)/g0(4); K(4)/g0(4)]; % 14x1
% ww = [weights]; % tracking weights
weights = [1 0 0 0 1 0 0 1 0 1]';
ww = [weights]; % tracking weights

%% Check fesibility w0
% P= [weights(1) weights(2)/2  weights(3)/2  weights(4)/2;...
%     weights(2)/2  weights(5)    weights(6)/2  weights(7)/2;...
%     weights(3)/2  weights(6)/2  weights(8)    weights(9)/2;...
%     weights(4)/2  weights(7)/2  weights(9)/2  weights(10)];
% 
% K = inv(R)*g0'*P;
% disp("Eig of system when apply gain K0: ")
% disp(eig(A-B*K))
%% Simulation
% Fsamples = 500;
% T = 0.05;
% uu = 0;
% t_span = [0, Fsamples*T];
% [t, x] = ode45("irl_2ode_nonlinear", t_span, x0);
%% %% Plot state of system nonlinear inverted pendulum
% figure("Name", "State of system")
% plot(t,x(:, 1:4), "linewidth", 1.5)
% grid on
% xlabel('Time (s)');
% ylabel('rad');
%% Simulation IRL
Fsamples = 300;
T = 0.05;
nop = 20;
SS = T/10000;
tt = 0;
tick = 0;
tt_weight = 0;
j = 0;
% for noise
rpb_k = 300;

for k = 1:Fsamples
    j = j+1;
    tspan = 0:SS:T;
    tick = tick + T;
%   probing noise
%     if norm(x0(1:4),2) <= 0.15 && k <= rpb_k || norm(x0(1:4),2) > 10
%         x0 = [x0(1)+0.15*(rand(1,1)-0.5);x0(2);x0(3)+0.15*(rand(1,1)-0.5);x0(4); 0];
%     end
    [t,x] = ode45("irl_2ode_nonlinear", tspan, x0);
    tt = [tt; t + T*(k-1)];
    xx = [xx; x];
    % param for learning
    phi = phi_fn(x0);
    phi_next = phi_fn(x(length(t),1:4));
    X(j,:) = phi - phi_next;
    Y(j,:) = x(length(t),5) - x(1,5);
    x0 = [x(length(t),1:4),x(length(t),5)];
    
    if mod(k,nop) == 0 
        if norm(X) > 0.1
            w = inv(X'*X)*X'*Y;
        end 

        size_w = size(ww);

        % update weights
        if norm(w - ww(:, size_w(2))) > 0.5  
            weights = w;
        end

        ww = [ww weights];
        tt_weight = [tt_weight; tick];

        X = zeros(nop,10);
        Y = zeros(nop,1);
        j = 0;
    end
end

%% Plot state of system nonlinear inverted pendulum
figure("Name", "State of system")
plot(tt,xx(:, 1:4), "linewidth", 1.5)
grid on
legend("\theta", "\dot{\theta}", "x", "\dot{x}", 'interpreter', 'latex','FontSize', 10)
xlabel('Time (s)');
ylabel('rad');
%% Plot convergen process of W
figure(2),  % weight
plot(tt_weight, ww(1,:),'-*', 'LineWidth', 1.5)
hold on
plot(tt_weight,ww(2,:),'-*','linewidth',1.5)
plot(tt_weight,ww(3,:),'-*','linewidth',1.5) 
plot(tt_weight,ww(4,:),'-*','linewidth',1.5)
plot(tt_weight,ww(5,:),'-*','linewidth',1.5)
plot(tt_weight,ww(6,:),'-*','linewidth',1.5)
plot(tt_weight,ww(7,:),'-*','linewidth',1.5)
plot(tt_weight,ww(8,:),'-*','linewidth',1.5)
plot(tt_weight,ww(9,:),'-*','linewidth',1.5)
plot(tt_weight,ww(10,:),'-*','linewidth',1.5)
% plot(T*((1:size(ww,2))-1),ww(11,:),'linewidth',1.5)
% plot(T*((1:size(ww,2))-1),ww(12,:),'linewidth',1.5)
% plot(T*((1:size(ww,2))-1),ww(13,:),'linewidth',1.5)
% plot(T*((1:size(ww,2))-1),ww(14,:),'linewidth',1.5)
% plot(T*((1:size(ww,2))-1),ww(15,:),'linewidth',1.5)
% plot(T*((1:size(ww,2))-1),ww(16,:),'linewidth',1.5)
% plot(T*((1:size(ww,2))-1),ww(17,:),'linewidth',1.5)
% plot(T*((1:size(ww,2))-1),ww(18,:),'linewidth',1.5)
grid on
title("Сходимости веса W")
xlabel('Время (с)');
% ylabel('Веса','Interpreter','latex');
legendEntries = {'$\\w_1$', '$\\w_2$', '$\\w_3$', '$\\w_4$', '$\\w_5$', '$\\w_6$', '$\\w_7$', '$\\w_8$', '$\\w_9$', '$\\w_{10}$', '$\\w_{11}$', '$\\w_{12}$', '$\\w_{13}$', '$\\w_{14}$', '$\\w_{15}$', '$\\w_{16}$', '$\\w_{17}$', '$\\w_{18}$'};
legend(legendEntries, 'interpreter', 'latex','FontSize', 10);
figure(2), hold off;

%% Plot value of cost function J
figure(4),
plot(tt, xx(:,5), "linewidth", 1.5)
grid on
xlabel('Время (с)');
ylabel('$J(x,u)$','Interpreter','latex');
legendEntries = {'$\\J(x,u)$'};
legend(legendEntries, 'interpreter', 'latex','FontSize', 10);
figure(4), hold off;