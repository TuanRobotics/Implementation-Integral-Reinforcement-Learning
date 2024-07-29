% Для линейных систем время отбора выборки данных сильно зависит от начальных условий системы. То есть не любое значение времени выборки данных приведет к сходимости системы к оптимальному значению.
% Для нелинейных систем это становится сложнее (на мой взгляд). И необходимо больше нейронов, чтобы система сходилась к оптимальному решению.


clear; close all; clc;

% parameters
global K; global u; global A; global B; global Q; global R; global P;
global Kopt; global Popt;

M = 2.4;
m = 0.23;
g = 9.81;
L = 0.46;
% M = 1;
% m = 0.1;
% L = 1;
% g = 9.81;


P = zeros(4,4);
uu = 0; % record control signal u


A = [0 1 0 0;
    (M+m)*g/(M*L) 0 0 0;
    0 0 0 1;
    -m*g/M 0 0 0];

B = [     0;
     -1/(M*L);
          0;
        1/M];

disp("eig of A without control signals")
disp(eig(A))
disp("A,B controlable check: ")
if rank(ctrb(A,B)) == 4
    disp("Oke")
else
    disp("No!")
    disp(rank(ctrb(A,B)))
end

% K0 = [-79.6904 -16.5960 -6.7523 -9.6783];
K = [-60.55 -8.789 -2.01 -1.6];
disp("Feasibility of stable gain K" )
pole = eig(A - B*K);
disp(pole)

w0 = [0;K(1)/B(2); 0; 0; K(2)/B(2); 0; 0; 0; K(3)/B(4); K(4)/B(4)];
ww = [w0];

Q = [1 0.1 0 0;
    0.1 1 0 0;
    0 0 1 0.1;
    0 0 0.1 1];

R = 1;
disp("check strq(Q), A detectabble")

if rank(obsv(sqrt(Q), A)) == 4
    disp("Oke")
else
    disp("Not")
    disp(rank(obsv(sqrt(Q), A)))
end

[Kopt, Popt] = lqr(A,B,Q,R);
disp("Optimal gain LQR")
disp(Kopt)

disp("Check contorlability of system: ")
disp(rank(ctrb(A,B)))

% K = [0 0 0 2];
x0 = [pi 1 2 1 0];
xx = [x0];
% simulation parameters
% Fsamples = 125;
Fsamples = 125;
T = 0.08;
SS = T/10000;
j = 0;
nop = 10;
tol = 1e-4;
KK = [K];
tt = 0; % time track

for k=1:Fsamples
    j = j + 1;
    tspan = 0:SS:T;
    [t,x]= ode45('irl_ode',tspan,x0);
    uu = [uu u];
    xx = [xx; x];
    x1 = x(length(t),1:4);
    tt = [tt; t + T*(k-1)];
    % calculate phi transpose
    X(j,:) = [ x0(1)^2 x0(1)*x0(2) x0(1)*x0(3) x0(1)*x0(4) ...
               x0(2)^2 x0(2)*x0(3) x0(2)*x0(4)...
               x0(3)^2 x0(3)*x0(4) x0(4)^2 ] - ...
             [ x1(1)^2 x1(1)*x1(2) x1(1)*x1(3) x1(1)*x1(4) ...
               x1(2)^2 x1(2)*x1(3) x1(2)*x1(4)...
               x1(3)^2 x1(3)*x1(4) x1(4)^2 ];
   
    Y(j,:) = x(length(t),5) - x(1,5);
    x0 = [x(length(t),1:4),x(length(t),5)];

    if mod(k,nop) == 0 && norm(X) > tol
        weights = pinv(X)*Y;
        if norm(weights - w0,2) > 1 % update weights if fit condition
            P=[ weights(1)    weights(2)/2  weights(3)/2  weights(4)/2;...
                weights(2)/2  weights(5)    weights(6)/2  weights(7)/2;...
                weights(3)/2  weights(6)/2  weights(8)    weights(9)/2;...
                weights(4)/2  weights(7)/2  weights(9)/2  weights(10)      ];
            K = inv(R)*B'*P;
            X = zeros(nop,10);
            Y = zeros(nop,1);
            j = 0;
            w0 = weights;
        end

    end
%     weights_track = [weights_track; weights];
    KK = [KK;K];
end
%% Simulaton for LQR case
global u_lqr;
[Kopt,Popt] = lqr(A,B,Q,R);
x0 = [pi 1 2 1 0];
SS = T/1000;
[Kopt, Popt] = lqr(A,B,Q,R);
tspan = 0:SS:T*Fsamples;
[t_lqr,x_lqr]= ode45('lqr_ode',tspan,x0);
%% Plot result
close all;
figure("Name", "Сигнал управления u(t) IRL") %  "position",[100 200 1420 500]
plot(T*[0:Fsamples],uu,'linewidth',1.5);
grid on
xlabel('Время (с)', 'fontsize',12);
ylabel("Сигнал управления u(t)",'fontsize',12)
legend("u(t) IRL",'fontsize',12)
hold off;
% grid on;
%% gain K
figure("Name", "Сходимости обратной матрицы K ")
% grid on 
xlabel('Время (с)');
ylabel("Сходимости  обратной матрицы K"); 
plot(T*[0:Fsamples], KK, "b", 'linewidth', 2)
hold on
grid on
plot(T*[0:Fsamples], ones(Fsamples+1,1)*Kopt, 'r--', 'linewidth', 2)
hold off;
%%
figure("Name","error gain K")
plot(T*[0:Fsamples], abs(ones(Fsamples+1,1)*Kopt - KK), "linewidth", 2)
grid on
legendEntries = {'$\\|K_{opt1} - K_1|$', '$\\|K_{opt2} - K_2|$', '$\\|K_{opt3} - K_3|$', '$\\|K_{opt4} - K_4|$'};
legend(legendEntries, 'fontsize',12, 'interpreter', 'latex');
xlabel("Время (с)",'fontsize',12)
ylabel("$|Kopt - K|$",'fontsize',12, "Interpreter", "latex")
%%
figure("Name", "State of system");
subplot(2, 2, 1);
plot(tt, xx(:, 1), "linewidth", 2);
grid on
title('Угловое положение (\theta)', 'fontsize',12);
xlabel('Время (с)','fontsize',12);
ylabel('rad','fontsize',12);

subplot(2, 2, 2);
plot(tt, xx(:, 2), "linewidth", 2);
grid on
% str = "Угловая скорость $\dot{\theta}$";
str = "Угловая скорость (θ̇')"; % Sử dụng tiếng Nga và LaTeX cho theta dot
title(str, 'fontsize',12, 'Interpreter','latex'); 
% title(str, 'Interpreter','latex');
xlabel('Время (с)','fontsize',12);
ylabel('rad/s','fontsize',12);

subplot(2, 2, 3);
plot(tt, xx(:, 3), "linewidth", 2);
grid on
title('Положение (x)','fontsize',12);
xlabel('Время (с)','fontsize',12);
ylabel('м','fontsize',12);

subplot(2, 2, 4);
plot(tt, xx(:, 4), "linewidth", 2);
grid on
title("Скорости (x')", 'Interpreter','latex','fontsize',12);
xlabel('Время (с)','fontsize',12);
ylabel('м/c^2','fontsize',12);
%%
figure("Name", "State of system")
plot(tt, xx(:,1), "linewidth", 1.5);
hold on
plot(tt,xx(:,2), "linewidth", 1.5);
plot(tt,xx(:,3), "linewidth", 1.5);
plot(tt,xx(:,4), "linewidth", 1.5);
grid on;
xlabel('Время (с)');
title('Состояние системы'); 
legendEntries = {'$\\X_{1IRL}(t)$', '$\\X_{2IRL}(t)$', '$\\X_{3IRL}(t)$', '$\\X_{4IRL}(t)$'};
legend(legendEntries, 'interpreter', 'latex','FontSize', 12);
% set(gca,'FontName','Times New Roman','FontSize',15,'linewidth',2);

disp("K optimal LQR: ")
disp(Kopt)
disp("K optimal IRL: ")
disp(K)
disp("P optimal LQR:")
disp(Popt)
disp("P optimal IRL:")
disp(P)
% 
% %% Plot for LQR and IRL case
% figure(4);
% plot(tt, xx(:,1), "linewidth", 1.5);
% hold on
% plot(tt,xx(:,2), "linewidth", 1.5);
% plot(tt,xx(:,3), "linewidth", 1.5);
% plot(tt,xx(:,4), "linewidth", 1.5);
% plot(t_lqr,x_lqr(:,1),'--', "linewidth", 1.5);
% plot(t_lqr,x_lqr(:,2),'--', "linewidth", 1.5);
% plot(t_lqr,x_lqr(:,3), '--', "linewidth", 1.5);
% plot(t_lqr,x_lqr(:,4), '--', "linewidth", 1.5);
% grid on;
% xlabel('Time (s)');
% title('Состояние системы'); 
% legendEntries = {'$\\X_{1IRL}(t)$', '$\\X_{2IRL}(t)$', '$\\X_{3IRL}(t)$', '$\\X_{4IRL}(t)$', '$\\X_{1LQR}(t)$', '$\\X_{2LQR}(t)$', '$\\X_{3LQR}(t)$', '$\\X_{4LQR}(t)$'};
% legend(legendEntries, 'interpreter', 'latex','FontSize', 10);
% set(gca,'FontName','Times New Roman','FontSize',15,'linewidth',2);
% figure(4), hold off;
% 
% 
%% 
figure("Name", "J(x,u)")
plot(tt,xx(:,5),'linewidth',2)
grid on
hold on
plot(t_lqr, x_lqr(:,5), 'linewidth',2)
xlabel('Время (с)', 'fontsize',12);
title('J(x,u)'); 
legendEntries = {'$\\J_{IRL}(t)$', '$\\J_{LQR}(t)$'};
legend(legendEntries, 'interpreter', 'latex','FontSize', 12);
% set(gca,'FontName','Times New Roman','FontSize',12,'linewidth',1);

