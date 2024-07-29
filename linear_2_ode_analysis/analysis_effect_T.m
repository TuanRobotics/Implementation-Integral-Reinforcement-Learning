%% System parameters
% a0 = -4;
% a1 = 4;
% b0 = 2;
% b1 = 1;
clc; clear;

a0 = -10;
a1 = 10;
b0 = 1;
b1 = 1;

A = [0 1; -a0 -a1];
B = [b1; b0];
x0 = [10;5];

if (det(ctrb(A,B)) ~= 0)
    disp("(A,B) - controllable");
    disp(ctrb(A,B));
    disp(det(ctrb(A,B)));
else
    disp("(A,B) - not controllable");
end

%% optimization parameters
Q = eye(2);
R = 1;

[Kopt, Popt] = lqr(A,B,Q,R);
disp("optimal P");
disp(Popt);
disp("optimal K");
disp(Kopt)


%% adaptation parameters
A0 = (A-B*Kopt);
Pa = lyap(A0',0.01*eye(2));
disp("Pa for adaptive control")
disp(Pa)
Pa = Popt;
disp("A0: ")
disp(A0)

%% initial policy
Ko = [1 1];
disp("admissible policy eigenvalues")
disp(eig(A-B*Ko));

%% simulation
T = 0.06;
b1 = sim("regulator_linear_2oder.slx", 3);
%%
pause(0.5)
T1 = 0.025;
b2 = sim("regulator_linear_2oder_T1.slx", 3);
%%
pause(0.5);
T2 = 0.005;
b3 = sim("regulator_linear_2oder_T2.slx", 3);

%% Plot results
close all;
%
figure("Name", " Сигнал управления u(t)","position",[100 200 1420 500]);
plot(b1.LQRu.time,b1.LQRu.data, "linewidth", 2);
hold on
grid on
plot(b1.IRLu.time,b1.IRLu.data,"linewidth", 2);
plot(b2.IRLu.time,b2.IRLu.data,"linewidth", 2);
plot(b3.IRLu.time,b3.IRLu.data,"linewidth", 2);
% plot(b1.ARu.time,b1.ARu.data,"linewidth", 2);
legend("LQR регулятор", "IRL T = 0.06", "IRL T = 0.025", "IRL T = 0.005", 'fontsize',12)
xlabel("Время (с)",'fontsize',12)
ylabel("Стратегия управления u(t)",'fontsize',12)
hold off

%%
figure("Name", "Состояния системы x(t)", "position",[100 200 1420 500])
plot(b1.LQRx.time,b1.LQRx.data, "linewidth", 2);
hold on
grid on
plot(b1.IRLx.time,b1.IRLx.data, "linewidth", 2); % T = 0.1
plot(b2.IRLx.time,b2.IRLx.data, "linewidth", 2); % T = 0.025;
plot(b3.IRLx.time,b3.IRLx.data, "linewidth", 2); % T = 0.005;
% plot(b1.ARx.time,b1.ARx.data, "linewidth", 2);
legend("x_{1}(t),LQR регулятор", "x_{2}(t),LQR регулятор", "x_{1}(t),IRL T = 0.06", "x_{2}(t),IRL T = 0.06", "x_{1}(t),IRL T = 0.025", "x_{2}(t),IRL T = 0.025", "x_{1}(t),IRL T = 0.005", "x_{2}(t),IRL T = 0.005",'fontsize',12)
xlabel("Время (с)",'fontsize',12)
ylabel("Состояние системы x(t)",'fontsize',12)
hold off
%%
figure("Name", "Функция стоимости J(x,u)", "position",[100 200 1420 500])
plot(b1.LQRJ.time,b1.LQRJ.data,"linewidth", 2);
hold on
grid on
plot(b1.IRLJ.time, b1.IRLJ.data,"linewidth", 2);
plot(b2.IRLJ.time, b2.IRLJ.data,"linewidth", 2);
plot(b3.IRLJ.time, b3.IRLJ.data,"linewidth", 2);
% plot(b1.ARJ.time, b1.ARJ.data,"linewidth", 2);
legend("LQR регулятор", "IRL T = 0.06","IRL T = 0.025", "IRL T = 0.005", 'fontsize',12) % , "Адаптивный регулятор"
xlabel("Время (с)",'fontsize',12)
ylabel("Функция стоимости J(x,u)",'fontsize',12)
hold off
%
%%
figure("Name", "коэффициент усиления обратной связи K", "position",[100 200 1420 500])
for i = 1:length(b1.IRLK.time)
    IRLK1(i,1) = b1.IRLK.data(1,1,i);
    IRLK1(i,2) = b1.IRLK.data(1,2,i);
end
for i = 1:length(b2.IRLK.time)
    IRLK2(i,1) = b2.IRLK.data(1,1,i);
    IRLK2(i,2) = b2.IRLK.data(1,2,i);
end
for i = 1:length(b3.IRLK.time)
    IRLK3(i,1) = b3.IRLK.data(1,1,i);
    IRLK3(i,2) = b3.IRLK.data(1,2,i);
end
% for i = 1:length(b1.ARt.time)
%     ARt(i,1) = b1.ARt.data(1,1,i);
%     ARt(i,2) = b1.ARt.data(2,1,i);
% end

hold on
grid on

% plot([0 b1.IRLK.time(size(b1.IRLK.time)*[1;0])],[Kopt(1) Kopt(1)], "linewidth", 2)
% plot([0 b1.IRLK.time(size(b1.IRLK.time)*[1;0])],[Kopt(2) Kopt(2)], "linewidth", 2)
plot(b1.IRLK.time,IRLK1, "linewidth", 2);
plot(b2.IRLK.time,IRLK2, "linewidth", 2);
plot(b3.IRLK.time,IRLK3, "linewidth", 2);
% plot(out.ARt.time,ARt, "linewidth", 2);
legend("$K_{1}, T = 0.06$","$K_{2}, T = 0.06$",  "$K_{1}, T = 0.025$","$K_{2}, T = 0.025$",  "$K_{1}, T = 0.005$","$K_{2}, T = 0.005$",'fontsize',12, 'Interpreter','latex'); %  "$\hat{\theta}_{1}$","$\hat{\theta}_{2}$"
% legendEntries = {'$\\LQR K_{1}$', '$\\X_{2IRL}(t)$', '$\\X_{3IRL}(t)$', '$\\X_{4IRL}(t)$', '$\\X_{1LQR}(t)$', '$\\X_{2LQR}(t)$', '$\\X_{3LQR}(t)$', '$\\X_{4LQR}(t)$'};
% legend(legendEntries, 'interpreter', 'latex','FontSize', 10);
xlabel("Время (с)",'fontsize',12)
ylabel("Коэффициент усиления обратной связи K",'fontsize',12)
hold off
