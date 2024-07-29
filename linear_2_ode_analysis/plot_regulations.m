 close all;
%
figure("Name", " Сигнал управления u(t)","position",[100 200 1420 500]);
plot(out.LQRu.time,out.LQRu.data, "linewidth", 2);
hold on
grid on
plot(out.IRLu.time,out.IRLu.data,"linewidth", 2);
plot(out.ARu.time,out.ARu.data,"linewidth", 2);
legend("LQR регулятор", "IRL регулятор", "Адаптивный регулятор", 'fontsize',12)
xlabel("Время (с)", 'fontsize',12)
ylabel("Стратегия управления u(t)",'fontsize',12)
hold off

%% 
figure("Name", "Состояния системы x(t)", "position",[100 200 1420 500])
plot(out.LQRx.time,out.LQRx.data, "linewidth", 2);
hold on
grid on
plot(out.IRLx.time,out.IRLx.data, "linewidth", 2);
plot(out.ARx.time,out.ARx.data, "linewidth", 2);
legend("x_{1}(t),LQR регулятор", "x_{2}(t),LQR регулятор", "x_{1}(t),IRL регулятор", "x_{2}(t),IRL регулятор, ", "x_{1}(t),Адаптивный регулятор", "x_{2}(t),Адаптивный регулятор", 'fontsize',12)
xlabel("Время (с)",'fontsize',12)
ylabel("Состояние системы x(t)",'fontsize',12)
hold off
%%
figure("Name", "Функция стоимости J(x,u)", "position",[100 200 1420 500])
plot(out.LQRJ.time,out.LQRJ.data,"linewidth", 2);
hold on
grid on
plot(out.IRLJ.time,out.IRLJ.data,"linewidth", 2);
plot(out.ARJ.time,out.ARJ.data,"linewidth", 2);
legend("LQR регулятор", "IRL регулятор", "Адаптивный регулятор", 'fontsize',12)
xlabel("Время (с)",'fontsize',12)
ylabel("Функция стоимости J(x,u)",'fontsize',12)
hold off
%%
figure("Name", "Обратная матица K", "position",[100 200 1420 500])
for i = 1:length(out.IRLK.time)
    IRLK(i,1) = out.IRLK.data(1,1,i);
    IRLK(i,2) = out.IRLK.data(1,2,i);
end
for i = 1:length(out.ARt.time)
    ARt(i,1) = out.ARt.data(1,1,i);
    ARt(i,2) = out.ARt.data(2,1,i);
end

hold on
grid on

plot([0 out.IRLK.time(size(out.IRLK.time)*[1;0])],[Kopt(1) Kopt(1)], "--", "linewidth", 2)
plot([0 out.IRLK.time(size(out.IRLK.time)*[1;0])],[Kopt(2) Kopt(2)], "--", "linewidth", 2)
plot(out.IRLK.time,IRLK, "linewidth", 2);
plot(out.ARt.time,ARt, "linewidth", 2);
legend("LQR $K_{1}$","$\\LQR, K_{2}$", "$IRL, K_{1}$","$IRL, K_{2}$", "$\hat{\theta}_{1}$","$\hat{\theta}_{2}$", 'fontsize',12,'Interpreter','latex');
% legend("LQR K_{1}","LQR, K_{2}", "IRL, K_{1}","IRL, K_{2}", "$\hat{\theta}_{1}$","$\hat{\theta}_{2}$",'Interpreter','latex');
% legendEntries = {'$\\LQR K_{1}$', '$\\X_{2IRL}(t)$', '$\\X_{3IRL}(t)$', '$\\X_{4IRL}(t)$', '$\\X_{1LQR}(t)$', '$\\X_{2LQR}(t)$', '$\\X_{3LQR}(t)$', '$\\X_{4LQR}(t)$'};
% legend(legendEntries, 'interpreter', 'latex','FontSize', 10);
xlabel("Время (с)", 'fontsize',12)
ylabel("Коэффициент обратной матрицы K", 'fontsize',12)
hold off
