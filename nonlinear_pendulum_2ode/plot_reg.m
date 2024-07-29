figure(1);
subplot(2, 1, 1);
plot(tt, xx(:, 1), "linewidth", 1.5);
hold on
plot(tt, xx1(:,1), "linewidth", 1.5)
grid on
title('Угловое положение (theta)');
xlabel('Time (s)');
ylabel('rad');
hold off;

subplot(2, 1, 2);
plot(tt, xx(:, 2), "linewidth", 1.5);
hold on
plot(tt, xx1(:, 2), "linewidth", 1.5);
grid on
hold off
title('Угловая скорость (theta dot)');
xlabel('Время (с)');
ylabel('rad/s');
figure(1);