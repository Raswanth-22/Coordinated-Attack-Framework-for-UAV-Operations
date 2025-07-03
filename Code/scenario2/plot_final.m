%% -------- Plot distance to the goal --------
figure
axis([0 delta_t*iters 0 inf]);
grid on
line = 1.5;

hold on
plot(delta_t*(1:iters-1), ...
     vecnorm(bag(1:2,1:iters-1) - over'), ...
     'Color', '#2084C5', 'LineWidth', line)

hold on
plot(delta_t*(1:iters-1), ...
     vecnorm(bag2(1:2,1:iters-1) - over'), ...
     'Color', '#DE6836', 'LineWidth', line)

% Threshold circle around the goal (guidance radius)
yline(r_gui, '--', 'Threshold = 6 m');

legend('UAV 1', 'UAV 2')
xlabel('t (s)')
ylabel('d (m)')

%% -------- Plot distances from UAV 1 to obstacles --------
figure
axis([0 delta_t*iters 0 inf]);
grid on
line = 1.5;

hold on
plot(delta_t*(1:iters-1), ...
     vecnorm(bag_obs(:,1:iters-1)  - bag(1:2,1:iters-1)), ...
     'Color', '#2084C5', 'LineWidth', line)

hold on
plot(delta_t*(1:iters-1), ...
     vecnorm(bag_obs2(:,1:iters-1) - bag(1:2,1:iters-1)), ...
     'Color', '#DE6836', 'LineWidth', line)

hold on
plot(delta_t*(1:iters-1), ...
     vecnorm(bag_obs3(:,1:iters-1) - bag(1:2,1:iters-1)), ...
     'Color', '#EDB120', 'LineWidth', line)

yline(5, '--', 'Threshold');
legend('d_{1}', 'd_{2}', 'd_{3}')
xlabel('t (s)')
ylabel('d (m)')

%% -------- Plot distances from UAV 2 to obstacles --------
figure
axis([0 delta_t*iters 0 inf]);
grid on

hold on
plot(delta_t*(1:iters-1), ...
     vecnorm(bag_obs(:,1:iters-1)  - bag2(1:2,1:iters-1)), ...
     'Color', '#2084C5', 'LineWidth', line)

hold on
plot(delta_t*(1:iters-1), ...
     vecnorm(bag_obs2(:,1:iters-1) - bag2(1:2,1:iters-1)), ...
     'Color', '#DE6836', 'LineWidth', line)

hold on
plot(delta_t*(1:iters-1), ...
     vecnorm(bag_obs3(:,1:iters-1) - bag2(1:2,1:iters-1)), ...
     'Color', '#EDB120', 'LineWidth', line)

yline(5, '--', 'Threshold');
legend('d_{1}', 'd_{2}', 'd_{3}')
xlabel('t (s)')
ylabel('d (m)')
