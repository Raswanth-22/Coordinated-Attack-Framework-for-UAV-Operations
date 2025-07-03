figure
axis([0 delta_t*iters 0 inf]);
grid on
line = 1.5;

hold on
plot(delta_t*(1:iters-1), ...
     vecnorm(bag(1:2,1:iters-1) - over'), ...
     'Color', '#2084C5', 'linewidth', line)

hold on
plot(delta_t*(1:iters-1), ...
     vecnorm(bag2(1:2,1:iters-1) - over'), ...
     'Color', '#DE6836', 'linewidth', line)

hold on
yline(r_gui, '--', 'Threshold = 6 m');   % helper line
legend('uav1', 'uav2');                 % legend

xlabel('t / (s)')
ylabel('d / (m)')
