function [] = plot_obstacle2(bias_x, bias_y, r, h)
% Generate a cylindrical mesh
[x, y, z] = cylinder(1, 20);
x = x * r + bias_x;
y = y * r + bias_y;
z = z * h;

% Plot the obstacle cylinder
hold on
surf(x, y, z, 'EdgeColor', 'red', ...
     'FaceColor', 'red', 'FaceAlpha', 0.1);

% Alternative solid-face rendering:
% surf(x, y, z, 'EdgeColor', 'none', ...
%      'FaceColor', 'red', 'FaceAlpha', 0.3);
% colormap([1 0 0])   % Force solid red if desired
end
