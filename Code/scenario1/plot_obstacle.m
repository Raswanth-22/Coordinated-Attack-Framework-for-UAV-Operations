function [] = plot_obstacle2(bias_x, bias_y, r, h)
% Create a cylindrical obstacle and plot it in 3‑D space.
%   bias_x, bias_y : Center coordinates of the cylinder base
%   r              : Cylinder radius
%   h              : Cylinder height (scaled)

% ----- Generate cylinder mesh -----
[x, y, z] = cylinder(1, 20);
x = x * r + bias_x;
y = y * r + bias_y;
z = z * h;

% ----- Plot the obstacle -----
hold on
surf(x, y, z, 'EdgeColor', 'red', ...
    'FaceColor', 'red', 'FaceAlpha', 0.1);

% Alternative solid-face rendering (commented):
% surf(x, y, z, 'EdgeColor', "none", ...
%     'FaceColor', 'red', 'FaceAlpha', 0.3);
% color = [1 0 0];
% colormap(color)
end
