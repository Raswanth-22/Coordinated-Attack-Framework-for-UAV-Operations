function [] = ploy_target3(x_target, y_target, h, r)
% Create a cylindrical mesh representing the target region
[x, y, z] = cylinder(1, 20);

% Scale and translate the mesh to the target position
x = x * r + x_target;
y = y * r + y_target;
z = z * h;

% Plot the cylindrical target volume
hold on
surf(x, y, z, 'EdgeColor', 'blue', ...
     'FaceColor', 'blue', 'FaceAlpha', 0.05);

% Plot the target center point
plot3(x_target, y_target, h, '*', 'Color', 'blue', 'MarkerSize', 10);
end
