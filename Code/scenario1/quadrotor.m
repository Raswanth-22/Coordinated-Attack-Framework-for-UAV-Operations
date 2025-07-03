function [] = quadrotor(bias_x, bias_y, bias_z, pitch, roll)
% Draw a simple 3‑D quadrotor model at the specified position and attitude.
%   bias_x, bias_y, bias_z : Center position of the UAV
%   pitch, roll           : Attitude angles (degrees)

% ----- Scale factor -----
zoom = 5;

% ----- Rotor frame (outer/inner rings) -----
% Parameters of the larger cylinder (outer ring)
radius_big  = 0.1 * zoom;   % Large cylinder radius
height_big  = 0.02 * zoom;  % Large cylinder height
resolution  = 10;           % Mesh resolution

% Create large cylinder mesh
[X, Y, Z] = cylinder(radius_big, resolution);
Z = Z * height_big;

% Parameters of the smaller cylinder (inner ring)
radius_small = 0.8 * radius_big;   % Small cylinder radius
height_small = 0.5 * height_big;   % Small cylinder height

% Create small cylinder mesh
[X_small, Y_small, Z_small] = cylinder(radius_small, resolution);
Z_small = Z_small * height_small;

%% Draw rotor rings
bias = 0.15 * zoom * ...
    [ 1,  1;
     -1,  1;
     -1, -1;
      1, -1];

for i = 1:4
    % Outer ring
    x = X + bias(i,1);
    y = Y + bias(i,2);
    z = Z;
    hold on
    s_1(i) = surf(x, y, z, 'FaceAlpha', 0.7);
    
    % Inner ring (hub)
    x_small = X_small + bias(i,1);
    y_small = Y_small + bias(i,2);
    z_small = Z_small;
    hold on
    s_2(i) = surf([x(2,:); x_small(2,:)], [y(2,:); y_small(2,:)], [z(2,:); z_small(2,:)], 'FaceAlpha', 0.7);
end

%% Draw quadrotor arms/frame
[x, y, z] = cylinder(1, 50);
\% Thin cylindrical arm
x = 0.01 * x * zoom;
y = 0.01 * y * zoom;
z = z * 0.15 * sqrt(2) * zoom;

for i = 1:4
    ss_2(i) = surf(x, y, z, 'EdgeColor', "none", ...
        'FaceColor', 'black', 'FaceAlpha', 0.35);
    origin = [0, 0, 0];
    rotate(ss_2(i), [1 0 0], 90, origin);      % Stand the cylinder up
    rotate(ss_2(i), [0 0 1], 45 + 90*(i-1), origin); % Rotate to each arm
end

%% Apply UAV attitude and translation
% Coordinate convention:
%   +y is the UAV nose direction
%   +x is right wingtip when viewed from above
for i = 1:4
    % Roll (rotation around +x‑axis)
    rotate(s_1(i),  [0 1 0], roll,  origin);
    rotate(s_2(i),  [0 1 0], roll,  origin);
    rotate(ss_2(i), [0 1 0], roll,  origin);
    
    % Pitch (rotation around –y‑axis so nose dips when positive)
    rotate(s_1(i),  [-1 0 0], pitch, origin);
    rotate(s_2(i),  [-1 0 0], pitch, origin);
    rotate(ss_2(i), [-1 0 0], pitch, origin);
    
    % Translate to desired position
    s_1(i).XData  = s_1(i).XData + bias_x;
    s_2(i).XData  = s_2(i).XData + bias_x;
    ss_2(i).XData = ss_2(i).XData + bias_x;
    
    s_1(i).YData  = s_1(i).YData + bias_y;
    s_2(i).YData  = s_2(i).YData + bias_y;
    ss_2(i).YData = ss_2(i).YData + bias_y;
    
    s_1(i).ZData  = s_1(i).ZData + bias_z;
    s_2(i).ZData  = s_2(i).ZData + bias_z;
    ss_2(i).ZData = ss_2(i).ZData + bias_z;
end
end
