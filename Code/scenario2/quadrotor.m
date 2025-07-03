function [] = quadrotor(bias_x, bias_y, bias_z, pitch, roll)
%QUADROTOR  Render a simple 3‑D quadrotor model at the desired pose.
%   quadrotor(bias_x, bias_y, bias_z, pitch, roll) draws four rotor rings
%   connected by two crossing arms, then applies a roll‑pitch attitude and
%   translates the entire assembly to (bias_x, bias_y, bias_z).
%
%   Coordinate frame (right‑handed):
%       +y : nose direction (forward)
%       +x : right wingtip when viewed from above
%       +z : upward
%
%   Inputs
%   ------
%   bias_x, bias_y, bias_z : Centre position of the UAV.
%   pitch (deg)            : Rotation about the –x axis (nose‑up positive).
%   roll  (deg)            : Rotation about the +y axis (right‑wing‑down positive).

%% --------------------------- Global scale factor ---------------------------
zoom = 5;   % Scales every geometric primitive

%% --------------------------- Rotor rings -----------------------------------
% Large ring (outer rotor frame)
radius_big  = 0.1 * zoom;     % Radius of large cylinder
height_big  = 0.02 * zoom;    % Height (thickness) of large cylinder
resolution  = 10;             % Mesh resolution
[X, Y, Z]   = cylinder(radius_big, resolution);
Z           = Z * height_big;

% Small ring (inner hub)
radius_small = 0.8 * radius_big;
height_small = 0.5 * height_big;
[Xs, Ys, Zs] = cylinder(radius_small, resolution);
Zs           = Zs * height_small;

%% --------------------------- Ring positions --------------------------------
% Four rotor positions in a square layout (± along x and y)
ringOffset = 0.15 * zoom * ...
    [ 1,  1;
     -1,  1;
     -1, -1;
      1, -1];

for i = 1:4
    % Outer ring
    x = X + ringOffset(i,1);
    y = Y + ringOffset(i,2);
    z = Z;
    hold on
    s_outer(i) = surf(x, y, z, 'FaceAlpha', 0.7);

    % Inner hub
    x_small = Xs + ringOffset(i,1);
    y_small = Ys + ringOffset(i,2);
    z_small = Zs;
    s_inner(i) = surf([x(2,:); x_small(2,:)], ...
                      [y(2,:); y_small(2,:)], ...
                      [z(2,:); z_small(2,:)], 'FaceAlpha', 0.7);
end

%% --------------------------- Frame / arms ----------------------------------
% Thin cylindrical arms crossing at 45° and 135° about body‑z
[x_arm, y_arm, z_arm] = cylinder(1, 50);
scale = 0.01 * zoom;
x_arm = x_arm * scale;
y_arm = y_arm * scale;
arm_len = 0.15 * sqrt(2) * zoom;
z_arm = z_arm * arm_len;

for i = 1:4
    arms(i) = surf(x_arm, y_arm, z_arm, 'EdgeColor', 'none', ...
                   'FaceColor', 'black', 'FaceAlpha', 0.35);
    origin = [0, 0, 0];
    rotate(arms(i), [1 0 0], 90,   origin);          % Stand cylinder upright
    rotate(arms(i), [0 0 1], 45 + 90*(i-1), origin); % Rotate into position
end

%% --------------------------- Apply attitude --------------------------------
for i = 1:4
    % Roll (about +y)
    rotate(s_outer(i), [0 1 0], roll,  [0 0 0]);
    rotate(s_inner(i), [0 1 0], roll,  [0 0 0]);
    rotate(arms(i),    [0 1 0], roll,  [0 0 0]);

    % Pitch (about –x so positive value raises the nose)
    rotate(s_outer(i), [-1 0 0], pitch, [0 0 0]);
    rotate(s_inner(i), [-1 0 0], pitch, [0 0 0]);
    rotate(arms(i),    [-1 0 0], pitch, [0 0 0]);

    % Translate to desired world position
    s_outer(i).XData = s_outer(i).XData + bias_x;
    s_inner(i).XData = s_inner(i).XData + bias_x;
    arms(i).XData    = arms(i).XData    + bias_x;

    s_outer(i).YData = s_outer(i).YData + bias_y;
    s_inner(i).YData = s_inner(i).YData + bias_y;
    arms(i).YData    = arms(i).YData    + bias_y;

    s_outer(i).ZData = s_outer(i).ZData + bias_z;
    s_inner(i).ZData = s_inner(i).ZData + bias_z;
    arms(i).ZData    = arms(i).ZData    + bias_z;
end
end
