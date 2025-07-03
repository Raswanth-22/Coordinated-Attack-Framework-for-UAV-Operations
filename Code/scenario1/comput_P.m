function output = comput_P(curr, over, obstacle, Q_star)
% Computes the potential field value at a given point.
%   Attractive and repulsive potentials are summed to obtain the total.

k_att = 1;         % Attractive potential gain
repu  = 0;         % Initialize repulsive component
k_rep = 1000;      % Repulsive potential gain
Q_star = Q_star;   % Obstacle influence radius

% Attractive potential from current point to goal
attr = 0.5 * k_att * (norm(curr - over))^2;

% Repulsive potential from obstacles to current point
% Set obstacle repulsion influence radius to Q_star
for i = 1:size(obstacle, 2)
    if norm(curr - obstacle(:, i)) <= Q_star
        repu = repu + 0.5 * k_rep * (1 / norm(curr - obstacle(:, i)) - 1 / Q_star) ^ 0.5;
    else
        repu = repu + 0;
    end
end

output = attr + repu;
end
