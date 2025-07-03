function output = comput_P(curr, over, obstacle, Q_star)
%COMPUT_P  Evaluate the combined attractive–repulsive potential at a point.
%   output = COMPUT_P(curr, over, obstacle, Q_star) returns the scalar
%   potential value at the current 2‑D position 'curr'. The potential is
%   the sum of an attractive term pulling the UAV toward the goal 'over'
%   and a repulsive term pushing it away from nearby obstacles.
%
%   Inputs
%   ------
%   curr     : 2‑element column vector [x; y] – current position.
%   over     : 2‑element column vector – goal (target) position.
%   obstacle : 2×N matrix – columns are obstacle positions.
%   Q_star   : Scalar – influence radius of each obstacle.
%
%   Output
%   -------
%   output   : Scalar potential value at 'curr'. Lower values are preferred
%              by the gradient‑descent planner.

k_att = 1;        % Attractive gain
k_rep = 1000;     % Repulsive gain
repu  = 0;        % Initialize repulsive component

%% Attractive potential toward the goal -----------------------------
attr = 0.5 * k_att * (norm(curr - over))^2;

%% Repulsive potential from obstacles -------------------------------
% Only consider obstacles within Q_star of the current position.
for i = 1:size(obstacle, 2)
    dist = norm(curr - obstacle(:, i));
    if dist <= Q_star
        repu = repu + 0.5 * k_rep * (1 / dist - 1 / Q_star) ^ 0.5;
    end
end

%% Total potential ---------------------------------------------------
output = attr + repu;
end
