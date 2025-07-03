clc
clear all
close all

%% -------------------------- Figure setup --------------------------
figure
axis([0 50 0 50 0 10]);
axis equal
xlabel('x/(m)')
ylabel('y/(m)')
zlabel('z/(m)')
view(3)
grid on

% -------------------------- Color selection --------------------------
rng(1)
all_colors = rand(6,3);

% -------------------------- Data logs --------------------------
bag      = zeros(6,300);   % UAV 1 state log
bag_obs  = zeros(2,300);   % Obstacle 1 state log
bag_obs2 = zeros(2,300);   % Obstacle 2 state log
bag_obs3 = zeros(2,300);   % Obstacle 3 state log
bag2     = zeros(6,300);   % UAV 2 state log

% -------------------------- Start and goal points --------------------------
begin  = [5, 0];   % UAV 1 start (x,y)
begin2 = [40, 0];  % UAV 2 start (x,y)

goal   = [25, 45]; % Common goal (x,y)

% -------------------------- Virtual guidance points --------------------------
r_gui      = 6;            % Guidance circle radius
theta_gui  = -0.75*pi;     % Bearing for UAV 1 guidance point
theta_gui2 = -0.25*pi;     % Bearing for UAV 2 guidance point
v_target   = 0;            % Target velocity (stationary)

over_gui  = goal + r_gui * [cos(theta_gui),  sin(theta_gui)];   % Guidance point for UAV 1
over_gui2 = goal + r_gui * [cos(theta_gui2), sin(theta_gui2)];  % Guidance point for UAV 2

% -------------------------- Moving‑obstacle parameters --------------------------
r_obsmove = 10;  % Orbit radius (m)
v_obsmove = 1;   % Tangential speed (m/s)
w = v_obsmove / r_obsmove;   % Angular rate (rad/s)

% Initial obstacle positions (three obstacles)
obstacle = [25 + r_obsmove*cos( pi/2),      25 + r_obsmove*cos(-pi/6),      25 + r_obsmove*cos(-5*pi/6);
            20 + r_obsmove*sin( pi/2),      20 + r_obsmove*sin(-pi/6),      20 + r_obsmove*sin(-5*pi/6)];

% -------------------------- Visualization flags --------------------------
vision_uav  = true;   % Toggle UAV 1 mesh
vision_uav2 = true;   % Toggle UAV 2 mesh

% -------------------------- Cooperation flags --------------------------
flag_co  = false;   % UAV 1 has reached cooperation region
flag_co2 = false;   % UAV 2 has reached cooperation region

accu      = 0;      % Counter for UAV 1 guided phase
accu2     = 0;      % Counter for UAV 2 guided phase
accu_time = 0;      % Time steps where both UAVs are in cooperation region

% -------------------------- Simulation parameters --------------------------
v       = 3;      % Nominal UAV speed (m/s)
delta_t = 0.1;    % Simulation time‑step (s)
t_end   = 200;    % Total simulation time (s)
iters   = 1;      % Iteration counter

% -------------------------- State initialisation --------------------------
curr          = [begin';  0];   % UAV 1 current state [x;y;z]
curr_previous = curr;

curr2         = [begin2'; 0];   % UAV 2 current state [x;y;z]
curr_previous2 = curr2;

testR  = v * delta_t;   % Radius of sampling circle for potential‑field search
Q_star = 6;             % Obstacle influence radius

num_point    = 36;                       % Number of sample points on the circle
testPoint    = zeros(num_point,2);       % Coordinates of sample points
testOut      = zeros(1,num_point);       % Potential at each sample point
step_predict = 10;                       % MPC prediction horizon (steps)

% Prediction arrays (z‑coordinate pre‑filled)
pos_predict  = zeros(step_predict,3);
pos_predict2 = zeros(step_predict,3);

h  = 2; pos_predict(:,3)  = h;   % Flight altitude for UAV 1 (m)
h2 = 4; pos_predict2(:,3) = h2;  % Flight altitude for UAV 2 (m)

% -------------------------- Quadrotor drawing parameters --------------------------
roll_max  = 5;   % Maximum roll for rendering (deg)
pitch_max = 5;   % Maximum pitch for rendering (deg)
U_k  = zeros(3,1);  % Latest control for UAV 1
U_k2 = zeros(3,1);  % Latest control for UAV 2

% -------------------------- Plot initial markers --------------------------
hold on
plot3(begin(1),  begin(2),  0, '*b', 'MarkerSize', 10);
plot3(begin2(1), begin2(2), 0, '*b', 'MarkerSize', 10);

% Draw goal regions
plot_target(goal(1), goal(2), h,  r_gui);
plot_target(goal(1), goal(2), h2, r_gui);

% -------------------------- MPC matrices --------------------------
A = [zeros(3), eye(3);
     zeros(3), zeros(3)] * delta_t + eye(6);
B = [0.5*eye(3)*delta_t^2; eye(3)*delta_t];
N = step_predict;            % Prediction horizon (steps)

x_k  = [begin(1);  begin(2);  0; zeros(3,1)];
x_k2 = [begin2(1); begin2(2); 0; zeros(3,1)];

Q = [eye(3), zeros(3); zeros(3), zeros(3)];
F = Q;                       % Terminal weight
R = zeros(3);

u_max = 3;                                   % Max acceleration (m/s²)
ub = kron(ones(N,1), [u_max; u_max; u_max]);
lb = -ub;

%% -------------------------- Main simulation loop --------------------------
while iters <= t_end / delta_t
    %% ----- Update cooperation flags -----
    if flag_co  || norm(curr(1:2)'  - over_gui ) < 2*testR, flag_co  = true; end
    if flag_co2 || norm(curr2(1:2)' - over_gui2) < 2*testR, flag_co2 = true; end
    if flag_co && flag_co2, accu_time = accu_time + 1; end

    %% ----- Clear previous dynamic graphics -----
    if vision_uav || vision_uav2
        delete(findobj('FaceAlpha',0.7));
        delete(findobj('FaceAlpha',0.35));
    end
    delete(findobj('Color','green'));     % Prediction path UAV 1
    delete(findobj('Color','magenta'));   % Prediction path UAV 2
    delete(findobj('FaceColor','red'));   % Obstacles
    delete(findobj('Color', all_colors(5,:))); % Guidance points

    %% ----- Update guidance points (target stationary) -----
    over_gui(1)  = over_gui(1)  + v_target*delta_t;
    over_gui2(1) = over_gui2(1) + v_target*delta_t;

    %% ----- Update moving obstacles (circular motion) -----
    t = iters * delta_t;
    obstacle = [25 + r_obsmove*cos( pi/2    + w*t),
                25 + r_obsmove*cos(-pi/6   + w*t),
                25 + r_obsmove*cos(-5*pi/6 + w*t);
                20 + r_obsmove*sin( pi/2    + w*t),
                20 + r_obsmove*sin(-pi/6   + w*t),
                20 + r_obsmove*sin(-5*pi/6 + w*t)];
    bag_obs(:,iters)  = obstacle(:,1);
    bag_obs2(:,iters) = obstacle(:,2);
    bag_obs3(:,iters) = obstacle(:,3);

    %% -------------------- Visualization: UAV 1 --------------------
    plot3([curr(1), curr(1)], [curr(2), curr(2)], [curr(3), 0], ':', 'Color', "#4DBEEE", 'LineWidth', 1);
    plot3([curr_previous(1), curr(1)], [curr_previous(2), curr(2)], [curr_previous(3), curr(3)], '-', 'Color', "#4DBEEE", 'LineWidth', 2);
    curr_previous = curr;
    if vision_uav
        roll  = U_k(1)/u_max * roll_max;
        pitch = U_k(2)/u_max * pitch_max;
        quadrotor(curr(1), curr(2), curr(3), pitch, roll);
    end

    %% -------------------- Visualization: UAV 2 --------------------
    plot3([curr2(1), curr2(1)], [curr2(2), curr2(2)], [curr2(3), 0], ':', 'Color', "#D95319", 'LineWidth', 1);
    plot3([curr_previous2(1), curr2(1)], [curr_previous2(2), curr2(2)], [curr_previous2(3), curr2(3)], '-', 'Color', "#D95319", 'LineWidth', 2);
   	curr_previous2 = curr2;
    if vision_uav2
        roll  = U_k2(1)/u_max * roll_max;
        pitch = U_k2(2)/u_max * pitch_max;
        quadrotor(curr2(1), curr2(2), curr2(3), pitch, roll);
    end

    %% -------------------- Potential‑field prediction: UAV 1 --------------------
    if accu_time <= 2
        curr_temp = curr(1:2)';
        for i = 1:step_predict
            for j = 1:num_point
                testPoint(j,:) = [testR*cos((j-1)*2*pi/num_point)+curr_temp(1), ...
                                  testR*sin((j-1)*2*pi/num_point)+curr_temp(2)];
                testOut(j) = comput_P(testPoint(j,:)'; , over
