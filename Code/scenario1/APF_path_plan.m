clc
clear all
close all

%% Figure setup
figure
axis([0 50 0 50 0 10]);
axis equal
xlabel('x/(m)')
ylabel('y/(m)')
zlabel('z/(m)')
view(3)
grid on

% ----- Color selection -----
rng(1)
all_colors = rand(6,3);

% ----- Data containers -----
bag  = zeros(6,300);
bag2 = zeros(6,300);

% ----- Start and goal points -----
startPos  = [5,0];
goalPos   = [25,45];

startPos2 = [40,0];

% ----- Virtual guidance point -----
r_gui      = 6;            % Radius of the virtual guidance circle
theta_gui  = -0.75*pi;     % Angle for guidance point 1
v_target   = 0;            % Target velocity (stationary)
guidePos   = goalPos + r_gui*[cos(theta_gui),sin(theta_gui)];

theta_gui2 = -0.25*pi;     % Angle for guidance point 2
guidePos2  = goalPos + r_gui*[cos(theta_gui2),sin(theta_gui2)];

% ----- Obstacle locations -----
obstacle=[15 35 10 24 40 20 21 35; ...
          15 15 30 30 25 15 30 30];

% ----- Visualization flags -----
showUAV1 = true;
showUAV2 = true;

% ----- Cooperation flags -----
flag_co  = false;   % UAV1 cooperation flag
flag_co2 = false;   % UAV2 cooperation flag

accu      = 0;      % Accumulator for guidance point 1
accu2     = 0;      % Accumulator for guidance point 2
accu_time = 0;      % Both UAVs within cooperation region counter

% ----- Simulation parameters -----
v        = 3;        % Speed (m/s)
delta_t  = 0.1;      % Time step (s)
t_end    = 200;      % Total simulation time (s)
iters    = 1;        % Iteration counter

% ----- Current state initialization -----
curr          = [startPos';0];
curr_previous = curr;

curr2          = [startPos2';0];
curr_previous2 = curr2;

% ----- Potential field parameters -----
testR      = v*delta_t;     % Test circle radius
Q_star     = 5;             % Obstacle influence radius
num_point  = 36;            % Number of potential points around UAV
testPoint  = zeros(num_point,2);
testOut    = zeros(1,num_point);
step_predict = 10;          % Prediction horizon (steps)

% ----- Prediction array -----
pos_predict  = zeros(step_predict,3);
pos_predict2 = zeros(step_predict,3);

% Add fixed flight altitude
h  = 2;  pos_predict(:,3)  = h;
h2 = 4;  pos_predict2(:,3) = h2;

% ----- UAV attitude drawing parameters -----
roll_max  = 5;   % deg
pitch_max = 5;   % deg
U_k  = zeros(3,1);
U_k2 = zeros(3,1);

% ----- Plot start points -----
hold on;
plot3(startPos(1), startPos(2), 0,'*b','MarkerSize',10);
plot3(startPos2(1),startPos2(2),0,'*b','MarkerSize',10);

% ----- Draw circular goal regions -----
plot_target(goalPos(1),goalPos(2),h ,r_gui);
plot_target(goalPos(1),goalPos(2),h2,r_gui);

% ----- MPC parameters -----
A = [zeros(3),eye(3);
     zeros(3),zeros(3)]*delta_t + eye(6);

B = [0.5*eye(3)*delta_t^2;eye(3)*delta_t];

N  = step_predict;           % Prediction length
x_k  = [startPos(1);startPos(2);0;zeros(3,1)];
x_k2 = [startPos2(1);startPos2(2);0;zeros(3,1)];

Q = [eye(3),zeros(3);zeros(3),zeros(3)];
F = Q;
R = zeros(3);

u_max = 3;                   % Acceleration constraint
ub = kron(ones(N,1),[u_max;u_max;u_max]);
lb = -ub;

%% ---------------------- Main loop ---------------------- %%
while iters <= t_end/delta_t
    
    % --- Update cooperation flags ---
    if flag_co || norm(curr(1:2)'-guidePos)  < 2*testR, flag_co  = true; end
    if flag_co2|| norm(curr2(1:2)'-guidePos2) < 2*testR, flag_co2 = true; end
    if flag_co && flag_co2, accu_time = accu_time + 1; end
    
    % --- Clear previous graphics ---
    if showUAV1
        delete(findobj('FaceAlpha',0.7));
        delete(findobj('FaceAlpha',0.35));
    end
    if showUAV2
        delete(findobj('FaceAlpha',0.7));
        delete(findobj('FaceAlpha',0.35));
    end
    delete(findobj('color','green'));
    delete(findobj('FaceColor','red'));
    delete(findobj('color',all_colors(5,:)));
    delete(findobj('color','magenta'));
    
    % --- Move guidance points (if target moves) ---
    guidePos(1)  = guidePos(1)  + v_target*delta_t;
    guidePos2(1) = guidePos2(1) + v_target*delta_t;
    
    % ================= Visualization for UAV1 =================
    plot3([curr(1), curr(1)], [curr(2), curr(2)], [curr(3),0], ':','Color','#4DBEEE','linewidth',1);
    plot3([curr_previous(1),curr(1)], [curr_previous(2),curr(2)], [curr_previous(3),curr(3)], '-','Color','#4DBEEE','linewidth',2);
    curr_previous = curr;
    
    if showUAV1
        roll  = U_k(1)/u_max*roll_max;
        pitch = U_k(2)/u_max*pitch_max;
        quadrotor(curr(1),curr(2),curr(3),pitch,roll);
    end
    
    % ================= Visualization for UAV2 =================
    plot3([curr2(1), curr2(1)], [curr2(2), curr2(2)], [curr2(3),0], ':','Color','#D95319','linewidth',1);
    plot3([curr_previous2(1),curr2(1)], [curr_previous2(2),curr2(2)], [curr_previous2(3),curr2(3)], '-','Color','#D95319','linewidth',2);
    curr_previous2 = curr2;
    
    if showUAV2
        roll  = U_k2(1)/u_max*roll_max;
        pitch = U_k2(2)/u_max*pitch_max;
        quadrotor(curr2(1),curr2(2),curr2(3),pitch,roll);
    end
    
    % ----- Potential field to obtain prediction set for UAV1 -----
    if accu_time <= 2
        curr_temp = curr(1:2)';
        for i = 1:step_predict
            for j = 1:num_point
                testPoint(j,:) = [testR*cos((j-1)*2*pi/num_point)+curr_temp(1), ...
                                  testR*sin((j-1)*2*pi/num_point)+curr_temp(2)];
                testOut(j) = comput_P(testPoint(j,:)'; , guidePos', obstacle, Q_star);
            end
            [~,idx] = min(testOut);
            curr_temp = testPoint(idx,:);
            pos_predict(i,1:2) = curr_temp;
        end
    else
        testR = v*delta_t*2/3;
        x_start = guidePos(1) - testR*cos(theta_gui)*accu;
        y_start = guidePos(2) - testR*sin(theta_gui)*accu;
        accu = accu + 1;
        idx = (1:step_predict)';
        pos_predict(:,1) = x_start - testR*cos(theta_gui)*idx;
        pos_predict(:,2) = y_start - testR*sin(theta_gui)*idx;
    end
    
    % ----- Potential field to obtain prediction set for UAV2 -----
    if accu_time <= 2
        curr_temp = curr2(1:2)';
        for i = 1:step_predict
            for j = 1:num_point
                testPoint(j,:) = [testR*cos((j-1)*2*pi/num_point)+curr_temp(1), ...
                                  testR*sin((j-1)*2*pi/num_point)+curr_temp(2)];
                testOut(j) = comput_P(testPoint(j,:)'; , guidePos2', obstacle, Q_star);
            end
            [~,idx] = min(testOut);
            curr_temp = testPoint(idx,:);
            pos_predict2(i,1:2) = curr_temp;
        end
    else
        x_start = guidePos2(1) - testR*cos(theta_gui2)*accu2;
        y_start = guidePos2(2) - testR*sin(theta_gui2)*accu2;
        accu2 = accu2 + 1;
        idx = (1:step_predict)';
        pos_predict2(:,1) = x_start - testR*cos(theta_gui2)*idx;
        pos_predict2(:,2) = y_start - testR*sin(theta_gui2)*idx;
    end
    
    % ================= Plot prediction sets and environment =================
    plot3(pos_predict(:,1), pos_predict(:,2), pos_predict(:,3), 'Color','green','linewidth',2);
    plot3([pos_predict(1,1),curr(1)], [pos_predict(1,2),curr(2)], [pos_predict(1,3),curr(3)], 'Color','green','linewidth',2);
    
    for j = 1:size(obstacle,2)
        plot_obstacle(obstacle(1,j), obstacle(2,j), Q_star/2, h2*1.5);
    end
    
    plot3(guidePos(1), guidePos(2), h,  '*','Color',all_colors(5,:),'MarkerSize
