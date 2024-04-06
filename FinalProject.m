clear;
addpath('mr'); % add mr library to path

%% To run the code
% There are 3 tasks in total, namely "Best", "Overshooting", "New Task"

%%%%%%%%%% BEST
%  Cube location: selected by default [initial (Tsc_initial) & goal (Tsc_goal)]
%  Gains: Kp and Ki selected by default

%%%%%%%%%% OVERSHOOTING
%  Cube location: selected by default [initial (Tsc_initial) & goal (Tsc_goal)]
%  Gains: Kp and Ki select the one mentioned for OVERSHOOTING

%%%%%%%%%% NEW TASK
%  Cube location: uncomment next set of values [initial (Tsc_initial) &
%  goal (Tsc_goal)]
%  Gains: Kp and Ki select the one mentioned for NEW TASK

%%%%%%%%%% Results
% generates an error plot
% state results are in "state_next1.csv"
%% Variables
% initial state of the robot
% state = [phi, x, y, J1, J2, J3, J4, J5, W1, W2, W3, W4];
state = [-0.2 -0.6 -0.01 0.3 -0.15 -1.9 -0.15 0.15 0 0 0 0 0];

k = 1;
time_step = 0.01;   % % The timestep Δt between reference trajectory configurations
max_velocity = 30;  % maximum joint and wheel velocities (kept uniform here)

% initial state of the robot
phi = state(1);
x = state(2);
y = state(3);

% config of base of chassis {b} relative to {s}
Tsb = [[cos(phi), -sin(phi), 0, x];
       [sin(phi), cos(phi), 0, y];
       [0, 0, 1, 0.0963];
       [0, 0, 0, 1]];

% fixed offset from {b} to base of arm {0}
Tb0 = [[1, 0, 0, 0.1662];
        [0, 1, 0, 0];
        [0, 0, 1, 0.0026];
        [0, 0, 0, 1]];

% TF from {0} to end effector {e} (when in home position)
M0e = [[1, 0, 0, 0.033];
        [0, 1, 0, 0];
        [0, 0, 1, 0.6546];
        [0, 0, 0, 1]];

% Screw axes for home configuration
B1 = [0; 0; 1; 0; 0.033; 0];
B2 = [0; -1; 0; -0.5076; 0; 0];
B3 = [0; -1; 0; -0.3526; 0; 0];
B4 = [0; -1; 0; -0.2176; 0; 0];
B5 = [0; 0; 1; 0; 0; 0];

Blist = [B1 B2 B3 B4 B5];
thetalist = state(4:8)';
T0e = FKinBody(M0e, Blist, thetalist);

% gripper variables
d1_min = 0.02;  % [m] gripper minimum close
d1_max = 0.07;  % [m] gripper maximum open
d2 = 0.035;     % [m] interior length of gripper fingers
d3 = 0.043;     % [m] distance from base of fingers to {e}

%% Location of EE and Cube
% Cube location (initial and final) for Best and Overshoot task
Tsc_initial = [[1, 0, 0, 1];
               [0, 1, 0, 0];
               [0, 0, 1, 0.025];
               [0, 0, 0, 1]];

Tsc_goal = [[0, 1, 0, 0];
           [-1, 0, 0, -1];
            [0, 0, 1, 0.025];
            [0, 0, 0, 1]];

% % Cube location (initial and final) for NEW TSAK
% Tsc_initial = [[1, 0, 0, 1];
%                [0, 1, 0, 0.5];
%                [0, 0, 1, 0.025];
%                [0, 0, 0, 1]];
% 
% Tsc_goal = [[0, 1, 0, -0.5];
%            [-1, 0, 0, -1];
%             [0, 0, 1, 0.025];
%             [0, 0, 0, 1]];
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

Tse_initial = [[0, 0, 1, 0];
               [0, 1, 0, 0];
               [-1, 0, 0, 0.5];
               [0, 0, 0, 1]];


Tce_grasp = [[cos(3*pi/4), 0, sin(3*pi/4), 0];
             [0, 1, 0, 0];
             [-sin(3*pi/4), 0, cos((3*pi/4)), -0.01];
             [0, 0, 0, 1]];

Tce_standoff = [[cos(3*pi/4), 0, sin(3*pi/4), 0];
                [0, 1, 0, 0];
                [-sin(3*pi/4), 0, cos((3*pi/4)), 0.15];
                [0, 0, 0, 1]];
% ------------
%% Controller Gains
% The PI gain matrices Kp and Ki
    %%%%%%%%%%%%%%% for BEST TASK
    Kp = 2 * [1 0 0 0 0 0;
        0 1 0 0 0 0;
        0 0 1 0 0 0;
        0 0 0 1 0 0;
        0 0 0 0 1 0;
        0 0 0 0 0 0.4];

    Ki = 0.02 * [1 0 0 0 0 0;
        0 1 0 0 0 0;
        0 0 1 0 0 0;
        0 0 0 1 0 0;
        0 0 0 0 1 0;
        0 0 0 0 0 1.5];

    %%%%%%%%%%%%% for OVERSHOOT 
    % Kp = 1.9 * [1 0 0 0 0 0;
    %     0 1 0 0 0 0;
    %     0 0 1 0 0 0;
    %     0 0 0 1 0 0;
    %     0 0 0 0 1 0;
    %     0 0 0 0 0 0.4];
    % 
    % Ki = 2.2 * [1 0 0 0 0 0;
    %     0 1 0 0 0 0;
    %     0 0 1 0 0 0;
    %     0 0 0 1 0 0;
    %     0 0 0 0 1 0;
    %     0 0 0 0 0 1.5];

    %%%%%%%%%%%%%% for NEW TASK
    % Kp = 2.2 * [1 0 0 0 0 0;
    %     0 1 0 0 0 0;
    %     0 0 1 0 0 0;
    %     0 0 0 1 0 0;
    %     0 0 0 0 1 0;
    %     0 0 0 0 0 0.4];
    % 
    % Ki = 0.015 * [1 0 0 0 0 0;
    %     0 1 0 0 0 0;
    %     0 0 1 0 0 0;
    %     0 0 0 1 0 0;
    %     0 0 0 0 1 0;
    %     0 0 0 0 0 1.5];

%% Trajectory Generator
[generated_trajectory, tf] = TrajectoryGenerator(Tse_initial, ...
    Tsc_initial, Tsc_goal, Tce_grasp, Tce_standoff, k, 5);
writematrix(generated_trajectory, 'generated_trajectory.csv')

%% Feedforward plus PI Controller
% state = [0, 0, 0, 0, 0, 0.2,-1.6, 0, 0, 0, 0, 0];
X_err_next = zeros(length(generated_trajectory)-1, 6);
total_err_dt = [0; 0; 0; 0; 0; 0];  % error from current to desired (velocity)
next_state1 = zeros(length(generated_trajectory), 13);

for i=1:length(generated_trajectory)-1
% for i=1
    % The current actual end-effector configuration X (aka Tse)
    % state of the robot
    phi = state(i, 1);
    x = state(i, 2);
    y = state(i, 3);

    % config of base of chassis {b} relative to {s}
    Tsb = [[cos(phi), -sin(phi), 0, x];
           [sin(phi), cos(phi), 0, y];
           [0, 0, 1, 0.0963];
           [0, 0, 0, 1]];

    Blist = [B1 B2 B3 B4 B5];
    thetalist = state(i, 4:8)';
    T0e = FKinBody(M0e, Blist, thetalist);

    X = Tsb * Tb0 * T0e;

    % The current reference end-effector configuration Xd (aka Tse,d)
    Xd = row2SE3(generated_trajectory(i,:));
    % Xd = [0, 0, 1, 0.5; 0, 1, 0, 0; -1, 0, 0, 0.5; 0, 0, 0, 1];
    
    % The reference end-effector configuration at the next timestep, Xd,next (aka Tse,d,next)
    Xd_next = row2SE3(generated_trajectory(i+1,:));
    % Xd_next = [0, 0, 1, 0.6; 0, 1, 0, 0; -1, 0, 0, 0.3; 0, 0, 0, 1];
    
    % define jacobian
    J_body = JacobianBody(Blist, thetalist);

    % Mecunum Wheel Mobile Robot fixed parameters
    l = 0.47/2;  % l = halh length
    w = 0.3/2;   % w = half width
    r = 0.0475;  % r = radius of the wheel

    H = (r/4)*[[-1/(l+w), 1/(l+w), 1/(l+w), -1/(l+w)];
    [1, 1, 1, 1];
    [-1, 1, -1, 1]];

    F_matrix = [[0, 0, 0, 0];
        [0, 0, 0, 0];
        H;
        [0, 0, 0, 0]];

    J_base = Adjoint(pinv(T0e, 1e-3)*pinv(Tb0, 1e-3))*F_matrix;
    Je = [J_base, J_body];

    [twist_ee, velocity, X_err_dt] = FeedbackControl(X, Xd, Xd_next, Kp, Ki, time_step, total_err_dt, Je);

    % update input arguments for next desired state 
    velocity = velocity';
    next_state1(i,1:12) = NextState(state(i,1:12), velocity, time_step, max_velocity);
    next_state1(i,13) = generated_trajectory(i,13);
    state = [state; next_state1(i,:)];

    total_err_dt = total_err_dt + X_err_dt;

    X_err_next(i, :) = X_err_dt';

end

%% Results
% write csv file to be uploaded in CoppeliaSim
writematrix(next_state1, 'next_state1.csv')
save('X_err.mat', 'X_err_next');

%% Plots
% Define time vector (each iteration represents 0.01 seconds)
time = (0:0.01:(size(X_err_next, 1) - 1) * 0.01)';

% Extract components of error twist
error_components = {'wx', 'wy', 'wz', 'vx', 'vy', 'vz'};

% Plot all components in one graph
figure;
hold on;
for i = 1:6
    plot(time, X_err_next(:, i), 'DisplayName', error_components{i});
end
hold off;
title('Error Twist Components during wait-time', 'FontSize', 14); % Set title font size
xlabel('Time (seconds)', 'FontSize', 18); % Set x-axis label font size
ylabel('Error Value', 'FontSize', 18); % Set y-axis label font size
legend('Location', 'best', 'FontSize', 24); % Set legend font size
set(gca, 'FontSize', 18); % Set axis tick label font size
grid on;
