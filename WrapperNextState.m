clear;
addpath('mr'); % add mr library to path

%% Component 2
% this is a wrapper function to run the NextState.m function

% This function helps in estimating the state of the robot at the next state
% It takes current state, velocity, timestep, and max vel as input arguments
% It returns next state of the robot as an output

%% Next State
state = [0 0 0 0 0 0 0 0 0 0 0 0 0];

time_step = 0.01;   % % The timestep Δt between reference trajectory configurations

T = 1;   % Robot run time
max_iterations = T/time_step;

% Joint and wheel velocities
% velocity = [u1, u2, u3, u4, J1_dot, J2_dot, J3_dot, J4_dot, J5_dot];
velocity = [10, 10, 10, 10, 0, 0, 0, 0, 0];
max_velocity = 30;  % maximum joint and wheel velocities (kept uniform here)

% initializing the next state
next_state = zeros(max_iterations, 13);

for i=1:max_iterations
    next_state(i,1:12) = NextState(state, velocity, time_step, max_velocity);
    state = next_state(i,1:12);
end

writematrix(next_state, 'wrapper_next_state.csv')