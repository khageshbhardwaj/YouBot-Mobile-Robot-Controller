clear;
addpath('mr'); % add mr library to path

%% Component 1
% this is a wrapper function to generate reference trajectory

% This code generates the trajectory for the given initial and final cube
% location. One is free to change those values if they wat to generate new
% trajectories.

% % % Input
% Function takes initial starting pose.
% Initial cube pose
% final cube pose
% grasp pose
% standoff pose over the cube

% Output
% generates a .csv file which contains reference trajecories
% first 9 values are part of rotation and next 3 are the positional
% arguments
% last value represents the gripper state

%% Location of EE and Cube
k = 1;

Tsc_initial = [[1, 0, 0, 1];
               [0, 1, 0, 0];
               [0, 0, 1, 0.025];
               [0, 0, 0, 1]];

Tsc_goal = [[0, 1, 0, 0];
           [-1, 0, 0, -1];
            [0, 0, 1, 0.025];
            [0, 0, 0, 1]];

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
% Trajectory Generator
[generated_trajectory, tf] = TrajectoryGenerator(Tse_initial, ...
    Tsc_initial, Tsc_goal, Tce_grasp, Tce_standoff, k, 5);
writematrix(generated_trajectory, 'wrapper_generated_trajectory.csv')