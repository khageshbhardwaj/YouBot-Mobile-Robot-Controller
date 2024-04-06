function [gen_traj, tf] = TrajectoryGenerator(Tse_i, Tsc_i, Tsc_f, Tce_g, Tce_s, k, vel)
% IN: initial Tse (e-e), initial Tsc (cube), desired final Tsc (cube),
%       Tce when grasping, Tce standoff config before & after grasping,
%       k: number of trajectory reference configs per 0.01s
%       time: time of segment
%       ts: timestep
% OUT: r11, r12, r13, r21, r22, r23, r31, r32, r33, px, py, pz, gripper state
% use ScrewTrajectory or CartesianTrajectory

% calculating time requierd in between trans (change in distance of end-effector position)/max_velocity
t1 = magDistTF(Tse_i, Tsc_i*Tce_s)/vel;
t2 = magDistTF(Tsc_i*Tce_s, Tsc_i*Tce_g)/vel;
t4 = magDistTF(Tsc_i*Tce_g, Tsc_i*Tce_s)/vel;
t5 = magDistTF(Tsc_i*Tce_s, Tsc_f*Tce_s)/vel;
t6 = magDistTF(Tsc_f*Tce_s, Tsc_f*Tce_g)/vel;
t8 = magDistTF(Tsc_f*Tce_g, Tsc_f*Tce_s)/vel;

tf = [t1*5, t2*4, 0.63, t4*3, t5*6, t6*6, 0.63, t8*4]; % [s] time for each segment
N = [3.2 1.8 1 1.8 3.2 1.8 1 1.8]*k/0.01; % number of reference trajectories per segment

% N = round(tf*k/0.01);  

gen_traj = zeros(sum(N),13);     % initialize output array for trajectory generated

% 1. Move gripper to initial "standoff" position
Ts1 = ScrewTrajectory(Tse_i, Tsc_i*Tce_s, tf(1), N(1), 5);
gen_traj(1:N(1),:) = [flatTF(Ts1) zeros(N(1),1)];

% 2. Move gripper to initial "grasp" position
Ts2 = ScrewTrajectory(Tsc_i*Tce_s, Tsc_i*Tce_g, tf(2), N(2), 5);
gen_traj((N(1)+1):sum(N(1:2)),:) = [flatTF(Ts2) zeros(N(2),1)];

% 3. Close gripper
for i=(sum(N(1:2))+1):sum(N(1:3))
    gen_traj(i,:) = [gen_traj(i-1,1:12) 1];
end

% 4. Move gripper from "grasp" to "standoff"
Ts4 = ScrewTrajectory(Tsc_i*Tce_g, Tsc_i*Tce_s, tf(4), N(4), 5);
gen_traj((sum(N(1:3))+1):sum(N(1:4)),:) = [flatTF(Ts4) ones(N(4),1)];

% 5. Move gripper from "standoff" above initial to "standoff" above final
Ts5 = ScrewTrajectory(Tsc_i*Tce_s, Tsc_f*Tce_s, tf(5), N(5), 5);
gen_traj((sum(N(1:4))+1):sum(N(1:5)),:) = [flatTF(Ts5) ones(N(5),1)];

% 6. Move gripper from "standoff" above final to "gripper" above final 
Ts6 = ScrewTrajectory(Tsc_f*Tce_s, Tsc_f*Tce_g, tf(6), N(6), 5);
gen_traj((sum(N(1:5))+1):sum(N(1:6)),:) = [flatTF(Ts6) ones(N(6),1)];

% 7. Open gripper
for i=(sum(N(1:6))+1):sum(N(1:7))
    gen_traj(i,:) = [gen_traj(i-1,1:12) 0];
end

% 8. Move gripper from final configuration to "standoff"
Ts8 = ScrewTrajectory(Tsc_f*Tce_g, Tsc_f*Tce_s, tf(8), N(8), 5);
gen_traj((sum(N(1:7))+1):sum(N(1:8)),:) = [flatTF(Ts8) zeros(N(8),1)];
end