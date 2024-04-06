function [twist_ee, velocity, X_err_dt] = FeedbackControl(X, Xd, Xd_next, Kp, Ki, dt, total_err_dt, Je)

%% Description
% Input: [actual pose, reference pose, desired next pose...
%  gains, timestep, total twist error, Jacobian]
% 
% Output: [twist in end effector frame, velocity of all the joint (u,
% theta_dot), error in the twist]


%% 
% error twist Xerr that takes X to Xd
X_err = se3ToVec(MatrixLog6(pinv(X, 1e-3)*Xd));

X_err_dt = X_err * dt;
total_err_dt = total_err_dt + X_err_dt;
%% 
% The feedforward reference twist Vd that takes Xd to Xd,next in time Δt
Vd_err = se3ToVec((1/dt)*(MatrixLog6(pinv(Xd, 1e-3)*Xd_next)));

%%
% feedback control law gives the commanded EE twist V in the EE frame
% Kp = eye(6);
% Ki = zeros(6);
twist_ee = (Adjoint(pinv(X, 1e-3)*Xd) * Vd_err) + (Kp * X_err) + (Ki * total_err_dt);

%%
% turn twist_ee into commanded wheel and arm joint speeds
velocity = pinv(Je, 1e-3) * twist_ee;

end