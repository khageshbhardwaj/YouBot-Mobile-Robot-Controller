clear;
addpath('mr'); % add mr library to path

%% 
% this is a wrapper function to check the correctness of the FeedbackControl function
% IN: given hereby next
% OUT: twist in end effector frame and velocity

%% Feedforward plus PI Controller
state = [0, 0, 0, 0, 0, 0.2,-1.6, 0, 0, 0, 0, 0];
total_err_dt = [0; 0; 0; 0; 0; 0];  % error from current to desired (velocity)
time_step = 0.01;   % % The timestep Δt between reference trajectory configurations


% The current actual end-effector configuration X (aka Tse)
% state of the robot
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

X = Tsb * Tb0 * T0e;
Kp = eye(6);
Ki = eye(6);

% The current reference end-effector configuration Xd (aka Tse,d)
Xd = [0, 0, 1, 0.5; 0, 1, 0, 0; -1, 0, 0, 0.5; 0, 0, 0, 1];

% The reference end-effector configuration at the next timestep, Xd,next (aka Tse,d,next)
Xd_next = [0, 0, 1, 0.6; 0, 1, 0, 0; -1, 0, 0, 0.3; 0, 0, 0, 1];

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
