function [next_state] = NextState(state, velocity, time_step, max_velocity)

% unpacking the urrent state
old_chesis_state = state(1:3);
old_joint_angles = state(4:8);
old_wheel_angles = state(9:12);

% unpacking the velocities
old_wheel_velocity = velocity(1:4);
old_joint_velocity = velocity(5:9);

for i=1:4
    if abs(old_wheel_velocity(i)) > max_velocity
        old_wheel_velocity (i) = sign(old_wheel_velocity(i)) * max_velocity;       
    end
end

% updated join angles
new_joint_angles = old_joint_angles + (old_joint_velocity * time_step);

% updated wheel rotations
new_wheel_angles = old_wheel_angles + (old_wheel_velocity * time_step);

% Mecunum Wheel Mobile Robot fixed parameters
l = 0.47/2;  % l = halh length
w = 0.3/2;   % w = half width
r = 0.0475;  % r = radius of the wheel

H = (r/4)*[[-1/(l+w), 1/(l+w), 1/(l+w), -1/(l+w)];
    [1, 1, 1, 1];
    [-1, 1, -1, 1]];

vb = H * (old_wheel_velocity)' * time_step;

% new_chesis_state = old_chesis_state *time_step;

if vb(1) == 0
    delta_qb = [0; vb(2); vb(3)];
else 
    delta_qb = [vb(1);
                (vb(2)*sin(vb(1)) + vb(3)*(cos(vb(1)) - 1))/vb(1);
                (vb(3)*sin(vb(1)) + vb(2)*(1 - cos(vb(1))))/vb(1)];

end

phi = old_chesis_state(1);

delta_q = [1 0 0; 0 cos(phi) -sin(phi); 0 sin(phi) cos(phi)] * delta_qb;

chesis_state = old_chesis_state + (delta_q)';

next_state = [chesis_state new_joint_angles new_wheel_angles];

end



