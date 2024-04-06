function T = row2SE3(vec)
% converts row vector obtained from reference trajectory to SE(3)
    T = [[vec(1,1) vec(1,2) vec(1,3) vec(1,10)];
        [vec(1,4) vec(1,5) vec(1,6) vec(1,11)];
        [vec(1,7) vec(1,8) vec(1,9) vec(1,12)];
        [0 0 0 1]];
end