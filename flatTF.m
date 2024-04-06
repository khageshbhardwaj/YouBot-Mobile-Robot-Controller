function matOut = flatTF(tf_cell)
% "flattens" R and p matrices into single vector
% IN: cell array of transformation matrices
% OUT: 2D array where each row contains the tranformation matrix R & p 
% as vector[r11, r12, r13, r21, r22, r23, r31, r32, r33, px, py, pz]

matOut = zeros(length(tf_cell), 12);
for idx=1:length(tf_cell)
   matOut(idx,:) = [tf_cell{idx}(1,1:3), tf_cell{idx}(2,1:3), ...
       tf_cell{idx}(3,1:3), tf_cell{idx}(1:3,4)']; 
end
end