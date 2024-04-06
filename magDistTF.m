function magdist = magDistTF(tfMat1, tfMat2)
% relative translation between two transformation matrices
magdist = norm(tfMat2(1:3,4) - tfMat1(1:3,4));
end

