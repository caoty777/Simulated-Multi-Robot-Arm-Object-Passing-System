function S = unskew4(x)
% Retrieves a 6x1 vector from a 4x4 skew symetric matrix
    S = [x(3,2); -x(3,1); x(2,1); x(1,4); x(2,4); x(3,4)];
end
