function s = skew4(x)
% Creates a 4x4 skew symetric matrix from a 6x1 vector
    s = [skew3(x) [x(4); x(5); x(6)]; 0 0 0 0];
end
