function s = skew3(x)
% Creates a 3x3 skew symetric matrix from a 3x1 vector
    s = [0 -x(3) x(2) ; x(3) 0 -x(1) ; -x(2) x(1) 0 ];
end
