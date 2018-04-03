function J = jacobian(S, theta)
% Finds the space Jacobian of an 6xn matrix, where n is the number of joints
% a matrix of n spatial screw axis S = [S1 S2 ... Sn]
% and a matrix of n thetas. theta can be 1xn or nx1
    % find n based from columns of S
    [m,n] = size(S);
    J = zeros(6,n);
    % Set first column of jacobian to S1
    J(:,1) = S(:,1);
    if n == 1
        return
    end
    A_o = 1;
    % Build jacobian for rest of columns
    for N = 2:n
        % Uses S and theta from 1 to n-1
        A_n = A_o * adjoint(expm(skew4(S(:,N-1))*theta(N-1)));
        J(:,N) = A_n * S(:,N);
        A_o = A_n;
    end
end
