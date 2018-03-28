function [th msg] = inverseKinematics(M,R,S,e)
    
    msg = 0;
    sz = size(S);
    sz = sz(2);
    %%%%
    thetaInit = getThetas(M,R,S);
    th = thetaInit;
    %%%%
    alpha = 1;
    E = zeros(4,4,sz+1);
    E(:,:,1) = eye(4);
    k = 0;
    er = 5;
    
    for i=2:sz+1
        E(:,:,i) = E(:,:,i-1)*expm(braket6(S(:,i-1))*th(i-1) );
    end
    T = E(:,:,sz+1)*M;
    V = logm(R*inv(T));
    v = braket6_inverse(V);
    while er > e
        
        for i=2:sz+1
            E(:,:,i) = E(:,:,i-1)*expm(braket6(S(:,i-1))*th(i-1) );
        end
        T = E(:,:,sz+1)*M;
        V = logm(R*inv(T));
        v = braket6_inverse(V);
        J = Space_Jacobian(S,th);
        thetadot = J\v;
        th = th+thetadot*alpha;
        er = norm(V);
        if k > 100
            er = 0.0;
            disp("NO SOLUTION!!")
            msg = 1;
            return
        end
        if msg == 1
            return 
        end
        k=k+1;
    end
    if msg == 1
        return
    end
end