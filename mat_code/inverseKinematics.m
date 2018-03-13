function th = inverseKinematics(M,R,S,e)
    
    sz = size(S);
    sz = sz(2);
    thetaInit = rand(sz,1);
    th = thetaInit;
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
        if k > 1000
            er = 0.0;
            disp("not found with 1000")
        end
        k=k+1;
    end
end