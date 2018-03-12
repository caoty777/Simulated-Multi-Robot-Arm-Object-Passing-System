function J = Body_Jacobian(b,th)

    dataSize = size(b);
    dataSize = dataSize(2);
    B = zeros(4,4,dataSize,'like',b);
    E = zeros(4,4,dataSize,'like',b);
    A = zeros(6,6,dataSize,'like',b);
	J = zeros(6,dataSize,'like',b);
    
    E(:,:,dataSize) = eye(4);
    A(:,:,dataSize) = eye(6);
    
    for i=1:dataSize
        B(:,:,i) = braket6( b(:,i) );
    end
    
    for i = 1:dataSize-1
        E(:,:,dataSize-i) = E(:,:,dataSize-i+1)*expm(-B(:,:,dataSize-i+1)*th(dataSize-i+1) );
        A(:,:,dataSize-i) = Adj(E(:,:,dataSize-i));
    end
    
    for i = 1:dataSize
        J(:,i) = A(:,:,i)*b(:,i);
    end

end