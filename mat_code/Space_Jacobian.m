function J = Space_Jacobian(s,th)

    dataSize = size(s);
    dataSize = dataSize(2);
    S = zeros(4,4,dataSize,'like',s);
    E = zeros(4,4,dataSize,'like',s);
    A = zeros(6,6,dataSize,'like',s);
	J = zeros(6,dataSize,'like',s);
    
    E(:,:,1) = eye(4);
    A(:,:,1) = eye(6);
    
	for i=1:dataSize
        S(:,:,i) = braket6( s(:,i) );
    end
    
	for i = 2:dataSize
        E(:,:,i) = E(:,:,i-1)*expm(S(:,:,i-1)*th(i-1) );
        A(:,:,i) = Adj(E(:,:,i));
    end
    
	for i = 1:dataSize
        J(:,i) = A(:,:,i)*s(:,i);
	end

end