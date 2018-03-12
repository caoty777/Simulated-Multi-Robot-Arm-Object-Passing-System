function Ad = Adj(H)
    
    R = getR(H);
    p = getP(H);
    zer = zeros(3);
    brkt = braket3(p);
    
    Ad = [  R zer ; brkt*R R ]; 
end