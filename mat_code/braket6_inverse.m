function V = braket6_inverse(R)
    w = braket3_inverse( getR(R) );
    v = getP(R);
    V = [w;v];
end