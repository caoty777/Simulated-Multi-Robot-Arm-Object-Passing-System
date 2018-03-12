function brkt = braket6(p)

     W = braket3(p(1:3));   
     v = p(4:6);
     brkt = getH(W,v);
     brkt(4,4)=0;

end