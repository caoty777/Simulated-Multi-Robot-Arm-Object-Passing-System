function brkt = braket3(p)

    a = p(1);
    b = p(2);
    c = p(3);
    
    brkt = [ 0 -c b;
             c 0 -a;
            -b a 0];

end