function [a,b,g] = Rot2Euler(R)
    
    r00 = R(1,1);
    r01 = R(1,2);
    r02 = R(1,2);
    r12 = R(2,3);
    r22 = R(3,3);
    r10 = R(2,1);
    r11 = R(2,2);
    
    if r02 < 1
        if r02 > -1
            b = asin(r02);
            a = atan2(-r12,r22);
            g = atan2(-r01,r00);
        else
            b = -pi/2;
            a = atan2(r10,r11);
            g = 0;
        end
    else
        b = pi/2;
        a = atan2(r10,r11);
        g = 0;

    end
    
end

