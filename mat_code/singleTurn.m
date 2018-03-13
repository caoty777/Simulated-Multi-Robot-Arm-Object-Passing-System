function th = singleTurn(theta)

        sz = max(size(theta));
        th = zeros(1,sz);
        for i = 1:sz
            if theta(i) >= 2*pi || theta(i) <= -2*pi
                th(i) = mod(theta(i),2*pi);
            else
                th(i) = theta(i);
            end
        end
end