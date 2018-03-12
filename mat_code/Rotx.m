function R = Rotx(angle)
    R = [ 1 0 0; 0 c(angle) -s(angle); 0 s(angle) c(angle)]; % Rot about x axis
end