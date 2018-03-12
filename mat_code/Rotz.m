function R = Rotz(angle)
    R = [ c(angle) -s(angle) 0; s(angle) c(angle) 0; 0 0 1]; % Rot about z axis
end