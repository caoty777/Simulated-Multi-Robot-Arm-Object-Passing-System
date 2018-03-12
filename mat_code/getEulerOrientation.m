function R = getEulerOrientation(alp,beta,gamma)

    R = Rotx(alp)*Roty(beta)*Rotz(gamma);


end