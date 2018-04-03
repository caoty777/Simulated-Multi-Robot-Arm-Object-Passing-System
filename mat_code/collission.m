function collission(vrep,clientID,points,radii)

    n = max(size(radii));
    collided = 0;
    
    for i = 0:n-1
        [result, hand_handles(i+1)] = vrep.simxGetObjectHandle(clientID, points(6*i+1:6*i+6), vrep.simx_opmode_blocking);
        if result ~= vrep.simx_return_ok
            sprintf('could not get handle for %s', points(6*i+1:6*i+6))
        end
        [result, q(:,i+1)] = vrep.simxGetObjectPosition(clientID,hand_handles(i+1),-1,vrep.simx_opmode_blocking);
        if result ~= vrep.simx_return_ok
            sprintf('could not get position for %s', points(6*i+1:6*i+6))
        end
    end

    for i=1:n
        for j=i:n
            if( norm(q(:,i)-q(:,j)) <= radii(i)+radii(j) && ( j ~= i ))
                if( (i~=6 && j~=7) && (i~=4 && j~=5) && (i~=4 && j~=9) )
                sprintf("joint %s and joint %s are in collission!",points(6*(i-1)+1:6*(i-1)+6),points(6*(j-1)+1:6*(j-1)+6))
                %q,q(:,i),q(:,j),nrm = norm(q(:,i)-q(:,j)),dist = radii(i)+radii(j),i,j
                collided = 1;
                end
            end
        end
    end
    
    if collided == 0
        disp("No Collission")
    end
    
end
