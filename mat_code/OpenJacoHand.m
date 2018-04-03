function OpenJacoHand(vrep,clientID)


%====================================================================
% Get "handles"
%====================================================================
% Get "handle" to joints of Jaco hand

[result,j0] = vrep.simxGetObjectHandle(clientID,'JacoHand_fingers12_motor1',vrep.simx_opmode_blocking);
if result ~= vrep.simx_return_ok
    disp("could not get handle for JacoHand_fingers12_motor1")
end
[result,j1] = vrep.simxGetObjectHandle(clientID,'JacoHand_fingers12_motor2',vrep.simx_opmode_blocking);
if result ~= vrep.simx_return_ok
    disp("could not get handle JacoHand_fingers12_motor2")
end
[result,j2] = vrep.simxGetObjectHandle(clientID,'JacoHand_finger3_motor1',vrep.simx_opmode_blocking);
if result ~= vrep.simx_return_ok
    disp("could not get handle JacoHand_finger3_motor1")
end
[result,j3] = vrep.simxGetObjectHandle(clientID,'JacoHand_finger3_motor2',vrep.simx_opmode_blocking);
if result ~= vrep.simx_return_ok
    disp("could not get handle JacoHand_finger3_motor2")
end
closingVel = -0.04;

for i=1:5

    %====================================================================
    % Open the gripper
    %====================================================================
    result = vrep.simxSetJointTargetVelocity(clientID,j0,-closingVel,vrep.simx_opmode_blocking);
    if result ~= vrep.simx_return_ok
        disp("could not set joint velocity for JacoHand_fingers12_motor1")
    end
    result = vrep.simxSetJointTargetVelocity(clientID,j1,-closingVel,vrep.simx_opmode_blocking);
    if result ~= vrep.simx_return_ok
        disp("could not set joint velocity for JacoHand_fingers12_motor2")
    end
    result = vrep.simxSetJointTargetVelocity(clientID,j2,-closingVel,vrep.simx_opmode_blocking);
    if result ~= vrep.simx_return_ok
        disp("could not set joint velocity for JacoHand_finger3_motor1")
    end
    result = vrep.simxSetJointTargetVelocity(clientID,j3,-closingVel,vrep.simx_opmode_blocking);
    if result ~= vrep.simx_return_ok
        disp("could not set joint velocity for JacoHand_finger3_motor1")
    end

    pause(1)
end

end