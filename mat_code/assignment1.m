clear
clc
close all
%====================================================================
% My Variables
%====================================================================
ArmJoints = ['Jaco_joint1','Jaco_joint2','Jaco_joint3',...
             'Jaco_joint4','Jaco_joint5','Jaco_joint6'];
         
JointAngles = zeros(1,6);
joint_handles = zeros(1,6);

%====================================================================
% Initialize session and Start Simulation
%====================================================================
vrep=remApi('remoteApi');
vrep.simxFinish(-1);
clientID=vrep.simxStart('127.0.0.1',19997,true,true,5000,5);
if (clientID>-1)
    disp('Connected to remote API server');
end
vrep.simxStartSimulation(clientID, vrep.simx_opmode_oneshot);
pause(2)

%====================================================================
% Get "handle" to joints of robot
%====================================================================
for i = 0:5
    [result, joint_handles(i+1)] = vrep.simxGetObjectHandle(clientID, ArmJoints(11*i+1:11*i+11), vrep.simx_opmode_blocking);
    if result ~= vrep.simx_return_ok
        disp('could not get object handle for first joint')
    end
end

%====================================================================
% Get the values of all joint variables
%====================================================================
% First joint variable
for i = 1:6
    [result, JointAngles(i)] = vrep.simxGetJointPosition(clientID, joint_handles(i), vrep.simx_opmode_blocking);
    if result ~= vrep.simx_return_ok
        disp('could not get first joint variable')
    end
    sprintf('current value of first joint variable: theta%d = %d', i,JointAngles(i))
end


%====================================================================
% Set the desired values of joints variable
%====================================================================
for i = 1:6
    vrep.simxSetJointTargetPosition(clientID, joint_handles(i), JointAngles(i)+pi/2, vrep.simx_opmode_oneshot);
end
pause(3)

%====================================================================
% Get the values of all joint variables
%====================================================================
% First joint variable
for i = 1:6
    [result, JointAngles(i)] = vrep.simxGetJointPosition(clientID, joint_handles(i), vrep.simx_opmode_blocking);
    if result ~= vrep.simx_return_ok
        disp('could not get first joint variable')
    end
    sprintf('current value of first joint variable: theta%d = %d', i,JointAngles(i))
end

%====================================================================
% Stop Simulation & Finish session
%====================================================================
vrep.simxStopSimulation(clientID, vrep.simx_opmode_oneshot);
vrep.simxGetPingTime(clientID);
vrep.simxFinish(clientID);
