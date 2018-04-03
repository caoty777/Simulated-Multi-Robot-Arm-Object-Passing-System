clear
clc

%====================================================================
% My Variables
%====================================================================
ArmJoints = ['Jaco_joint1','Jaco_joint2','Jaco_joint3',...
             'Jaco_joint4','Jaco_joint5','Jaco_joint6'];
joint_handles = zeros(1,6);
JointAngles = zeros(1,6);
Dummies = ['Dummy1','Dummy2','Dummy3',...
           'Dummy4','Dummy5','Dummy6','Dummy0'...
           'Dummy7','Dummy8','Dummy9'];
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
% Get "handles
% = = = = = = =
%====================================================================
% Get "handle" to joints of robot to move the Jaco arm
for i = 0:5
    [result, joint_handles(i+1)] = vrep.simxGetObjectHandle(clientID, ArmJoints(11*i+1:11*i+11), vrep.simx_opmode_blocking);
    if result ~= vrep.simx_return_ok
        sprintf('could not get joint handle%d = %s', i,ArmJoints(11*i+1:11*i+11))
    end
end
%%% Get the handle of Jaco object
[result, jaco_handle] = vrep.simxGetObjectHandle(clientID, 'Jaco', vrep.simx_opmode_blocking);
if result ~= vrep.simx_return_ok
	disp('could not get Jaco handle')
end
%%% Get the handle of end effector (dummy7 objec)
[result, dummy_handle] = vrep.simxGetObjectHandle(clientID, 'JacoHand_Dummy7', vrep.simx_opmode_blocking);
if result ~= vrep.simx_return_ok
	disp('could not get dummy7 handle')
end
%%% Get the handle of object to track (dummy objec)
[result, dummy_ball_handle] = vrep.simxGetObjectHandle(clientID, 'Dummy', vrep.simx_opmode_blocking);
if result ~= vrep.simx_return_ok
	disp('could not get dummy ball handle')
end
pause(1)

%====================================================================
% Inverse Kinematics
% = = = = = = = = = 
% First obtain R and p for the base frame (Jaco) w.r.t the world frame
% and then S w.r.t the world frame
%====================================================================
%%% Get the position of base frame w.r.t the world frame
[result, p] = vrep.simxGetObjectPosition(clientID,jaco_handle,-1,vrep.simx_opmode_blocking);
if result ~= vrep.simx_return_ok
	disp('could not get Jaco position')
end
%%% Get the orientation of base w.r.t the world frame
[result , euler_angles] = vrep.simxGetObjectOrientation(clientID, jaco_handle, -1, vrep.simx_opmode_blocking);
if result ~= vrep.simx_return_ok
	disp('could not get Jaco orientation')
end
R_base_in_world = getEulerOrientation(euler_angles(1),euler_angles(2),euler_angles(3));
p_base_in_world = p';

OpenJacoHand(vrep,clientID)
disp(" ")
disp("Initial Pose")
disp(" ")
collission(vrep,clientID,Dummies,[0.1 0.1 0.1 0.1 0.07 0.07 0.13 0.1 0.1 0.1]/2)
pause(2)

for K_loop =1:5
%====================================================================
% Find S = [S1 S2 ...]
% = = = = = = = = = =
% a,q without fk are w.r.t the base frame
% a_fk and q_fk are w.r.t the world frame
%====================================================================
for i=1:6

[result, Euler] = vrep.simxGetObjectOrientation(clientID, joint_handles(i), jaco_handle, vrep.simx_opmode_blocking) ;
if result ~= vrep.simx_return_ok
	sprintf('could not get joint1 orientation: theta%d', i)
end
[result, q] = vrep.simxGetObjectPosition(clientID, joint_handles(i), jaco_handle, vrep.simx_opmode_blocking) ;
if result ~= vrep.simx_return_ok
	sprintf('could not get joint1 position: theta = %d', i,JointAngles(i))
end
R =  getEulerOrientation( Euler(1), Euler(2), Euler(3) );
a = R(1:3,3);
[a_wf, q_wf] = Base2World(a,q',R_base_in_world,p_base_in_world);
S(:,i) = Sr(a_wf,q_wf); %% ++++++ NEEDED +++++++ %%

end

%====================================================================
% Obtain the initial pose, M_1in0, of the end effector (dummy7 frame)
%====================================================================
%%% Get the position of dummy7 frame w.r.t the world frame
[result, p] = vrep.simxGetObjectPosition(clientID,dummy_handle,-1,vrep.simx_opmode_blocking);
if result ~= vrep.simx_return_ok
	disp('could not get dummy7 position')
end
%%% Get the orientation of dummy7 w.r.t the world frame
[result , euler_angles] = vrep.simxGetObjectOrientation(clientID, dummy_handle, -1, vrep.simx_opmode_blocking);
if result ~= vrep.simx_return_ok
	disp('could not get dummy7 orientation')
end
p_end = p';
R_end = getEulerOrientation( euler_angles(1),euler_angles(2),euler_angles(3) );
M_1in0 = getH(R_end,p_end); %% ++++++ NEEDED +++++++ %%


%====================================================================
% Obtain the desired pose, M_2in0, of the end effector (dummy7 frame,
% which is the same as the pose of the "dummy")
%====================================================================
%%% Get the position of dummy7 frame w.r.t the world frame
[result, p] = vrep.simxGetObjectPosition(clientID,dummy_ball_handle,-1,vrep.simx_opmode_blocking);
if result ~= vrep.simx_return_ok
	disp('could not get dummy7 position')
end

%%% Get the orientation of dummy7 w.r.t the world frame
[result , euler_angles] = vrep.simxGetObjectOrientation(clientID, dummy_ball_handle, -1, vrep.simx_opmode_blocking);
if result ~= vrep.simx_return_ok
	disp('could not get dummy7 orientation')
end
p_2in0 = p';
R_2in0 = getEulerOrientation( euler_angles(1),euler_angles(2),euler_angles(3) );
M_2in0 = getH(R_2in0,p_2in0); %% ++++++ NEEDED +++++++ %%

%====================================================================
% Thetas Inv kinematics
%====================================================================
%%% Get the current value of the current joint2
[result, theta2] = vrep.simxGetJointPosition(clientID, joint_handles(2), vrep.simx_opmode_blocking);
if result ~= vrep.simx_return_ok
	sprintf('could not get joint variable: theta%d', i)
end
%%% Get the current value of the current joint3
[result, theta3] = vrep.simxGetJointPosition(clientID, joint_handles(3), vrep.simx_opmode_blocking);
if result ~= vrep.simx_return_ok
	sprintf('could not get joint variable: theta%d', i)
end
loop = "true";
msg = 0;
while loop == "true" && msg == 0
    [ joint_angles, msg] = inverseKinematics(M_1in0,M_2in0,S);
    t2 = rad2deg( theta2+joint_angles(2) );
    t3 = rad2deg( theta3+joint_angles(3) );
    if msg == 0
        if t2 < 47 || t2 > 313
            loop = "true";
        elseif t3 < 19 || t3 > 341
            loop = "true";
        else 
            loop = "false";
        end
    end
end
%%% Iterate the following code block 6 times to move each joint individually
if msg == 0
    for i = 1:6

        curr_handle = joint_handles(i);

        %%% Get the current value of the current joint
        [result, theta] = vrep.simxGetJointPosition(clientID, curr_handle, vrep.simx_opmode_blocking);
        if result ~= vrep.simx_return_ok
            sprintf('could not get joint variable: theta%d', i)
        end

        %%% Wait two seconds
        pause(0.05)

        %%% Set the desired value of the current joint variable
        vrep.simxSetJointTargetPosition(clientID, curr_handle, theta + joint_angles(i), vrep.simx_opmode_oneshot);

        %%% Wait two seconds
        pause(0.05)

        %%% Get the new value of the current joint
        [result, theta] = vrep.simxGetJointPosition(clientID, curr_handle, vrep.simx_opmode_blocking);
        if result ~= vrep.simx_return_ok
            sprintf('could not get joint variable %d',i)
        end
        JointAngles(i) = theta;
        %sprintf('current value of joint variable: theta%d = %d', i,theta)    
    end
    pause(4)
    disp(" ")
    disp("Subsequent Pose")
    collission(vrep,clientID,Dummies,[0.1 0.1 0.1 0.1 0.07 0.07 0.13 0.1 0.1 0.1]/2)
    if K_loop == 1
        pause(25)
    else
        pause(3)
    end
    
end
pause(3)
end
%====================================================================
% Stop Simulation & Finish session
%====================================================================
vrep.simxStopSimulation(clientID, vrep.simx_opmode_oneshot);
vrep.simxGetPingTime(clientID);
vrep.simxFinish(clientID);
