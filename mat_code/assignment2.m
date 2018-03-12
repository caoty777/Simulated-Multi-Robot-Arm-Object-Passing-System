clear
clc

%====================================================================
% My Variables
%====================================================================
ArmJoints = ['Jaco_joint1','Jaco_joint2','Jaco_joint3',...
             'Jaco_joint4','Jaco_joint5','Jaco_joint6'];
joint_handles = zeros(1,6);
JointAngles = zeros(1,6);
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
% Forward Kinematis
%====================================================================
%%% A set of joint angles are given
joint_1_angle = pi/2;
joint_2_angle = pi/2;
joint_3_angle = pi/2;
joint_4_angle = pi/2;
joint_5_angle = pi/2;
joint_6_angle = pi/2;
joint_angles = [joint_1_angle,joint_2_angle,joint_3_angle,joint_4_angle,joint_5_angle,joint_6_angle];

%%% Get the R and p for the base frame (Jaco) w.r.t the world frame
%%% Get the handle of Jaco object
[result, jaco_handle] = vrep.simxGetObjectHandle(clientID, 'Jaco', vrep.simx_opmode_blocking);
if result ~= vrep.simx_return_ok
	disp('could not get Jaco handle')
end

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

%%% First, find the screw axes for all 6 joints
%%% a,q without fk are w.r.t the base frame
%%% a_fk and q_fk are w.r.t the world frame
a1 =  getEulerOrientation( deg2rad(-1.8000e+02),deg2rad(5.4277e-14),deg2rad(-3.4151e-06)) ;
a1 = a1(1:3,3);
q1 = [-3.2723e-05,-1.7315e-05,7.8382e-02]';
[a1_wf, q1_wf] = Base2World(a1,q1,R_base_in_world,p_base_in_world);
S1 = Sr(a1_wf,q1_wf);

a2 =  getEulerOrientation( deg2rad(-9.0000e+01),deg2rad(-7.2445e-06),deg2rad(2.4148e-06)) ;
a2 = a2(1:3,3);
q2 = [-3.2693e-05,-1.7308e-05,1.9713e-01]';
[a2_wf, q2_wf] = Base2World(a2,q2,R_base_in_world,p_base_in_world);
S2 = Sr(a2_wf,q2_wf);

a3 = getEulerOrientation( deg2rad(9.0000e+01),deg2rad(9.6593e-06),deg2rad(9.6593e-06)) ;
a3 = a3(1:3,3);
q3 = [-3.2663e-05,-1.7196e-05,6.0713e-01]';
[a3_wf, q3_wf] = Base2World(a3,q3,R_base_in_world,p_base_in_world);
S3 = Sr(a3_wf,q3_wf);

a4 = getEulerOrientation( deg2rad(1.8000e+02),deg2rad(1.6461e-05),deg2rad(-1.0245e-05)) ;
a4 = a4(1:3,3);
q4 = [-3.2663e-05,9.7829e-03,8.1444e-01]';
[a4_wf, q4_wf] = Base2World(a4,q4,R_base_in_world,p_base_in_world);
S4 = Sr(a4_wf,q4_wf);

a5 = getEulerOrientation( deg2rad(1.2500e+02),deg2rad(1.5084e-05),deg2rad(1.1702e-05)) ;
a5 = a5(1:3,3);
q5 = [-3.2663e-05,4.4032e-02,8.8027e-01]';
[a5_wf, q5_wf] = Base2World(a5,q5,R_base_in_world,p_base_in_world);
S5 = Sr(a5_wf,q5_wf);

a6 = getEulerOrientation( deg2rad(7.0000e+01),deg2rad(1.4016e-06),deg2rad(1.9864e-05)) ;
a6 = a6(1:3,3);
q6 = [-3.2723e-05,1.1769e-01,8.9004e-01]';
[a6_wf, q6_wf] = Base2World(a6,q6,R_base_in_world,p_base_in_world);
S6 = Sr(a6_wf,q6_wf);

S = [S1 S2 S3 S4 S5 S6];

%%% Second, obtain the initial pose of the end effector (dummy7 frame)
%%% Get the handle of dummy7 object
[result, dummy_handle] = vrep.simxGetObjectHandle(clientID, 'JacoHand_Dummy7', vrep.simx_opmode_blocking);
if result ~= vrep.simx_return_ok
	disp('could not get dummy7 handle')
end

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
M = getH(R_end,p_end);


%%% Third, use the matrix exponential mtehod to write down FK formulas
final_T = getExOrientation(M,joint_angles,S)

%%% Put a dummy object to the predicted location to show the predicted final pose of the tool frame
%%% First, get the R and p from T
final_R = getR(final_T);
final_p = getP(final_T);
[final_a,final_b,final_g] = Rot2Euler(final_R);
final_euler = [final_a, final_b, final_g ];

[result, dummy_ball_handle] = vrep.simxGetObjectHandle(clientID, 'Dummy', vrep.simx_opmode_blocking);
if result ~= vrep.simx_return_ok
	disp('could not get dummy ball handle')
end

pause(1)

result = vrep.simxSetObjectOrientation(clientID,dummy_ball_handle,-1,final_euler,vrep.simx_opmode_blocking);
if result ~= vrep.simx_return_ok
	disp('could not set dummy ball orientation')
end

pause(1)

result = vrep.simxSetObjectPosition(clientID,dummy_ball_handle,-1,final_p,vrep.simx_opmode_blocking);
if result ~= vrep.simx_return_ok
	disp('could not set dummy ball position')
end

pause(1)

%====================================================================
% Get "handle" to joints of robot
%====================================================================
for i = 0:5
    [result, joint_handles(i+1)] = vrep.simxGetObjectHandle(clientID, ArmJoints(11*i+1:11*i+11), vrep.simx_opmode_blocking);
    if result ~= vrep.simx_return_ok
        sprintf('could not get joint handle%d = %d', i,ArmJoints(11*i+1:11*i+11))
    end
end

pause(1)

%%% Iterate the following code block 6 times to move each joint individually
for i = 1:6
    
    curr_handle = joint_handles(i);

	%%% Get the current value of the current joint
	[result, theta] = vrep.simxGetJointPosition(clientID, curr_handle, vrep.simx_opmode_blocking);
	if result ~= vrep.simx_return_ok
		sprintf('could not get joint variable: theta%d', i)
    end

	%%% Wait two seconds
	pause(1)

	%%% Set the desired value of the current joint variable
	vrep.simxSetJointTargetPosition(clientID, curr_handle, theta + joint_angles(i), vrep.simx_opmode_oneshot);

	%%% Wait two seconds
	pause(1)

	%%% Get the new value of the current joint
	[result, theta] = vrep.simxGetJointPosition(clientID, curr_handle, vrep.simx_opmode_blocking);
	if result ~= vrep.simx_return_ok
		sprintf('could not get joint variable %d',i)
    end
    JointAngles(i) = theta;
    %sprintf('current value of joint variable: theta%d = %d', i,theta)    
end

pause(1)

%%% Print the actual pose of tool frame (dummy7) in the end
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
R_end = getEulerOrientation(euler_angles(1),euler_angles(2),euler_angles(3));
T_actual = getH(R_end,p_end)

%====================================================================
% Stop Simulation & Finish session
%====================================================================
vrep.simxStopSimulation(clientID, vrep.simx_opmode_oneshot);
vrep.simxGetPingTime(clientID);
vrep.simxFinish(clientID);
