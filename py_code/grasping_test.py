import vrep
import time
import numpy as np
import random
from scipy.linalg import expm, logm
from get_handle import GetJointsHandle
from pprint import pprint
from helper_functions import skew_3, skew_6, skew2twist, adjoint, revolute_S, euler2mat, deg2rad
from helper_functions import rad2deg, euler_to_screw_axis, base_to_world_frame_a_q
from helper_functions import T_from_R_p, rot2euler, get_S, forward_kinematics, isMtxSame
from helper_functions import inverse_kinematics, OptimizeJointAngles


# Close all open connections (just in case)
vrep.simxFinish(-1)

# Connect to V-REP (raise exception on failure)
clientID = vrep.simxStart('127.0.0.1', 19997, True, True, 5000, 5)
if clientID == -1:
    raise Exception('Failed connecting to remote API server')


# Start simulation
vrep.simxStartSimulation(clientID, vrep.simx_opmode_oneshot)


joint_handles = GetJointsHandle(clientID)

# a) In the initialization phase, retrieve some handles:
result, connector = vrep.simxGetObjectHandle(clientID, 'JacoHand_attachPoint', vrep.simx_opmode_blocking)
if result != vrep.simx_return_ok:
    raise Exception('could not get connector handle')

result, objectSensor = vrep.simxGetObjectHandle(clientID, 'JacoHand_attachProxSensor', vrep.simx_opmode_blocking)
if result != vrep.simx_return_ok:
    raise Exception('could not get objectSensor handle')

# b) Before closing the gripper, check which dynamically non-static and respondable object is in-between the fingers. Then attach the object to the gripper:
result, sphere = vrep.simxGetObjectHandle(clientID, 'Sphere', vrep.simx_opmode_blocking)
if result != vrep.simx_return_ok:
    raise Exception('could not get Sphere handle')


# obtain the initial zero position angles for the six joints
joints_zero_pos = np.zeros(6)
for i in range(6):
    # Get the current value of the current joint
    result, theta = vrep.simxGetJointPosition(clientID, joint_handles[i], vrep.simx_opmode_blocking)
    if result != vrep.simx_return_ok:
        raise Exception('could not get joint variable #{}'.format(i+1))
    joints_zero_pos[i] = theta


# Obtain the initial pose of the end effector (dummy7 frame)
# Get the handle of dummy7 object
result, dummy_handle = vrep.simxGetObjectHandle(clientID, 'JacoHand_Dummy7', vrep.simx_opmode_blocking)
if result != vrep.simx_return_ok:
    raise Exception('could not get dummy7 handle')

# Get the position of dummy7 frame w.r.t the world frame
result, p = vrep.simxGetObjectPosition(clientID,dummy_handle,-1,vrep.simx_opmode_blocking)
if result != vrep.simx_return_ok:
    raise Exception('could not get dummy7 position')

# Get the orientation of dummy7 w.r.t the world frame
result , euler_angles = vrep.simxGetObjectOrientation(clientID, dummy_handle, -1, vrep.simx_opmode_blocking)
if result != vrep.simx_return_ok:
    raise Exception('could not get dummy7 orientation')

p_end = np.reshape(p,(3,1))
R_end = euler2mat(euler_angles[0],euler_angles[1],euler_angles[2])
M = T_from_R_p(R_end,p_end)

# Get the initial goal pose
result, dummy_ball_handle = vrep.simxGetObjectHandle(clientID, 'Dummy', vrep.simx_opmode_blocking)
if result != vrep.simx_return_ok:
    raise Exception('could not get dummy ball handle')

result, p = vrep.simxGetObjectPosition(clientID,sphere,-1,vrep.simx_opmode_blocking)
if result != vrep.simx_return_ok:
    raise Exception('could not get sphere position')

# Get the orientation of sphere w.r.t the world frame
result , euler_angles = vrep.simxGetObjectOrientation(clientID, sphere, -1, vrep.simx_opmode_blocking)
if result != vrep.simx_return_ok:
    raise Exception('could not get sphere orientation')

sphere_p = np.reshape(p,(3,1))
sphere_R = euler2mat(euler_angles[0],euler_angles[1],euler_angles[2])


# Get the R and p for the base frame (Jaco) w.r.t the world frame
# Get the handle of Jaco object
result, jaco_handle = vrep.simxGetObjectHandle(clientID, 'Jaco', vrep.simx_opmode_blocking)
if result != vrep.simx_return_ok:
    raise Exception('could not get Jaco handle')

# Get the position of base frame w.r.t the world frame
result, p = vrep.simxGetObjectPosition(clientID,jaco_handle,-1,vrep.simx_opmode_blocking)
if result != vrep.simx_return_ok:
    raise Exception('could not get Jaco position')

# Get the orientation of base w.r.t the world frame
result , euler_angles = vrep.simxGetObjectOrientation(clientID, jaco_handle, -1, vrep.simx_opmode_blocking)
if result != vrep.simx_return_ok:
    raise Exception('could not get Jaco orientation')

R_base_in_world = euler2mat(euler_angles[0],euler_angles[1],euler_angles[2])
p_base_in_world = np.reshape(p,(3,1))

# get S - screw axes
S1,S2,S3,S4,S5,S6 = get_S(R_base_in_world,p_base_in_world)

result,j0 = vrep.simxGetObjectHandle(clientID,'JacoHand_fingers12_motor1',vrep.simx_opmode_blocking)
if result != vrep.simx_return_ok:
    raise Exception('could not get handle 1')
result,j1 = vrep.simxGetObjectHandle(clientID,'JacoHand_fingers12_motor2',vrep.simx_opmode_blocking)
if result != vrep.simx_return_ok:
    raise Exception('could not get handle 2')
result,j2 = vrep.simxGetObjectHandle(clientID,'JacoHand_finger3_motor1',vrep.simx_opmode_blocking)
if result != vrep.simx_return_ok:
    raise Exception('could not get handle 3')
result,j3 = vrep.simxGetObjectHandle(clientID,'JacoHand_finger3_motor2',vrep.simx_opmode_blocking)
if result != vrep.simx_return_ok:
    raise Exception('could not get handle 4')

closingVel = -0.04


sphere_p[2][0] = sphere_p[2][0] + 0.07
first_goal_pose = T_from_R_p(sphere_R,sphere_p)

error, joint_angles = inverse_kinematics(clientID,M,first_goal_pose,M,joints_zero_pos,joint_handles,S1,S2,S3,S4,S5,S6)
joint_angles = OptimizeJointAngles(joint_angles)
joint_angles = joint_angles + joints_zero_pos
joint_angles = joint_angles % (np.pi*2)

if error == 1:
    pprint('Cannot find an inverse kinematics solution within 100 iterations')
elif rad2deg(joint_angles[1]) < 47 or rad2deg(joint_angles[1]) > 313:
    pprint('Joint 2 out of valid range')
    pprint(joint_angles[1])
elif rad2deg(joint_angles[2]) < 19 or rad2deg(joint_angles[2]) > 341:
    pprint('Joint 3 out of valid range')
    pprint(joint_angles[2])
else:
    for i in range(6):
        vrep.simxSetJointTargetPosition(clientID, joint_handles[i], joint_angles[i], vrep.simx_opmode_oneshot)
        # Wait two seconds
        time.sleep(0.1)

time.sleep(1)

# obtain the current pose of the tool frame:
# Get the position of dummy7 frame w.r.t the world frame
result, p = vrep.simxGetObjectPosition(clientID,dummy_handle,-1,vrep.simx_opmode_blocking)
if result != vrep.simx_return_ok:
    raise Exception('could not get dummy7 position')

# Get the orientation of dummy7 w.r.t the world frame
result , euler_angles = vrep.simxGetObjectOrientation(clientID, dummy_handle, -1, vrep.simx_opmode_blocking)
if result != vrep.simx_return_ok:
    raise Exception('could not get dummy7 orientation')

p_end = np.reshape(p,(3,1))
R_end = euler2mat(euler_angles[0],euler_angles[1],euler_angles[2])
curr_pose = T_from_R_p(R_end,p_end)

sphere_p[2][0] = sphere_p[2][0] - 0.07
goal_pose = T_from_R_p(sphere_R,sphere_p)

# Use the IK function to find the desired joint angles
error, joint_angles = inverse_kinematics(clientID,M,goal_pose,curr_pose,joints_zero_pos,joint_handles,S1,S2,S3,S4,S5,S6)
joint_angles = OptimizeJointAngles(joint_angles)
joint_angles = joint_angles + joints_zero_pos
joint_angles = joint_angles % (np.pi*2)

if error == 1:
    pprint('Cannot find an inverse kinematics solution within 100 iterations')
elif rad2deg(joint_angles[1]) < 47 or rad2deg(joint_angles[1]) > 313:
    pprint('Joint 2 out of valid range')
    pprint(joint_angles[1])
elif rad2deg(joint_angles[2]) < 19 or rad2deg(joint_angles[2]) > 341:
    pprint('Joint 3 out of valid range')
    pprint(joint_angles[2])
else:
    for i in range(6):
        vrep.simxSetJointTargetPosition(clientID, joint_handles[i], joint_angles[i], vrep.simx_opmode_oneshot)
        # Wait two seconds
        time.sleep(0.1)


time.sleep(2)

#if (simGetObjectInt32Parameter(shape,sim_shapeintparam_static)==0) and (simGetObjectInt32Parameter(shape,sim_shapeintparam_respondable)~=0) and (simCheckProximitySensor(objectSensor,shape)==1) then
# Ok, we found a non-static respondable shape that was detected
#      attachedShape=shape
# Do the connection:
#result = vrep.simxSetObjectParent(clientID,sphere,connector,True,vrep.simx_opmode_blocking)
#if result != vrep.simx_return_ok:
#    raise Exception('could not set parent')


# Close the gripper
result = vrep.simxSetJointTargetVelocity(clientID,j0,closingVel,vrep.simx_opmode_blocking)
if result != vrep.simx_return_ok:
    raise Exception('could not set joint velocity')
result = vrep.simxSetJointTargetVelocity(clientID,j1,closingVel,vrep.simx_opmode_blocking)
if result != vrep.simx_return_ok:
    raise Exception('could not set joint velocity')
result = vrep.simxSetJointTargetVelocity(clientID,j2,closingVel,vrep.simx_opmode_blocking)
if result != vrep.simx_return_ok:
    raise Exception('could not set joint velocity')
result = vrep.simxSetJointTargetVelocity(clientID,j3,closingVel,vrep.simx_opmode_blocking)
if result != vrep.simx_return_ok:
    raise Exception('could not set joint velocity')

# c) And just before opening the gripper again, detach the previously attached shape:
#simSetObjectParent(attachedShape,-1,true)

time.sleep(2)

result, p = vrep.simxGetObjectPosition(clientID,dummy_ball_handle,-1,vrep.simx_opmode_blocking)
if result != vrep.simx_return_ok:
    raise Exception('could not get dummy ball position')

# Get the orientation of sphere w.r.t the world frame
result , euler_angles = vrep.simxGetObjectOrientation(clientID, dummy_ball_handle, -1, vrep.simx_opmode_blocking)
if result != vrep.simx_return_ok:
    raise Exception('could not get dummy ball orientation')

p_end = np.reshape(p,(3,1))
R_end = euler2mat(euler_angles[0],euler_angles[1],euler_angles[2])
curr_goal_pose = T_from_R_p(R_end,p_end)

error, joint_angles = inverse_kinematics(clientID,M,curr_goal_pose,M,joints_zero_pos,joint_handles,S1,S2,S3,S4,S5,S6)
joint_angles = OptimizeJointAngles(joint_angles)
joint_angles = joint_angles + joints_zero_pos
joint_angles = joint_angles % (np.pi*2)
if error == 1:
    pprint('Cannot find an inverse kinematics solution within 100 iterations')
elif rad2deg(joint_angles[1]) < 47 or rad2deg(joint_angles[1]) > 313:
    pprint('Joint 2 out of valid range')
    pprint(joint_angles[1])
elif rad2deg(joint_angles[2]) < 19 or rad2deg(joint_angles[2]) > 341:
    pprint('Joint 3 out of valid range')
    pprint(joint_angles[2])
else:
    for i in range(6):
        vrep.simxSetJointTargetPosition(clientID, joint_handles[i], joint_angles[i], vrep.simx_opmode_oneshot)
        # Wait two seconds
        time.sleep(0.5)

time.sleep(2)

#result = vrep.simxSetObjectParent(clientID,sphere,-1,True,vrep.simx_opmode_blocking)
#if result != vrep.simx_return_ok:
#    raise Exception('could not remove parent')
# Open the gripper
result = vrep.simxSetJointTargetVelocity(clientID,j0,-closingVel,vrep.simx_opmode_blocking)
if result != vrep.simx_return_ok:
    raise Exception('could not set joint velocity')
result = vrep.simxSetJointTargetVelocity(clientID,j1,-closingVel,vrep.simx_opmode_blocking)
if result != vrep.simx_return_ok:
    raise Exception('could not set joint velocity')
result = vrep.simxSetJointTargetVelocity(clientID,j2,-closingVel,vrep.simx_opmode_blocking)
if result != vrep.simx_return_ok:
    raise Exception('could not set joint velocity')
result = vrep.simxSetJointTargetVelocity(clientID,j3,-closingVel,vrep.simx_opmode_blocking)
if result != vrep.simx_return_ok:
    raise Exception('could not set joint velocity')

time.sleep(2)

# Stop simulation
vrep.simxStopSimulation(clientID, vrep.simx_opmode_oneshot)

# Before closing the connection to V-REP, make sure that the last command sent out had time to arrive. You can guarantee this with (for example):
vrep.simxGetPingTime(clientID)

# Close the connection to V-REP
vrep.simxFinish(clientID)






